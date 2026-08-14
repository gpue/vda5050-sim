"""Loads the fleet config and runs one AGV per configured robot over the
active Transport (NATS for Nova-app mode, MQTT for standalone mode)."""

from __future__ import annotations

import asyncio
from pathlib import Path

import yaml

from vda5050_sim.agv import FaultProfile, RobotConfig, SimulatedAgv
from vda5050_sim.config import Settings
from vda5050_sim.logbuffer import LogBuffer
from vda5050_sim.robot_specs import ROBOT_SPECS
from vda5050_sim.schemas import ConnectionState
from vda5050_sim.transport import Transport


def _resolve_path(path: str) -> Path:
    p = Path(path)
    if p.is_absolute() or p.exists():
        return p
    candidate = Path(__file__).resolve().parent.parent.parent / path
    return candidate if candidate.exists() else p


def load_fleet_config(path: str) -> list[RobotConfig]:
    data = yaml.safe_load(_resolve_path(path).read_text())
    robots: list[RobotConfig] = []
    for entry in data.get("robots", []):
        fault = entry.get("fault_profile") or {}
        model = entry["model"]
        # Fall back to the real robot's published top speed (this repo's own
        # ROBOT_SPECS registry) instead of inventing one, unless overridden.
        max_speed = entry.get("max_speed")
        if max_speed is None:
            max_speed = ROBOT_SPECS.get(model, {}).get("maximumSpeed")
        robots.append(
            RobotConfig(
                id=entry["id"],
                model=model,
                manufacturer=entry.get("manufacturer", ""),
                supported_actions=list(entry.get("supported_actions", [])),
                max_speed=max_speed,
                initial_battery=float(entry.get("initial_battery", 100.0)),
                fault_profile=FaultProfile(
                    connection_drop_probability=float(fault.get("connection_drop_probability", 0.0)),
                    error_injection_probability=float(fault.get("error_injection_probability", 0.0)),
                ),
            )
        )
    return robots


class RobotRuntime:
    """Owns one SimulatedAgv plus its transport subscriptions and publish loops."""

    def __init__(self, agv: SimulatedAgv, transport: Transport, settings: Settings) -> None:
        self.agv = agv
        self.transport = transport
        self.settings = settings
        self._tasks: list[asyncio.Task] = []
        self._subs: list = []
        # referenceStateHeaderId is required (not optional) in v3.0.0's
        # visualization schema — start at 0 in case visualization's first
        # tick races ahead of state's first publish.
        self._last_state_header_id: int = 0
        self.online = False

    async def start(self) -> None:
        self._subs.append(
            await self.transport.subscribe_order(self.agv.manufacturer, self.agv.serial_number, self._on_order)
        )
        self._subs.append(
            await self.transport.subscribe_instant_actions(
                self.agv.manufacturer, self.agv.serial_number, self._on_instant_actions
            )
        )
        self._tasks = [
            asyncio.create_task(self._movement_loop()),
            asyncio.create_task(self._connection_loop()),
            asyncio.create_task(self._state_loop()),
            asyncio.create_task(self._visualization_loop()),
        ]

    async def stop(self) -> None:
        for t in self._tasks:
            t.cancel()
        for s in self._subs:
            await s.unsubscribe()

    async def _on_order(self, order) -> None:
        self.agv.handle_order(order)

    async def _on_instant_actions(self, msg) -> None:
        self.agv.handle_instant_actions(msg)

    async def _publish(self, message_type: str, model) -> None:
        await self.transport.publish(self.agv.manufacturer, self.agv.serial_number, message_type, model)

    async def _movement_loop(self) -> None:
        tick_s = self.settings.tick_s
        while True:
            self.agv.tick(tick_s)
            self.agv.maybe_inject_fault()
            await asyncio.sleep(tick_s)

    async def _connection_loop(self) -> None:
        await self._publish("connection", self.agv.build_connection_message(ConnectionState.CONNECTION_BROKEN))
        await asyncio.sleep(0.2)
        self.online = True
        await self._publish("connection", self.agv.build_connection_message(ConnectionState.ONLINE))
        while True:
            await asyncio.sleep(self.settings.connection_heartbeat_s)
            if self.agv.should_drop_connection():
                self.online = False
                await self._publish("connection", self.agv.build_connection_message(ConnectionState.CONNECTION_BROKEN))
                await asyncio.sleep(self.settings.connection_heartbeat_s)
                self.online = True
            await self._publish("connection", self.agv.build_connection_message(ConnectionState.ONLINE))

    async def _state_loop(self) -> None:
        interval = 1.0 / self.settings.state_hz
        while True:
            if self.agv.factsheet_requested:
                self.agv.factsheet_requested = False
                await self._publish("factsheet", self.agv.build_factsheet_message())
            state = self.agv.build_state_message()
            self._last_state_header_id = state.headerId
            await self._publish("state", state)
            await asyncio.sleep(interval)

    async def _visualization_loop(self) -> None:
        interval = 1.0 / self.settings.visualization_hz
        while True:
            await self._publish("visualization", self.agv.build_visualization_message(self._last_state_header_id))
            await asyncio.sleep(interval)


class Fleet:
    def __init__(self, settings: Settings, transport: Transport, log_buffer: LogBuffer) -> None:
        self.settings = settings
        self.transport = transport
        self.log_buffer = log_buffer
        self.runtimes: dict[str, RobotRuntime] = {}

    async def start(self, configs: list[RobotConfig]) -> None:
        for cfg in configs:
            agv = SimulatedAgv(
                cfg,
                action_duration_s=self.settings.action_duration_s,
                default_speed_mps=self.settings.default_speed_mps,
                on_log=self.log_buffer.add,
            )
            runtime = RobotRuntime(agv, self.transport, self.settings)
            self.runtimes[cfg.id] = runtime
            await runtime.start()

    async def stop(self) -> None:
        for runtime in self.runtimes.values():
            await runtime.stop()

    def snapshot(self) -> list[dict]:
        out = []
        for rid, runtime in self.runtimes.items():
            agv = runtime.agv
            out.append(
                {
                    "id": rid,
                    "model": agv.cfg.model,
                    "manufacturer": agv.manufacturer,
                    "online": runtime.online,
                    "orderId": agv.order_id,
                    "orderUpdateId": agv.order_update_id,
                    "driving": agv.driving,
                    "paused": agv.paused,
                    "idle": agv.is_idle(),
                    "position": {"x": agv.x, "y": agv.y, "theta": agv.theta, "mapId": agv.map_id},
                    "battery": agv.battery_soc,
                }
            )
        return out
