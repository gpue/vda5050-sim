"""Loads the fleet config and runs one AGV per configured robot over the
active Transport (NATS for Nova-app mode, MQTT for standalone mode)."""

from __future__ import annotations

import asyncio
import contextlib
from pathlib import Path

import yaml

from vda5050_sim.agv import FaultProfile, RobotConfig, SimulatedAgv
from vda5050_sim.config import Settings
from vda5050_sim.logbuffer import LogBuffer
from vda5050_sim.robot_specs import ROBOT_SPECS
from vda5050_sim.schemas import ConnectionState
from vda5050_sim.transport import RETAINED_MESSAGE_TYPES, Transport, TransportFactory


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
        # Fall back to the real robot's published top speed/turn rate (this
        # repo's own ROBOT_SPECS registry) instead of inventing one, unless overridden.
        max_speed = entry.get("max_speed")
        if max_speed is None:
            max_speed = ROBOT_SPECS.get(model, {}).get("maximumSpeed")
        angular_speed = entry.get("angular_speed")
        if angular_speed is None:
            angular_speed = ROBOT_SPECS.get(model, {}).get("maximumAngularSpeed")
        robots.append(
            RobotConfig(
                id=entry["id"],
                model=model,
                manufacturer=entry.get("manufacturer", ""),
                supported_actions=list(entry.get("supported_actions", [])),
                max_speed=max_speed,
                angular_speed=angular_speed,
                initial_battery=float(entry.get("initial_battery", 100.0)),
                battery_drain_percent_per_meter=entry.get("battery_drain_percent_per_meter"),
                battery_charge_percent_per_s=entry.get("battery_charge_percent_per_s"),
                initial_x=float(entry.get("initial_x", 0.0)),
                initial_y=float(entry.get("initial_y", 0.0)),
                initial_theta=float(entry.get("initial_theta", 0.0)),
                protocol_version=str(entry.get("protocol_version", "3.0.0")),
                fault_profile=FaultProfile(
                    connection_drop_probability=float(fault.get("connection_drop_probability", 0.0)),
                    error_injection_probability=float(fault.get("error_injection_probability", 0.0)),
                    field_violation_probability=float(fault.get("field_violation_probability", 0.0)),
                    service_mode_probability=float(fault.get("service_mode_probability", 0.0)),
                    emergency_stop_probability=float(fault.get("emergency_stop_probability", 0.0)),
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
        # NATS is a shared connection for the whole fleet (transport.prefix
        # is fixed at construction) — a per-robot subject prefix override is
        # how a robot announcing an older protocol_version ends up on
        # vda5050.v1/v2 instead of the configured default. Derived by
        # swapping the version segment on the *configured* prefix (not a
        # hardcoded "vda5050.v3") so this also works against the test
        # suite's "vda5050.v3test" prefix. None (no override) for 3.0.0
        # robots, so existing v3-only deployments/tests are untouched.
        # Ignored by MqttTransport, whose topic_prefix is already
        # version-correct per-instance (see build_transport_factory).
        self._protocol_major = agv.cfg.protocol_version.split(".", 1)[0]
        self._version_prefix = (
            None if self._protocol_major == "3" else settings.vda5050_prefix.replace("v3", f"v{self._protocol_major}", 1)
        )

    async def start(self) -> None:
        self._subs.append(
            await self.transport.subscribe_order(
                self.agv.manufacturer, self.agv.serial_number, self._on_order, version_prefix=self._version_prefix
            )
        )
        self._subs.append(
            await self.transport.subscribe_instant_actions(
                self.agv.manufacturer, self.agv.serial_number, self._on_instant_actions, version_prefix=self._version_prefix
            )
        )
        if self._protocol_major == "3":
            # zoneSet is a 3.0.0+ capability (see agv.py's
            # LEGACY_UNSUPPORTED_ACTIONS) — a legacy robot has nothing to
            # subscribe to.
            self._subs.append(
                await self.transport.subscribe_zone_set(
                    self.agv.manufacturer, self.agv.serial_number, self._on_zone_set, version_prefix=self._version_prefix
                )
            )
        self._subs.append(
            await self.transport.subscribe_responses(
                self.agv.manufacturer, self.agv.serial_number, self._on_responses, version_prefix=self._version_prefix
            )
        )
        self._tasks = [
            asyncio.create_task(self._movement_loop()),
            asyncio.create_task(self._connection_loop()),
            asyncio.create_task(self._state_loop()),
            asyncio.create_task(self._visualization_loop()),
        ]

    async def stop(self) -> None:
        # Best-effort: report an orderly disconnect before tearing down, so a
        # clean shutdown doesn't look identical to a crash to a fleet manager.
        # OFFLINE is the schema's best-fit value for "disconnecting in an
        # orderly fashion" (see schemas.py's ConnectionState enum note).
        with contextlib.suppress(Exception):
            await self._publish("connection", self.agv.build_connection_message(ConnectionState.OFFLINE))
        for t in self._tasks:
            t.cancel()
        for s in self._subs:
            # A prior self-triggered _do_shutdown() (from the `shutdown`
            # instant action) may have already unsubscribed these.
            with contextlib.suppress(Exception):
                await s.unsubscribe()

    async def _on_order(self, order) -> None:
        self.agv.handle_order(order)

    async def _on_instant_actions(self, msg) -> None:
        self.agv.handle_instant_actions(msg)

    async def _on_zone_set(self, msg) -> None:
        self.agv.handle_zone_set(msg)

    async def _on_responses(self, msg) -> None:
        self.agv.handle_responses(msg)

    async def _publish(self, message_type: str, model) -> None:
        retain = message_type in RETAINED_MESSAGE_TYPES
        await self.transport.publish(
            self.agv.manufacturer,
            self.agv.serial_number,
            message_type,
            model,
            retain=retain,
            version=self.agv.cfg.protocol_version,
            version_prefix=self._version_prefix,
        )

    async def _movement_loop(self) -> None:
        tick_s = self.settings.tick_s
        while True:
            self.agv.tick(tick_s)
            self.agv.maybe_inject_fault(tick_s)
            if self.agv.shutdown_requested:
                await self._do_shutdown()
                return
            await asyncio.sleep(tick_s)

    async def _do_shutdown(self) -> None:
        # Mirrors stop()'s orderly-disconnect publish, but self-triggered from
        # inside a loop this same call cancels — must not cancel itself.
        with contextlib.suppress(Exception):
            await self._publish("connection", self.agv.build_connection_message(ConnectionState.OFFLINE))
        current = asyncio.current_task()
        for t in self._tasks:
            if t is not current:
                t.cancel()
        for s in self._subs:
            await s.unsubscribe()

    async def force_connection_state(self, state: ConnectionState) -> None:
        """Debug-only: publish a connection message immediately instead of
        waiting for the next connection_heartbeat_s tick — see
        debug_routes.py. `online` is updated the same way _connection_loop
        derives it, so /fleet and downstream consumers agree with what was
        just published."""
        self.online = state == ConnectionState.ONLINE
        await self._publish("connection", self.agv.build_connection_message(state))

    async def _connection_loop(self) -> None:
        await self._publish("connection", self.agv.build_connection_message(ConnectionState.CONNECTION_BROKEN))
        await asyncio.sleep(0.2)
        self.online = True
        was_hibernating = False
        await self._publish("connection", self.agv.build_connection_message(ConnectionState.ONLINE))
        while True:
            await asyncio.sleep(self.settings.connection_heartbeat_s)
            if self.agv.hibernating:
                if not was_hibernating:
                    await self._publish("connection", self.agv.build_connection_message(ConnectionState.HIBERNATING))
                    was_hibernating = True
                continue
            if was_hibernating:
                await self._publish("connection", self.agv.build_connection_message(ConnectionState.ONLINE))
                was_hibernating = False
                continue
            if self.agv.should_drop_connection():
                self.online = False
                await self._publish("connection", self.agv.build_connection_message(ConnectionState.CONNECTION_BROKEN))
                await asyncio.sleep(self.settings.connection_heartbeat_s)
                self.online = True
            await self._publish("connection", self.agv.build_connection_message(ConnectionState.ONLINE))

    async def _state_loop(self) -> None:
        interval = 1.0 / self.settings.state_hz
        self.agv.state_request_event = asyncio.Event()
        while True:
            if self.agv.hibernating:
                # Spec: "no longer needs to send state messages" while
                # HIBERNATING — stop publishing until stopHibernation.
                await asyncio.sleep(min(interval, 0.5))
                continue
            if self.agv.factsheet_requested:
                self.agv.factsheet_requested = False
                await self._publish("factsheet", self.agv.build_factsheet_message())
            state = self.agv.build_state_message()
            self._last_state_header_id = state.headerId
            await self._publish("state", state)
            self.agv.state_request_event.clear()
            # stateRequest wakes this early instead of waiting out the full
            # interval; a normal timeout just falls through to the next publish.
            with contextlib.suppress(TimeoutError):
                await asyncio.wait_for(self.agv.state_request_event.wait(), timeout=interval)

    async def _visualization_loop(self) -> None:
        interval = 1.0 / self.settings.visualization_hz
        while True:
            await self._publish("visualization", self.agv.build_visualization_message(self._last_state_header_id))
            await asyncio.sleep(interval)


class Fleet:
    """Owns one RobotRuntime per configured robot. Each runtime's Transport
    comes from `transport_factory` — for NATS mode that's the same shared,
    already-connected instance every time; for MQTT mode it's a fresh
    per-robot connection (own Last-Will-Testament) each call. Either way,
    Fleet tracks each *unique* transport instance so it closes each one
    exactly once on shutdown, regardless of how many robots share it."""

    def __init__(self, settings: Settings, transport_factory: TransportFactory, log_buffer: LogBuffer) -> None:
        self.settings = settings
        self.transport_factory = transport_factory
        self.log_buffer = log_buffer
        self.runtimes: dict[str, RobotRuntime] = {}
        self._transports: dict[int, Transport] = {}

    async def start(self, configs: list[RobotConfig]) -> None:
        for cfg in configs:
            transport = await self.transport_factory(cfg)
            self._transports[id(transport)] = transport
            agv = SimulatedAgv(
                cfg,
                action_duration_s=self.settings.action_duration_s,
                default_speed_mps=self.settings.default_speed_mps,
                default_angular_speed_rad_s=self.settings.default_angular_speed_rad_s,
                horizon_threshold_nodes=self.settings.horizon_threshold_nodes,
                default_battery_drain_percent_per_meter=self.settings.default_battery_drain_percent_per_meter,
                default_battery_charge_percent_per_s=self.settings.default_battery_charge_percent_per_s,
                on_log=self.log_buffer.add,
            )
            runtime = RobotRuntime(agv, transport, self.settings)
            self.runtimes[cfg.id] = runtime
            await runtime.start()

    async def stop(self) -> None:
        for runtime in self.runtimes.values():
            await runtime.stop()
        for transport in self._transports.values():
            await transport.close()

    def find_runtime(self, manufacturer: str, serial: str) -> RobotRuntime | None:
        """Debug-only lookup by (manufacturer, serial) — see debug_routes.py.
        `runtimes` is keyed by config id, which *is* the serial number
        (SimulatedAgv.serial_number = cfg.id), so this also guards against a
        manufacturer mismatch on an id collision across configs."""
        runtime = self.runtimes.get(serial)
        if runtime is not None and runtime.agv.manufacturer == manufacturer:
            return runtime
        return None

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
