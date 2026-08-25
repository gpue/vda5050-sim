"""Transport abstraction: the same Fleet/SimulatedAgv logic can speak either

- plain MQTT (`vda5050/v3/{manufacturer}/{serial}/*`) — the real VDA5050
  wire protocol, so any standard fleet manager/master control can connect
  directly with no other infrastructure required; or
- NATS (`vda5050.v3.{manufacturer}.{serial}.*`) — for teams whose own stack
  already runs on a NATS message bus instead of MQTT.

MQTT mode uses one `aiomqtt.Client` connection *per robot*, each with its own
Last-Will-Testament — this matches how real VDA5050 AGVs actually connect
(each is its own MQTT client) and is what makes per-robot LWT possible: a
single shared connection for a whole simulated fleet could only ever carry
one Will, which can't represent N independently-crashable robots. NATS mode
keeps one shared connection (no retain/LWT concept in core NATS to motivate
the split there) — see `build_transport_factory` for how the two modes wire
up differently while `Fleet`/`RobotRuntime` stay transport-agnostic.
"""

from __future__ import annotations

import abc
import asyncio
import json
import logging
from collections.abc import Awaitable, Callable
from datetime import UTC, datetime
from typing import TYPE_CHECKING

from nats.aio.client import Client as NATS
from pydantic import BaseModel

from vda5050_sim.schemas import (
    ConnectionMessage,
    ConnectionState,
    InstantActionsMessage,
    OrderMessage,
    ResponsesMessage,
    ZoneSetMessage,
)

if TYPE_CHECKING:
    from vda5050_sim.agv import RobotConfig
    from vda5050_sim.config import Settings

logger = logging.getLogger("vda5050_sim.transport")

# Retained message types persist on the broker so a fleet manager that
# subscribes late still immediately sees last-known state — matches real
# VDA5050 wire behavior. `visualization` is deliberately excluded: it's
# high-rate/ephemeral by design, retaining it would be pointless and wasteful.
RETAINED_MESSAGE_TYPES = frozenset({"state", "connection", "factsheet"})


class Subscription(abc.ABC):
    @abc.abstractmethod
    async def unsubscribe(self) -> None: ...


class Transport(abc.ABC):
    """Publish/subscribe VDA5050 messages for one robot, transport-agnostic."""

    @abc.abstractmethod
    async def connect(self) -> None: ...

    @abc.abstractmethod
    async def close(self) -> None: ...

    @abc.abstractmethod
    async def publish(
        self, manufacturer: str, serial_number: str, message_type: str, model: BaseModel, *, retain: bool = False
    ) -> None: ...

    @abc.abstractmethod
    async def subscribe_order(
        self, manufacturer: str, serial_number: str, handler: Callable[[OrderMessage], Awaitable[None]]
    ) -> Subscription: ...

    @abc.abstractmethod
    async def subscribe_instant_actions(
        self, manufacturer: str, serial_number: str, handler: Callable[[InstantActionsMessage], Awaitable[None]]
    ) -> Subscription: ...

    @abc.abstractmethod
    async def subscribe_zone_set(
        self, manufacturer: str, serial_number: str, handler: Callable[[ZoneSetMessage], Awaitable[None]]
    ) -> Subscription: ...

    @abc.abstractmethod
    async def subscribe_responses(
        self, manufacturer: str, serial_number: str, handler: Callable[[ResponsesMessage], Awaitable[None]]
    ) -> Subscription: ...


def _dump(model: BaseModel) -> bytes:
    # exclude_none: unset Optional/V2-compat fields (agvPosition, batteryState,
    # localizationScore, ...) must be omitted, not sent as JSON null — the
    # VDA5050 JSON Schemas reject `null` for most of these.
    return json.dumps(model.model_dump(mode="json", exclude_none=True)).encode("utf-8")


# ── NATS (shared connection) ─────────────────────────────────────────────────


class _NatsSubscription(Subscription):
    def __init__(self, sub) -> None:
        self._sub = sub

    async def unsubscribe(self) -> None:
        await self._sub.unsubscribe()


class NatsTransport(Transport):
    def __init__(self, nats_url: str, prefix: str) -> None:
        self._nats_url = nats_url
        self.prefix = prefix
        self._nc: NATS | None = None

    def _subject(self, manufacturer: str, serial_number: str, message_type: str) -> str:
        return f"{self.prefix}.{manufacturer}.{serial_number}.{message_type}"

    async def connect(self) -> None:
        self._nc = NATS()
        await self._nc.connect(servers=[self._nats_url], max_reconnect_attempts=-1)

    async def close(self) -> None:
        if self._nc is not None:
            await self._nc.drain()

    async def publish(self, manufacturer, serial_number, message_type, model, *, retain: bool = False) -> None:
        # retain is an MQTT-only concept; core NATS has no equivalent.
        assert self._nc is not None
        await self._nc.publish(self._subject(manufacturer, serial_number, message_type), _dump(model))

    async def subscribe_order(self, manufacturer, serial_number, handler) -> Subscription:
        assert self._nc is not None
        subject = self._subject(manufacturer, serial_number, "order")

        async def _cb(msg) -> None:
            await handler(OrderMessage.model_validate_json(msg.data))

        return _NatsSubscription(await self._nc.subscribe(subject, cb=_cb))

    async def subscribe_instant_actions(self, manufacturer, serial_number, handler) -> Subscription:
        assert self._nc is not None
        subject = self._subject(manufacturer, serial_number, "instantActions")

        async def _cb(msg) -> None:
            await handler(InstantActionsMessage.model_validate_json(msg.data))

        return _NatsSubscription(await self._nc.subscribe(subject, cb=_cb))

    async def subscribe_zone_set(self, manufacturer, serial_number, handler) -> Subscription:
        assert self._nc is not None
        subject = self._subject(manufacturer, serial_number, "zoneSet")

        async def _cb(msg) -> None:
            await handler(ZoneSetMessage.model_validate_json(msg.data))

        return _NatsSubscription(await self._nc.subscribe(subject, cb=_cb))

    async def subscribe_responses(self, manufacturer, serial_number, handler) -> Subscription:
        assert self._nc is not None
        subject = self._subject(manufacturer, serial_number, "responses")

        async def _cb(msg) -> None:
            await handler(ResponsesMessage.model_validate_json(msg.data))

        return _NatsSubscription(await self._nc.subscribe(subject, cb=_cb))


# ── MQTT (one connection per robot) ─────────────────────────────────────────


class _MqttSubscription(Subscription):
    def __init__(self, transport: MqttTransport, topic: str) -> None:
        self._transport = transport
        self._topic = topic

    async def unsubscribe(self) -> None:
        await self._transport._unsubscribe(self._topic)


class MqttTransport(Transport):
    """Plain VDA5050-over-MQTT for exactly one robot — no Nova/NATS involved.

    Any real fleet manager / master control that speaks standard VDA5050 MQTT
    can connect to the same broker and control this simulated robot directly.
    One instance = one `aiomqtt.Client` = one robot, so each gets its own
    Last-Will-Testament (see `connect()`).
    """

    def __init__(
        self,
        host: str,
        port: int,
        *,
        manufacturer: str,
        serial_number: str,
        username: str | None = None,
        password: str | None = None,
        tls: bool = False,
        topic_prefix: str = "vda5050/v3",
    ) -> None:
        self._host = host
        self._port = port
        self._manufacturer = manufacturer
        self._serial_number = serial_number
        self._username = username
        self._password = password
        self._tls = tls
        self.topic_prefix = topic_prefix
        self._client_ctx = None
        self._client = None
        self._handlers: dict[str, tuple[Callable, type]] = {}
        self._listen_task: asyncio.Task | None = None

    def _topic(self, message_type: str) -> str:
        return f"{self.topic_prefix}/{self._manufacturer}/{self._serial_number}/{message_type}"

    def _will_payload(self) -> bytes:
        # Fixed sentinel headerId + a connect-time (not actual-disconnect-time)
        # timestamp — a will payload is captured once at connect and can't be
        # dynamically updated, so neither can track the real disconnect event
        # precisely. Only connectionState (CONNECTION_BROKEN) matters for a
        # fleet manager's crash-detection purposes.
        msg = ConnectionMessage(
            headerId=0,
            timestamp=datetime.now(UTC).isoformat(timespec="milliseconds"),
            manufacturer=self._manufacturer,
            serialNumber=self._serial_number,
            connectionState=ConnectionState.CONNECTION_BROKEN,
        )
        return _dump(msg)

    async def connect(self) -> None:
        import aiomqtt

        tls_params = aiomqtt.TLSParameters() if self._tls else None
        will = aiomqtt.Will(topic=self._topic("connection"), payload=self._will_payload(), qos=1, retain=True)
        self._client_ctx = aiomqtt.Client(
            hostname=self._host,
            port=self._port,
            username=self._username,
            password=self._password,
            tls_params=tls_params,
            will=will,
            identifier=f"vda5050-sim-{self._manufacturer}-{self._serial_number}",
        )
        self._client = await self._client_ctx.__aenter__()
        self._listen_task = asyncio.create_task(self._listen())

    async def close(self) -> None:
        if self._listen_task is not None:
            self._listen_task.cancel()
        if self._client_ctx is not None:
            await self._client_ctx.__aexit__(None, None, None)

    async def publish(self, manufacturer, serial_number, message_type, model, *, retain: bool = False) -> None:
        assert self._client is not None
        await self._client.publish(self._topic(message_type), payload=_dump(model), qos=1, retain=retain)

    async def subscribe_order(self, manufacturer, serial_number, handler) -> Subscription:
        topic = self._topic("order")
        assert self._client is not None
        await self._client.subscribe(topic)
        self._handlers[topic] = (handler, OrderMessage)
        return _MqttSubscription(self, topic)

    async def subscribe_instant_actions(self, manufacturer, serial_number, handler) -> Subscription:
        topic = self._topic("instantActions")
        assert self._client is not None
        await self._client.subscribe(topic)
        self._handlers[topic] = (handler, InstantActionsMessage)
        return _MqttSubscription(self, topic)

    async def subscribe_zone_set(self, manufacturer, serial_number, handler) -> Subscription:
        topic = self._topic("zoneSet")
        assert self._client is not None
        await self._client.subscribe(topic)
        self._handlers[topic] = (handler, ZoneSetMessage)
        return _MqttSubscription(self, topic)

    async def subscribe_responses(self, manufacturer, serial_number, handler) -> Subscription:
        topic = self._topic("responses")
        assert self._client is not None
        await self._client.subscribe(topic)
        self._handlers[topic] = (handler, ResponsesMessage)
        return _MqttSubscription(self, topic)

    async def _unsubscribe(self, topic: str) -> None:
        self._handlers.pop(topic, None)
        if self._client is not None:
            await self._client.unsubscribe(topic)

    async def _listen(self) -> None:
        assert self._client is not None
        async for message in self._client.messages:
            entry = self._handlers.get(str(message.topic))
            if entry is None:
                continue
            handler, cls = entry
            try:
                await handler(cls.model_validate_json(message.payload))
            except Exception:
                logger.exception("error handling MQTT message on %s", message.topic)


# ── Factory: how Fleet obtains a Transport per robot ────────────────────────

TransportFactory = Callable[["RobotConfig"], Awaitable[Transport]]


async def build_transport_factory(settings: Settings) -> TransportFactory:
    """Returns an async factory `(RobotConfig) -> Transport`, already
    connected and ready to use. NATS mode connects one shared transport once
    and hands back that same instance for every robot; MQTT mode connects a
    fresh per-robot transport (with its own LWT) on each call."""
    if settings.transport == "mqtt":

        async def _make_mqtt(cfg: RobotConfig) -> Transport:
            transport = MqttTransport(
                settings.mqtt_host,
                settings.mqtt_port,
                manufacturer=cfg.manufacturer,
                serial_number=cfg.id,
                username=settings.mqtt_username,
                password=settings.mqtt_password,
                tls=settings.mqtt_tls,
                topic_prefix=settings.mqtt_topic_prefix,
            )
            await transport.connect()
            return transport

        return _make_mqtt

    shared = NatsTransport(settings.nats_broker, settings.vda5050_prefix)
    await shared.connect()

    async def _make_nats(cfg: RobotConfig) -> Transport:
        return shared

    return _make_nats
