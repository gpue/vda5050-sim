"""Transport abstraction: the same Fleet/SimulatedAgv logic can speak either

- NATS (`vda5050.v3.{manufacturer}.{serial}.*`) — wire-compatible with
  nova-nav's dashboard, for running as a Nova app; or
- plain MQTT (`vda5050/v3/{manufacturer}/{serial}/*`) — the real VDA5050
  wire protocol, so any standard fleet manager/master control can connect
  directly with no Nova platform or NATS bus involved.
"""

from __future__ import annotations

import abc
import asyncio
import json
import logging
from collections.abc import Awaitable, Callable

from nats.aio.client import Client as NATS
from nova_vda5050 import InstantActionsMessage, OrderMessage
from pydantic import BaseModel

logger = logging.getLogger("vda5050_sim.transport")


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
    async def publish(self, manufacturer: str, serial_number: str, message_type: str, model: BaseModel) -> None: ...

    @abc.abstractmethod
    async def subscribe_order(
        self, manufacturer: str, serial_number: str, handler: Callable[[OrderMessage], Awaitable[None]]
    ) -> Subscription: ...

    @abc.abstractmethod
    async def subscribe_instant_actions(
        self, manufacturer: str, serial_number: str, handler: Callable[[InstantActionsMessage], Awaitable[None]]
    ) -> Subscription: ...


def _dump(model: BaseModel) -> bytes:
    # exclude_none: unset Optional/V2-compat fields (agvPosition, batteryState,
    # localizationScore, ...) must be omitted, not sent as JSON null — the
    # VDA5050 JSON Schemas reject `null` for most of these.
    return json.dumps(model.model_dump(mode="json", exclude_none=True)).encode("utf-8")


# ── NATS (Nova app mode) ─────────────────────────────────────────────────────


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

    async def publish(self, manufacturer: str, serial_number: str, message_type: str, model: BaseModel) -> None:
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


# ── MQTT (standalone mode) ───────────────────────────────────────────────────


class _MqttSubscription(Subscription):
    def __init__(self, transport: MqttTransport, topic: str) -> None:
        self._transport = transport
        self._topic = topic

    async def unsubscribe(self) -> None:
        await self._transport._unsubscribe(self._topic)


class MqttTransport(Transport):
    """Plain VDA5050-over-MQTT — no Nova/NATS involved.

    Any real fleet manager / master control that speaks standard VDA5050 MQTT
    can connect to the same broker and control these simulated robots
    directly.
    """

    def __init__(
        self,
        host: str,
        port: int,
        *,
        username: str | None = None,
        password: str | None = None,
        tls: bool = False,
        topic_prefix: str = "vda5050/v3",
    ) -> None:
        self._host = host
        self._port = port
        self._username = username
        self._password = password
        self._tls = tls
        self.topic_prefix = topic_prefix
        self._client_ctx = None
        self._client = None
        self._handlers: dict[str, tuple[Callable, type]] = {}
        self._listen_task: asyncio.Task | None = None

    def _topic(self, manufacturer: str, serial_number: str, message_type: str) -> str:
        return f"{self.topic_prefix}/{manufacturer}/{serial_number}/{message_type}"

    async def connect(self) -> None:
        import aiomqtt

        tls_params = aiomqtt.TLSParameters() if self._tls else None
        self._client_ctx = aiomqtt.Client(
            hostname=self._host,
            port=self._port,
            username=self._username,
            password=self._password,
            tls_params=tls_params,
        )
        self._client = await self._client_ctx.__aenter__()
        self._listen_task = asyncio.create_task(self._listen())

    async def close(self) -> None:
        if self._listen_task is not None:
            self._listen_task.cancel()
        if self._client_ctx is not None:
            await self._client_ctx.__aexit__(None, None, None)

    async def publish(self, manufacturer: str, serial_number: str, message_type: str, model: BaseModel) -> None:
        assert self._client is not None
        await self._client.publish(self._topic(manufacturer, serial_number, message_type), payload=_dump(model), qos=1)

    async def subscribe_order(self, manufacturer, serial_number, handler) -> Subscription:
        topic = self._topic(manufacturer, serial_number, "order")
        assert self._client is not None
        await self._client.subscribe(topic)
        self._handlers[topic] = (handler, OrderMessage)
        return _MqttSubscription(self, topic)

    async def subscribe_instant_actions(self, manufacturer, serial_number, handler) -> Subscription:
        topic = self._topic(manufacturer, serial_number, "instantActions")
        assert self._client is not None
        await self._client.subscribe(topic)
        self._handlers[topic] = (handler, InstantActionsMessage)
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


def build_transport(settings) -> Transport:
    if settings.transport == "mqtt":
        return MqttTransport(
            settings.mqtt_host,
            settings.mqtt_port,
            username=settings.mqtt_username,
            password=settings.mqtt_password,
            tls=settings.mqtt_tls,
            topic_prefix=settings.mqtt_topic_prefix,
        )
    return NatsTransport(settings.nats_broker, settings.vda5050_prefix)
