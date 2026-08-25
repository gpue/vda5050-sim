"""Phase 2: MQTT transport depth — real retain semantics and per-robot
Last-Will-Testament, verified against a real local mosquitto broker (not
mocked) via the `mqtt_broker` fixture.
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import multiprocessing
import os
import signal

import aiomqtt
import pytest

from vda5050_sim.schemas import ConnectionState, PowerSupply, StateMessage, VisualizationMessage
from vda5050_sim.transport import MqttTransport

TEST_MQTT_PREFIX = "vda5050test/v3"
MANUFACTURER, SERIAL = "TestCo", "retain-bot-01"


def _topic(manufacturer: str, serial: str, message_type: str) -> str:
    return f"{TEST_MQTT_PREFIX}/{manufacturer}/{serial}/{message_type}"


async def _collect_one(host: str, port: int, topic: str, timeout: float = 3.0) -> bytes | None:
    """Fresh MQTT client subscribing *after* the fact — the point of `retain`
    is that this still immediately receives the last-published message."""
    async with aiomqtt.Client(hostname=host, port=port) as client:
        await client.subscribe(topic)
        try:
            async with asyncio.timeout(timeout):
                async for message in client.messages:
                    return bytes(message.payload)
        except TimeoutError:
            return None
    return None


async def test_retained_state_visible_to_late_subscriber(mqtt_broker):
    host, port = mqtt_broker
    transport = MqttTransport(host, port, manufacturer=MANUFACTURER, serial_number=SERIAL, topic_prefix=TEST_MQTT_PREFIX)
    await transport.connect()
    try:
        state = StateMessage(
            headerId=1,
            timestamp="2026-01-01T00:00:00.000Z",
            manufacturer=MANUFACTURER,
            serialNumber=SERIAL,
            powerSupply=PowerSupply(stateOfCharge=77.0, charging=False),
        )
        await transport.publish(MANUFACTURER, SERIAL, "state", state, retain=True)
        await asyncio.sleep(0.2)  # let the broker register the retained message

        payload = await _collect_one(host, port, _topic(MANUFACTURER, SERIAL, "state"))
        assert payload is not None, "a late subscriber never received the retained state message"
        assert json.loads(payload)["powerSupply"]["stateOfCharge"] == 77.0
    finally:
        await transport.close()


async def test_visualization_is_not_retained(mqtt_broker):
    host, port = mqtt_broker
    transport = MqttTransport(host, port, manufacturer=MANUFACTURER, serial_number=SERIAL, topic_prefix=TEST_MQTT_PREFIX)
    await transport.connect()
    try:
        viz = VisualizationMessage(
            headerId=1,
            timestamp="2026-01-01T00:00:00.000Z",
            manufacturer=MANUFACTURER,
            serialNumber=SERIAL,
            referenceStateHeaderId=1,
        )
        await transport.publish(MANUFACTURER, SERIAL, "visualization", viz, retain=False)
        await asyncio.sleep(0.2)

        payload = await _collect_one(host, port, _topic(MANUFACTURER, SERIAL, "visualization"), timeout=1.0)
        assert payload is None, "visualization should not be retained, but a late subscriber received one anyway"
    finally:
        await transport.close()


def _run_transport_until_killed(host: str, port: int, manufacturer: str, serial: str, prefix: str) -> None:
    """Runs in a separate process so it can be SIGKILL'd — a real abrupt,
    ungraceful disconnect (no MQTT DISCONNECT packet sent), not a bypass of
    aiomqtt internals from inside the test process."""
    import asyncio as _asyncio

    from vda5050_sim.schemas import ConnectionMessage
    from vda5050_sim.schemas import ConnectionState as _ConnectionState
    from vda5050_sim.transport import MqttTransport as _MqttTransport

    async def _main() -> None:
        transport = _MqttTransport(host, port, manufacturer=manufacturer, serial_number=serial, topic_prefix=prefix)
        await transport.connect()
        online = ConnectionMessage(
            headerId=1,
            timestamp="2026-01-01T00:00:00.000Z",
            manufacturer=manufacturer,
            serialNumber=serial,
            connectionState=_ConnectionState.ONLINE,
        )
        await transport.publish(manufacturer, serial, "connection", online, retain=True)
        await _asyncio.sleep(3600)

    _asyncio.run(_main())


@pytest.mark.timeout(30)
async def test_lwt_fires_on_abrupt_disconnect(mqtt_broker):
    host, port = mqtt_broker
    manufacturer, serial = "TestCo", "lwt-bot-01"
    topic = _topic(manufacturer, serial, "connection")

    ctx = multiprocessing.get_context("spawn")
    proc = ctx.Process(target=_run_transport_until_killed, args=(host, port, manufacturer, serial, TEST_MQTT_PREFIX))
    proc.start()

    try:
        async with aiomqtt.Client(hostname=host, port=port) as client:
            await client.subscribe(topic)
            messages = client.messages

            async def _next_connection_state() -> str:
                async with asyncio.timeout(10):
                    async for message in messages:
                        return json.loads(message.payload)["connectionState"]
                raise AssertionError("no message received")

            assert await _next_connection_state() == ConnectionState.ONLINE.value

            assert proc.pid is not None
            os.kill(proc.pid, signal.SIGKILL)
            proc.join(timeout=5)

            assert await _next_connection_state() == ConnectionState.CONNECTION_BROKEN.value
    finally:
        if proc.is_alive():
            with contextlib.suppress(Exception):
                proc.terminate()
            proc.join(timeout=5)
