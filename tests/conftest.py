from __future__ import annotations

import asyncio
import socket
import subprocess
import time

import pytest
import pytest_asyncio
from helpers import TEST_PREFIX
from nats.aio.client import Client as NATS

from vda5050_sim.agv import RobotConfig
from vda5050_sim.config import Settings
from vda5050_sim.fleet import Fleet
from vda5050_sim.logbuffer import LogBuffer
from vda5050_sim.transport import NatsTransport


def _port_open(host: str, port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.2)
        return s.connect_ex((host, port)) == 0


@pytest.fixture(scope="session")
def nats_url():
    """Reuse an already-running NATS (e.g. CI's `nats:2` service container),
    otherwise spawn a local nats-server for the test session."""
    port = 4222
    if _port_open("127.0.0.1", port):
        yield f"nats://127.0.0.1:{port}"
        return

    proc = subprocess.Popen(
        ["nats-server", "-p", str(port)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    for _ in range(50):
        if _port_open("127.0.0.1", port):
            break
        time.sleep(0.1)
    else:
        proc.terminate()
        raise RuntimeError("nats-server did not start in time")

    yield f"nats://127.0.0.1:{port}"

    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()


def test_settings(**overrides) -> Settings:
    base = dict(
        nats_broker="nats://127.0.0.1:4222",
        vda5050_prefix=TEST_PREFIX,
        tick_s=0.02,
        action_duration_s=0.1,
        state_hz=20.0,
        visualization_hz=20.0,
        connection_heartbeat_s=1000.0,
        default_speed_mps=5.0,
    )
    base.update(overrides)
    return Settings(**base)


@pytest_asyncio.fixture
async def fm(nats_url):
    """A NATS connection playing the role of an external fleet manager."""
    client = NATS()
    await client.connect(servers=[nats_url])
    yield client
    await client.drain()


@pytest_asyncio.fixture
async def fleet_factory(nats_url):
    """Factory: build+start a Fleet with the given RobotConfigs; cleans up after the test."""
    created: list[Fleet] = []

    async def _make(configs: list[RobotConfig], settings: Settings | None = None) -> Fleet:
        settings = settings or test_settings()
        transport = NatsTransport(nats_url, settings.vda5050_prefix)
        await transport.connect()
        fleet = Fleet(settings, transport, LogBuffer())
        await fleet.start(configs)
        created.append(fleet)
        return fleet

    yield _make

    for fleet in created:
        await fleet.stop()
        await fleet.transport.close()


@pytest_asyncio.fixture
async def running_fleet(fleet_factory):
    """Two default robots (spot, go2), already started and settled."""
    fleet = await fleet_factory(
        [
            RobotConfig(id="test-spot-01", model="spot", supported_actions=["pick", "drop"]),
            RobotConfig(id="test-go2-01", model="go2", supported_actions=[]),
        ]
    )
    await asyncio.sleep(0.3)
    return fleet
