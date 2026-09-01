"""Debug-only routes (debug_routes.py) — deterministic triggers a nova-nav
Analytics-tab test session (or any other consumer's test suite) can use to
force battery/connection/fault state instead of waiting out real
probability. Verified the same way the rest of this suite verifies
everything: over the real NATS wire (fm + state/connection listeners), not
by poking SimulatedAgv internals directly. Listeners are opened *before*
triggering the debug call — test_settings' connection_heartbeat_s=1000.0
means a missed connection message has nothing else to catch it on.
"""

from __future__ import annotations

import pytest
from fastapi import FastAPI
from httpx import ASGITransport, AsyncClient

from helpers import TEST_PREFIX, connection_listener, state_listener

from vda5050_sim.debug_routes import router as debug_router
from vda5050_sim.schemas import ConnectionState

MODEL, SERIAL, MANUFACTURER = "spot", "test-spot-01", "BostonDynamics"


@pytest.fixture
async def debug_client(running_fleet):
    app = FastAPI()
    app.state.fleet = running_fleet
    app.include_router(debug_router)
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        yield client


async def test_set_battery_reflected_in_next_state(fm, running_fleet, debug_client):
    async with state_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        resp = await debug_client.post(f"/debug/{MANUFACTURER}/{SERIAL}/battery", json={"percent": 12.5})
        assert resp.status_code == 200
        assert resp.json()["batterySoc"] == 12.5

        matched = await listener.wait_for(lambda s: s.powerSupply.stateOfCharge == 12.5)
        assert matched is not None


async def test_battery_percent_is_clamped(debug_client):
    resp = await debug_client.post(f"/debug/{MANUFACTURER}/{SERIAL}/battery", json={"percent": 150})
    assert resp.status_code == 200
    assert resp.json()["batterySoc"] == 100.0


async def test_force_offline_reflected_in_connection_message(fm, running_fleet, debug_client):
    async with connection_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        resp = await debug_client.post(f"/debug/{MANUFACTURER}/{SERIAL}/connection", json={"state": "OFFLINE"})
        assert resp.status_code == 200

        matched = await listener.wait_for(lambda c: c.connectionState == ConnectionState.OFFLINE)
        assert matched is not None
    assert running_fleet.find_runtime(MANUFACTURER, SERIAL).online is False


async def test_hardware_fault_appears_once_then_clears(fm, running_fleet, debug_client):
    async with state_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        resp = await debug_client.post(f"/debug/{MANUFACTURER}/{SERIAL}/fault", json={"type": "hardware_fault"})
        assert resp.status_code == 200

        matched = await listener.wait_for(lambda s: any(e.errorType == "hardwareFault" for e in s.errors))
        assert matched is not None

        # The one-shot error is cleared out of pending_errors as soon as it's
        # published (see SimulatedAgv.build_state_message) — the very next
        # state must come back clean.
        cleared = await listener.wait_for(lambda s: s.errors == [])
        assert cleared is not None


async def test_invalid_fault_type_400s(debug_client):
    resp = await debug_client.post(f"/debug/{MANUFACTURER}/{SERIAL}/fault", json={"type": "not-a-real-fault"})
    assert resp.status_code == 400


async def test_unknown_robot_404s(debug_client):
    resp = await debug_client.post(f"/debug/{MANUFACTURER}/does-not-exist/battery", json={"percent": 50})
    assert resp.status_code == 404


async def test_manufacturer_mismatch_404s(debug_client):
    resp = await debug_client.post(f"/debug/WrongManufacturer/{SERIAL}/battery", json={"percent": 50})
    assert resp.status_code == 404
