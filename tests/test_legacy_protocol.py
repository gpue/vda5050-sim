"""Robots configured with an older `protocol_version` (see fleet.default.yaml's
doc comment / legacy_shapes.py) must announce and reflect that on the wire:
pre-3.0 field names in state/connection, and a spec-correct rejection of
3.0.0-only instant actions — not just a different `version` string on an
otherwise-identical v3.0.0-shaped payload.
"""

from __future__ import annotations

import json

from helpers import (
    TEST_PREFIX,
    Listener,
    collect,
    collect_states,
    make_action_param,
    make_instant_actions,
    publish_instant_actions,
    state_listener,
)

from vda5050_sim.agv import RobotConfig
from vda5050_sim.robot_specs import get_manufacturer
from vda5050_sim.schemas import ActionStatus

MODEL = "spot"


def _legacy_prefix(version: str) -> str:
    major = version.split(".", 1)[0]
    return TEST_PREFIX.replace("v3", f"v{major}", 1)


async def test_legacy_state_uses_pre_3_0_field_names(fleet_factory, fm):
    serial = "test-legacy-state-01"
    await fleet_factory([RobotConfig(id=serial, model=MODEL, protocol_version="2.1.0")])

    subject = f"{_legacy_prefix('2.1.0')}.{get_manufacturer(MODEL)}.{serial}.state"
    [d] = await collect(fm, subject, json.loads, 1)

    assert d["version"] == "2.1.0"
    assert "agvPosition" in d
    assert "mobileRobotPosition" not in d
    assert "batteryState" in d
    assert "powerSupply" not in d
    assert "batteryCurrent" not in d["batteryState"]
    assert "eStop" in d["safetyState"]
    assert "activeEmergencyStop" not in d["safetyState"]


async def test_legacy_connection_uses_pre_3_0_field_names(fleet_factory, fm):
    # A robot's startup lifecycle publishes CONNECTION_BROKEN then ONLINE
    # (see test_lifecycle.py::test_connection_lifecycle_on_startup) — using
    # the raw dict (not the typed ConnectionMessage parser) here since
    # "CONNECTIONBROKEN" isn't a valid ConnectionState enum member and would
    # fail typed validation, which is exactly the shape difference under test.
    serial = "test-legacy-conn-01"
    subject = f"{_legacy_prefix('1.1.0')}.{get_manufacturer(MODEL)}.{serial}.connection"
    async with Listener(fm, subject, json.loads) as listener:
        await fleet_factory([RobotConfig(id=serial, model=MODEL, protocol_version="1.1.0")])
        broken = await listener.wait_for(lambda d: d["connectionState"] == "CONNECTIONBROKEN")
        online = await listener.wait_for(lambda d: d["connectionState"] == "ONLINE")

    assert broken["version"] == "1.1.0"
    assert online["version"] == "1.1.0"


async def test_v3_robot_unaffected_by_legacy_prefix_logic(fleet_factory, fm):
    """Guards against the version-prefix override leaking into the default
    (3.0.0) case — those robots must keep publishing on the fleet's
    configured prefix (here TEST_PREFIX) exactly as before this feature."""
    serial = "test-v3-unaffected-01"
    await fleet_factory([RobotConfig(id=serial, model=MODEL)])

    states = await collect_states(fm, TEST_PREFIX, MODEL, serial, 1)
    assert states[0].version == "3.0.0"


async def test_legacy_robot_rejects_zone_instant_action(fleet_factory, fm):
    # errors are drained from pending_errors into exactly one published
    # state (agv.py build_state_message), so this must catch that one
    # transient publish via a listener rather than collect_states's
    # "next N messages" (which can race past it) — same idiom as
    # test_instant_actions.py::test_cancel_order_with_no_active_order_fails.
    serial = "test-legacy-zone-01"
    await fleet_factory([RobotConfig(id=serial, model=MODEL, protocol_version="1.1.0")])

    async with state_listener(fm, _legacy_prefix("1.1.0"), MODEL, serial) as listener:
        msg = make_instant_actions(
            model=MODEL,
            serial=serial,
            actions=[
                make_action_param(
                    "za1", "downloadZoneSet", {"zoneSetId": "zs1", "zoneSetDownloadLink": "http://example/zs1"}
                )
            ],
        )
        await publish_instant_actions(fm, _legacy_prefix("1.1.0"), MODEL, serial, msg)

        matched = await listener.wait_for(
            lambda s: any(a.actionId == "za1" and a.actionStatus == ActionStatus.FAILED for a in s.instantActionStates)
        )
        assert any(e.errorType == "invalidInstantAction" for e in matched.errors)


async def test_pre_2_1_robot_rejects_map_instant_action(fleet_factory, fm):
    """Map distribution (downloadMap/enableMap/deleteMap, state.maps) was
    added in 2.1.0 (that release's own "Added map distribution" note) — not
    present in 1.1.0 or 2.0.0, despite 2.0.0 sharing major version "2" with
    2.1.0. Confirmed against the VDA5050 project's own vendored
    state.schema: no `maps` property at the 2.0.0 tag, present at 2.1.0."""
    serial = "test-legacy-map-01"
    await fleet_factory([RobotConfig(id=serial, model=MODEL, protocol_version="1.1.0")])

    async with state_listener(fm, _legacy_prefix("1.1.0"), MODEL, serial) as listener:
        msg = make_instant_actions(
            model=MODEL,
            serial=serial,
            actions=[make_action_param("dm1", "downloadMap", {"mapId": "warehouse-1", "mapVersion": "v1"})],
        )
        await publish_instant_actions(fm, _legacy_prefix("1.1.0"), MODEL, serial, msg)

        matched = await listener.wait_for(
            lambda s: any(a.actionId == "dm1" and a.actionStatus == ActionStatus.FAILED for a in s.instantActionStates)
        )
        assert any(e.errorType == "invalidInstantAction" for e in matched.errors)


async def test_2_0_0_robot_rejects_map_instant_action(fleet_factory, fm):
    """The exact boundary this fix targets: 2.0.0 and 2.1.0 share a major
    version but only 2.1.0 supports map distribution — a major-digit-only
    check (the bug this test guards) can't tell them apart."""
    serial = "test-legacy-map-2-0-0"
    await fleet_factory([RobotConfig(id=serial, model=MODEL, protocol_version="2.0.0")])

    prefix = TEST_PREFIX.replace("v3", "v2", 1)
    async with state_listener(fm, prefix, MODEL, serial) as listener:
        msg = make_instant_actions(
            model=MODEL,
            serial=serial,
            actions=[make_action_param("dm2", "downloadMap", {"mapId": "warehouse-1", "mapVersion": "v1"})],
        )
        await publish_instant_actions(fm, prefix, MODEL, serial, msg)

        matched = await listener.wait_for(
            lambda s: any(a.actionId == "dm2" and a.actionStatus == ActionStatus.FAILED for a in s.instantActionStates)
        )
        assert any(e.errorType == "invalidInstantAction" for e in matched.errors)


async def test_2_1_0_robot_accepts_map_instant_action(fleet_factory, fm):
    serial = "test-legacy-map-2-1-0"
    await fleet_factory([RobotConfig(id=serial, model=MODEL, protocol_version="2.1.0")])

    prefix = _legacy_prefix("2.1.0")
    async with state_listener(fm, prefix, MODEL, serial) as listener:
        msg = make_instant_actions(
            model=MODEL,
            serial=serial,
            actions=[make_action_param("dm3", "downloadMap", {"mapId": "warehouse-1", "mapVersion": "v1"})],
        )
        await publish_instant_actions(fm, prefix, MODEL, serial, msg)

        matched = await listener.wait_for(
            lambda s: any(m.mapId == "warehouse-1" and m.mapStatus == "DISABLED" for m in s.maps)
        )
        assert matched.version == "2.1.0"
