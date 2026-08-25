"""Map/zone-set lifecycle — downloadMap/enableMap/deleteMap and
downloadZoneSet/enableZoneSet/deleteZoneSet (spec Section 6.3/6.4, Table 4)."""

from __future__ import annotations

import asyncio

from helpers import (
    TEST_PREFIX,
    make_action_param,
    make_instant_actions,
    make_node,
    make_order,
    make_zone,
    make_zone_set,
    make_zone_set_message,
    publish_instant_actions,
    publish_order,
    publish_zone_set,
    state_listener,
)

from vda5050_sim.schemas import ActionStatus, ZoneType

MODEL, SERIAL = "spot", "test-spot-01"


async def _send(fm, action) -> None:
    msg = make_instant_actions(model=MODEL, serial=SERIAL, actions=[action])
    await publish_instant_actions(fm, TEST_PREFIX, MODEL, SERIAL, msg)


async def test_download_then_enable_map_appears_in_state(running_fleet, fm):
    async with state_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        params = {"mapId": "warehouse-3", "mapVersion": "v1", "mapDownloadLink": "https://example.test/m.map"}
        await _send(fm, make_action_param("dm1", "downloadMap", params))

        await listener.wait_for(lambda s: any(m.mapId == "warehouse-3" and m.mapStatus == "DISABLED" for m in s.maps))

        await _send(fm, make_action_param("em1", "enableMap", {"mapId": "warehouse-3", "mapVersion": "v1"}))

        matched = await listener.wait_for(lambda s: any(m.mapId == "warehouse-3" and m.mapStatus == "ENABLED" for m in s.maps))
        assert matched is not None


async def test_download_map_duplicate_rejected(running_fleet, fm):
    async with state_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        params = {"mapId": "dup-map", "mapVersion": "v1", "mapDownloadLink": "https://example.test/m.map"}
        await _send(fm, make_action_param("dm2", "downloadMap", params))
        await listener.wait_for(lambda s: any(m.mapId == "dup-map" for m in s.maps))

        await _send(fm, make_action_param("dm3", "downloadMap", params))
        matched = await listener.wait_for(
            lambda s: any(a.actionId == "dm3" and a.actionStatus == ActionStatus.FAILED for a in s.instantActionStates)
        )
        assert any(e.errorType == "duplicateMap" for e in matched.errors)


async def test_delete_map_removes_from_state(running_fleet, fm):
    async with state_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        params = {"mapId": "temp-map", "mapVersion": "v1", "mapDownloadLink": "https://example.test/m.map"}
        await _send(fm, make_action_param("dm4", "downloadMap", params))
        await listener.wait_for(lambda s: any(m.mapId == "temp-map" for m in s.maps))

        await _send(fm, make_action_param("dm5", "deleteMap", {"mapId": "temp-map", "mapVersion": "v1"}))
        await listener.wait_for(lambda s: not any(m.mapId == "temp-map" for m in s.maps))


async def test_order_with_unknown_map_id_reports_warning(running_fleet, fm):
    async with state_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        node = make_node("n0", 0, 0.0, 0.0)
        node.nodePosition.mapId = "never-downloaded"
        order = make_order(order_id="unknown-map-1", order_update_id=0, model=MODEL, serial=SERIAL, nodes=[node], edges=[])
        await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, order)

        matched = await listener.wait_for(lambda s: any(e.errorType == "unknownMapId" for e in s.errors))
        assert matched is not None


async def test_zone_set_topic_push_duplicate_rejected(running_fleet, fm):
    zone_set = make_zone_set(
        map_id="default", zone_set_id="dup-zone-set", zones=[make_zone("z1", ZoneType.BLOCKED, [(0, 0), (1, 0), (1, 1)])]
    )
    msg = make_zone_set_message(model=MODEL, serial=SERIAL, zone_set=zone_set)

    async with state_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        await publish_zone_set(fm, TEST_PREFIX, MODEL, SERIAL, msg)
        await asyncio.sleep(0.1)
        await publish_zone_set(fm, TEST_PREFIX, MODEL, SERIAL, msg)  # same zoneSetId again

        matched = await listener.wait_for(lambda s: any(e.errorType == "duplicateZoneSet" for e in s.errors))
        assert matched is not None
