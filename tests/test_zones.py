"""Phase 3: zone/traffic-control subsystem — `zoneSet` definitions, and the
two spec-explicit access-control triggers this simulator implements:
`Edge.corridor.releaseRequired` (-> edgeRequest) and RELEASE/
COORDINATED_REPLANNING zone membership (-> zoneRequest), both gated on a
matching `responses` grant.
"""

from __future__ import annotations

import asyncio

from helpers import (
    TEST_PREFIX,
    collect_states,
    make_corridor,
    make_edge,
    make_node,
    make_order,
    make_responses,
    make_zone,
    make_zone_set,
    make_zone_set_message,
    publish_order,
    publish_responses,
    publish_zone_set,
    state_listener,
)

from vda5050_sim.schemas import GrantType, RequestStatus, ZoneType
from vda5050_sim.validate import validate_message

MODEL, SERIAL = "spot", "test-spot-01"


async def test_zone_set_populates_state_zone_sets(running_fleet, fm):
    zone = make_zone("z1", ZoneType.BLOCKED, [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])
    zone_set = make_zone_set(map_id="default", zone_set_id="zs-1", zones=[zone])
    msg = make_zone_set_message(model=MODEL, serial=SERIAL, zone_set=zone_set)

    async with state_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        await publish_zone_set(fm, TEST_PREFIX, MODEL, SERIAL, msg)
        matched = await listener.wait_for(lambda s: any(zs.zoneSetId == "zs-1" for zs in s.zoneSets))
        entry = next(zs for zs in matched.zoneSets if zs.zoneSetId == "zs-1")
        assert entry.mapId == "default"


async def test_corridor_release_required_blocks_until_granted(running_fleet, fm):
    corridor = make_corridor(release_required=True)
    nodes = [make_node("n0", 0, 0.0, 0.0), make_node("n1", 2, 0.05, 0.0)]
    edges = [make_edge("e0", 1, corridor=corridor)]
    order = make_order(order_id="corridor-1", order_update_id=0, model=MODEL, serial=SERIAL, nodes=nodes, edges=edges)

    async with state_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, order)

        requested = await listener.wait_for(
            lambda s: any(r.edgeId == "e0" and r.requestStatus == RequestStatus.REQUESTED for r in s.edgeRequests)
        )
        assert requested.driving is False
        assert requested.mobileRobotPosition.x == 0.0
        request_id = next(r.requestId for r in requested.edgeRequests if r.edgeId == "e0")

        # Stays held — not auto-granted just because time passes.
        await asyncio.sleep(0.3)
        states = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 1)
        assert states[-1].mobileRobotPosition.x == 0.0

        grant = make_responses(model=MODEL, serial=SERIAL, responses=[(request_id, GrantType.GRANTED)])
        await publish_responses(fm, TEST_PREFIX, MODEL, SERIAL, grant)

        await listener.wait_for(
            lambda s: any(r.requestId == request_id and r.requestStatus == RequestStatus.GRANTED for r in s.edgeRequests)
        )

    await asyncio.sleep(0.3)
    states = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 1)
    assert states[-1].lastNodeId == "n1"


async def test_corridor_release_revoked_never_traverses(running_fleet, fm):
    corridor = make_corridor(release_required=True)
    nodes = [make_node("n0", 0, 0.0, 0.0), make_node("n1", 2, 0.05, 0.0)]
    edges = [make_edge("e0", 1, corridor=corridor)]
    order = make_order(order_id="corridor-2", order_update_id=0, model=MODEL, serial=SERIAL, nodes=nodes, edges=edges)

    async with state_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, order)
        requested = await listener.wait_for(
            lambda s: any(r.edgeId == "e0" and r.requestStatus == RequestStatus.REQUESTED for r in s.edgeRequests)
        )
        request_id = next(r.requestId for r in requested.edgeRequests if r.edgeId == "e0")

        reject = make_responses(model=MODEL, serial=SERIAL, responses=[(request_id, GrantType.REJECTED)])
        await publish_responses(fm, TEST_PREFIX, MODEL, SERIAL, reject)

        await listener.wait_for(
            lambda s: any(r.requestId == request_id and r.requestStatus == RequestStatus.REVOKED for r in s.edgeRequests)
        )

    await asyncio.sleep(0.3)
    states = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 1)
    assert states[-1].mobileRobotPosition.x == 0.0  # never moved
    assert states[-1].lastNodeId == ""


async def test_release_zone_blocks_until_granted(running_fleet, fm):
    # Destination node (5.0, 5.0) sits inside this zone's polygon.
    zone = make_zone("release-zone-1", ZoneType.RELEASE, [(4.0, 4.0), (6.0, 4.0), (6.0, 6.0), (4.0, 6.0)])
    zone_set = make_zone_set(map_id="default", zone_set_id="zs-release", zones=[zone])
    zone_msg = make_zone_set_message(model=MODEL, serial=SERIAL, zone_set=zone_set)
    await publish_zone_set(fm, TEST_PREFIX, MODEL, SERIAL, zone_msg)
    await asyncio.sleep(0.1)  # let the robot register the zone set before the order arrives

    nodes = [make_node("n0", 0, 0.0, 0.0), make_node("n1", 2, 5.0, 5.0)]
    edges = [make_edge("e0", 1)]
    order = make_order(order_id="release-1", order_update_id=0, model=MODEL, serial=SERIAL, nodes=nodes, edges=edges)

    async with state_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, order)

        requested = await listener.wait_for(
            lambda s: any(r.zoneId == "release-zone-1" and r.requestStatus == RequestStatus.REQUESTED for r in s.zoneRequests)
        )
        assert requested.driving is False
        request_id = next(r.requestId for r in requested.zoneRequests if r.zoneId == "release-zone-1")

        grant = make_responses(model=MODEL, serial=SERIAL, responses=[(request_id, GrantType.GRANTED)])
        await publish_responses(fm, TEST_PREFIX, MODEL, SERIAL, grant)

        await listener.wait_for(
            lambda s: any(r.requestId == request_id and r.requestStatus == RequestStatus.GRANTED for r in s.zoneRequests)
        )

    await asyncio.sleep(1.5)  # ~7m at 5 m/s test speed
    states = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 1)
    assert states[-1].lastNodeId == "n1"


async def test_zone_set_and_responses_conform_to_json_schema():
    zone = make_zone(
        "z1", ZoneType.SPEED_LIMIT, [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)], maximumSpeed=0.5
    )
    zone_set = make_zone_set(map_id="default", zone_set_id="zs-1", zones=[zone])
    zone_msg = make_zone_set_message(model=MODEL, serial=SERIAL, zone_set=zone_set)
    errors = validate_message(zone_msg.model_dump(mode="json", exclude_none=True), "zoneSet")
    assert errors == [], errors

    resp_msg = make_responses(model=MODEL, serial=SERIAL, responses=[("req-1", GrantType.GRANTED)])
    errors = validate_message(resp_msg.model_dump(mode="json", exclude_none=True), "responses")
    assert errors == [], errors
