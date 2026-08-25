"""Zone runtime effects (spec Section 6.4.1) — the geometrically-meaningful
subset chosen in scope for this gap-closing pass: BLOCKED, SPEED_LIMIT,
ACTION, DIRECTED, BIDIRECTED. PRIORITY/PENALTY/LINE_GUIDED deliberately stay
accepted-only (no real path planner for them to influence) — see README."""

from __future__ import annotations

import asyncio

from helpers import (
    TEST_PREFIX,
    make_action,
    make_order,
    make_route,
    make_zone,
    make_zone_set,
    make_zone_set_message,
    poll_until,
    publish_order,
    publish_zone_set,
)

from vda5050_sim.agv import RobotConfig
from vda5050_sim.schemas import ZoneType

MODEL = "spot"


async def test_speed_limit_zone_caps_effective_speed(fm, fleet_factory):
    cfg = RobotConfig(id="speedlimit-bot-01", model=MODEL, supported_actions=[])
    fleet = await fleet_factory([cfg])
    agv = fleet.runtimes[cfg.id].agv

    zone = make_zone("sl1", ZoneType.SPEED_LIMIT, [(-1, -1), (10, -1), (10, 1), (-1, 1)], maximumSpeed=0.1)
    zone_set = make_zone_set(map_id="default", zone_set_id="sl-zs", zones=[zone])
    await publish_zone_set(fm, TEST_PREFIX, MODEL, cfg.id, make_zone_set_message(model=MODEL, serial=cfg.id, zone_set=zone_set))
    await asyncio.sleep(0.05)

    nodes, edges = make_route([(0.0, 0.0), (5.0, 0.0)])  # test default_speed_mps=5.0 would cover this in ~1s uncapped
    order = make_order(order_id="sl-order", order_update_id=0, model=MODEL, serial=cfg.id, nodes=nodes, edges=edges)
    await publish_order(fm, TEST_PREFIX, MODEL, cfg.id, order)

    await poll_until(lambda: agv.driving)
    await asyncio.sleep(0.3)
    # At the zone's 0.1 m/s cap, 0.3s covers ~0.03m; uncapped (5 m/s) would
    # already be most of the way there.
    assert agv.x < 0.2


async def test_blocked_zone_holds_movement_and_requests_new_base(fm, fleet_factory):
    cfg = RobotConfig(id="blocked-bot-01", model=MODEL, supported_actions=[])
    fleet = await fleet_factory([cfg])
    agv = fleet.runtimes[cfg.id].agv

    zone = make_zone("bz1", ZoneType.BLOCKED, [(0.5, -1), (2, -1), (2, 1), (0.5, 1)])
    zone_set = make_zone_set(map_id="default", zone_set_id="bz-zs", zones=[zone])
    await publish_zone_set(fm, TEST_PREFIX, MODEL, cfg.id, make_zone_set_message(model=MODEL, serial=cfg.id, zone_set=zone_set))
    await asyncio.sleep(0.05)

    nodes, edges = make_route([(0.0, 0.0), (1.0, 0.0)])  # target node lies inside the BLOCKED zone
    order = make_order(order_id="bz-order", order_update_id=0, model=MODEL, serial=cfg.id, nodes=nodes, edges=edges)
    await publish_order(fm, TEST_PREFIX, MODEL, cfg.id, order)

    await poll_until(lambda: agv.new_base_request)
    await asyncio.sleep(0.2)
    assert agv.driving is False
    assert agv.x < 0.1  # never left the start


async def test_action_zone_fires_entry_and_exit_actions(fm, fleet_factory):
    cfg = RobotConfig(id="actionzone-bot-01", model=MODEL, supported_actions=["beep"])
    fleet = await fleet_factory([cfg])
    agv = fleet.runtimes[cfg.id].agv

    entry_action = make_action("zone-entry-a0", "beep")
    exit_action = make_action("zone-exit-a0", "beep")
    zone = make_zone(
        "az1",
        ZoneType.ACTION,
        [(0.4, -1), (0.6, -1), (0.6, 1), (0.4, 1)],
        entryActions=[entry_action],
        exitActions=[exit_action],
    )
    zone_set = make_zone_set(map_id="default", zone_set_id="az-zs", zones=[zone])
    await publish_zone_set(fm, TEST_PREFIX, MODEL, cfg.id, make_zone_set_message(model=MODEL, serial=cfg.id, zone_set=zone_set))
    await asyncio.sleep(0.05)

    nodes, edges = make_route([(0.0, 0.0), (1.0, 0.0)])
    order = make_order(order_id="az-order", order_update_id=0, model=MODEL, serial=cfg.id, nodes=nodes, edges=edges)
    await publish_order(fm, TEST_PREFIX, MODEL, cfg.id, order)

    await poll_until(lambda: any(s.actionId == "zone-entry-a0" for s in agv.zone_action_states), timeout=5.0)
    await poll_until(lambda: any(s.actionId == "zone-exit-a0" for s in agv.zone_action_states), timeout=5.0)


async def test_directed_zone_holds_movement_against_restricted_direction(fm, fleet_factory):
    cfg = RobotConfig(id="directed-bot-01", model=MODEL, supported_actions=[])
    fleet = await fleet_factory([cfg])
    agv = fleet.runtimes[cfg.id].agv

    # Zone only allows travel in the +x direction (0 rad); the order below
    # travels in -x, directly against it, under a STRICT limitation.
    zone = make_zone(
        "dz1", ZoneType.DIRECTED, [(-2, -1), (2, -1), (2, 1), (-2, 1)], direction=0.0, limitation="STRICT"
    )
    zone_set = make_zone_set(map_id="default", zone_set_id="dz-zs", zones=[zone])
    await publish_zone_set(fm, TEST_PREFIX, MODEL, cfg.id, make_zone_set_message(model=MODEL, serial=cfg.id, zone_set=zone_set))
    await asyncio.sleep(0.05)

    nodes, edges = make_route([(0.0, 0.0), (-1.0, 0.0)])
    order = make_order(order_id="dz-order", order_update_id=0, model=MODEL, serial=cfg.id, nodes=nodes, edges=edges)
    await publish_order(fm, TEST_PREFIX, MODEL, cfg.id, order)

    await poll_until(lambda: agv.new_base_request)
    await asyncio.sleep(0.2)
    assert agv.driving is False
    assert agv.x > -0.1  # held at start, never moved against the restricted direction
