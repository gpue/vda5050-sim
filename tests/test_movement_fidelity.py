"""Phase 1 correctness/movement-fidelity fixes: edge-bound actions actually
run, pause freezes action-duration budgets (and reflects PAUSED status),
initPosition, proactive horizon-threshold newBaseRequest, and per-edge
maximumSpeed/orientation/reachOrientationBeforeEntering enforcement.
"""

from __future__ import annotations

import asyncio
import math

from helpers import (
    TEST_PREFIX,
    collect_states,
    make_action,
    make_action_param,
    make_edge,
    make_instant_actions,
    make_node,
    make_order,
    publish_instant_actions,
    publish_order,
    state_listener,
)

from vda5050_sim.schemas import ActionStatus, BlockingType, OrientationType

MODEL, SERIAL = "spot", "test-spot-01"


async def test_edge_action_executes(running_fleet, fm):
    # "pick" (rather than an arbitrary custom action) since spot-01's fleet
    # config declares it as a supported capability — an undeclared action
    # would now be correctly rejected by the capability-gating added for the
    # full VDA5050 predefined-action catalog.
    edge_action = make_action("edge-a0", "pick", blocking=BlockingType.NONE)
    nodes = [make_node("n0", 0, 0.0, 0.0), make_node("n1", 2, 0.3, 0.0)]
    edges = [make_edge("e0", 1, actions=[edge_action])]
    order = make_order(order_id="edge-action-1", order_update_id=0, model=MODEL, serial=SERIAL, nodes=nodes, edges=edges)
    await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, order)

    await asyncio.sleep(0.6)  # long enough to traverse (0.3m at 5 m/s) and finish the 0.1s action
    states = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 1)
    matching = [a for a in states[-1].actionStates if a.actionId == "edge-a0"]
    assert matching, "edge-bound action was never tracked in actionStates"
    assert matching[0].actionStatus == ActionStatus.FINISHED


async def test_hard_blocking_edge_action_holds_movement(running_fleet, fm):
    edge_action = make_action("edge-hard0", "pick", blocking=BlockingType.HARD)
    nodes = [make_node("n0", 0, 0.0, 0.0), make_node("n1", 2, 0.05, 0.0)]
    edges = [make_edge("e0", 1, actions=[edge_action])]
    order = make_order(order_id="edge-hard-1", order_update_id=0, model=MODEL, serial=SERIAL, nodes=nodes, edges=edges)
    await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, order)

    await asyncio.sleep(0.05)
    states = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 1)
    assert states[-1].mobileRobotPosition.x < 0.01  # held at n0 while the HARD edge action runs

    await asyncio.sleep(0.5)
    states = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 1)
    assert states[-1].lastNodeId == "n1"  # eventually completes the trip once the action finishes


async def test_pause_freezes_action_duration_budget(running_fleet, fm):
    """The bug this guards: elapsed_s previously kept accruing during pause,
    so a RUNNING action's remaining duration budget silently burned down while
    paused and could appear FINISHED the instant the robot resumed."""
    blocking_action = make_action("pause-a0", "pick", blocking=BlockingType.HARD)
    nodes = [make_node("n0", 0, 0.0, 0.0, actions=[blocking_action]), make_node("n1", 2, 0.1, 0.0)]
    order = make_order(
        order_id="pause-budget-1", order_update_id=0, model=MODEL, serial=SERIAL, nodes=nodes, edges=[make_edge("e0", 1)]
    )
    await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, order)
    await asyncio.sleep(0.03)  # let the action start RUNNING (well short of its 0.1s duration)

    pause = make_instant_actions(model=MODEL, serial=SERIAL, actions=[make_action("pp1", "startPause")])
    await publish_instant_actions(fm, TEST_PREFIX, MODEL, SERIAL, pause)
    await asyncio.sleep(0.03)

    states = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 1)
    matching = [a for a in states[-1].actionStates if a.actionId == "pause-a0"]
    assert matching[0].actionStatus == ActionStatus.PAUSED

    # Stay paused for far longer than the action's remaining duration budget
    # (0.1s) would have needed — if elapsed_s kept accruing during pause,
    # this alone would already finish it.
    await asyncio.sleep(0.5)
    states = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 1)
    matching = [a for a in states[-1].actionStates if a.actionId == "pause-a0"]
    assert matching[0].actionStatus == ActionStatus.PAUSED  # still paused, not finished

    resume = make_instant_actions(model=MODEL, serial=SERIAL, actions=[make_action("pp2", "stopPause")])
    await publish_instant_actions(fm, TEST_PREFIX, MODEL, SERIAL, resume)

    # Only a small remaining budget should be left (~0.07s) — not finished
    # instantly, but finished shortly after resuming.
    await asyncio.sleep(0.3)
    states = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 1)
    matching = [a for a in states[-1].actionStates if a.actionId == "pause-a0"]
    assert matching[0].actionStatus == ActionStatus.FINISHED


async def test_init_position(running_fleet, fm):
    async with state_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        init = make_instant_actions(
            model=MODEL,
            serial=SERIAL,
            actions=[make_action_param("ip1", "initializePosition", {"x": 12.5, "y": -3.0, "theta": 1.0, "mapId": "warehouse-2"})],
        )
        await publish_instant_actions(fm, TEST_PREFIX, MODEL, SERIAL, init)

        matched = await listener.wait_for(lambda s: s.mobileRobotPosition.mapId == "warehouse-2")
        assert matched.mobileRobotPosition.x == 12.5
        assert matched.mobileRobotPosition.y == -3.0
        assert matched.mobileRobotPosition.theta == 1.0


async def test_horizon_threshold_triggers_new_base_request(running_fleet, fm):
    # Default horizon_threshold_nodes=2. A 3-node route starts with 3 nodes
    # remaining (no request yet); after the first hop only 2 remain, which
    # meets the threshold and should proactively flip newBaseRequest.
    nodes = [make_node("n0", 0, 0.0, 0.0), make_node("n1", 2, 0.05, 0.0), make_node("n2", 4, 0.10, 0.0)]
    edges = [make_edge("e0", 1), make_edge("e1", 3)]
    order = make_order(order_id="horizon-1", order_update_id=0, model=MODEL, serial=SERIAL, nodes=nodes, edges=edges)
    await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, order)

    await asyncio.sleep(0.4)  # enough to complete both short hops
    states = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 1)
    assert states[-1].newBaseRequest is True


async def test_edge_max_speed_override_slows_travel(running_fleet, fm):
    # Test fleet's default_speed_mps is 5.0 — a much lower edge.maximumSpeed
    # should visibly cap actual travel speed.
    nodes = [make_node("n0", 0, 0.0, 0.0), make_node("n1", 2, 10.0, 0.0)]
    edges = [make_edge("e0", 1, maximum_speed=0.5)]
    order = make_order(order_id="maxspeed-1", order_update_id=0, model=MODEL, serial=SERIAL, nodes=nodes, edges=edges)
    await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, order)

    await asyncio.sleep(0.3)
    states = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 1)
    x = states[-1].mobileRobotPosition.x
    assert x < 0.3  # would be ~1.5m at the unclamped 5 m/s test speed
    assert x > 0.05  # but it did move, at roughly 0.5 m/s


async def test_edge_global_orientation_overrides_heading(running_fleet, fm):
    fixed_theta = math.pi / 2  # facing +y, even though travel is along +x
    nodes = [make_node("n0", 0, 0.0, 0.0), make_node("n1", 2, 5.0, 0.0)]
    edges = [make_edge("e0", 1, orientation=fixed_theta, orientation_type=OrientationType.GLOBAL)]
    order = make_order(order_id="orientation-1", order_update_id=0, model=MODEL, serial=SERIAL, nodes=nodes, edges=edges)
    await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, order)

    await asyncio.sleep(0.1)
    states = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 1)
    assert states[-1].driving is True
    assert abs(states[-1].mobileRobotPosition.theta - fixed_theta) < 1e-6


async def test_edge_reach_orientation_before_entering_rotates_before_moving(running_fleet, fm):
    # Facing +y (pi/2) required before entering, but the geometric heading to
    # the target is +x (0 rad) — a ~1.57 rad turn at the default 1 rad/s
    # angular speed takes ~1.57s, during which position must not change.
    required_theta = math.pi / 2
    nodes = [make_node("n0", 0, 0.0, 0.0), make_node("n1", 2, 5.0, 0.0)]
    edges = [
        make_edge(
            "e0", 1, orientation=required_theta, orientation_type=OrientationType.GLOBAL, reach_orientation_before_entering=True
        )
    ]
    order = make_order(order_id="rotation-1", order_update_id=0, model=MODEL, serial=SERIAL, nodes=nodes, edges=edges)
    await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, order)

    await asyncio.sleep(0.5)  # well into the ~1.57s rotation phase
    states = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 1)
    assert states[-1].mobileRobotPosition.x == 0.0  # hasn't moved yet — still rotating in place
    assert 0.0 < states[-1].mobileRobotPosition.theta < required_theta  # rotating toward the target

    await asyncio.sleep(1.5)  # rotation should be done, movement should have started
    states = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 1)
    assert states[-1].mobileRobotPosition.x > 0.0
