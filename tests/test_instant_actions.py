"""cancelOrder / startPause / stopPause conformance."""

from __future__ import annotations

import asyncio

from helpers import (
    TEST_PREFIX,
    collect_states,
    make_action,
    make_instant_actions,
    make_node,
    make_order,
    make_route,
    publish_instant_actions,
    publish_order,
    state_listener,
)

from vda5050_sim.schemas import ActionStatus

MODEL, SERIAL = "spot", "test-spot-01"


async def test_cancel_order_clears_state_and_allows_new_order(running_fleet, fm):
    nodes, edges = make_route([(0.0, 0.0), (50.0, 0.0)])
    order = make_order(order_id="cancel-1", order_update_id=0, model=MODEL, serial=SERIAL, nodes=nodes, edges=edges)
    await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, order)
    await asyncio.sleep(0.2)  # confirm it's actually running, not idle

    async with state_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        cancel = make_instant_actions(model=MODEL, serial=SERIAL, actions=[make_action("c1", "cancelOrder")])
        await publish_instant_actions(fm, TEST_PREFIX, MODEL, SERIAL, cancel)

        matched = await listener.wait_for(
            lambda s: any(a.actionId == "c1" and a.actionStatus == ActionStatus.RUNNING for a in s.instantActionStates)
        )
        assert matched is not None

        finished = await listener.wait_for(
            lambda s: any(a.actionId == "c1" and a.actionStatus == ActionStatus.FINISHED for a in s.instantActionStates)
        )
        assert finished.orderId == ""
        assert finished.driving is False

    # A brand-new order must now be accepted immediately (idle gate satisfied).
    follow_up = make_order(
        order_id="after-cancel-1",
        order_update_id=0,
        model=MODEL,
        serial=SERIAL,
        nodes=[make_node("m0", 0, 0.0, 0.0)],
        edges=[],
    )
    await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, follow_up)
    states = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 3)
    assert states[-1].orderId == "after-cancel-1"
    assert not any(e.errorType == "otherOrderActive" for s in states for e in s.errors)


async def test_cancel_order_with_no_active_order_fails(running_fleet, fm):
    async with state_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        cancel = make_instant_actions(model=MODEL, serial=SERIAL, actions=[make_action("c2", "cancelOrder")])
        await publish_instant_actions(fm, TEST_PREFIX, MODEL, SERIAL, cancel)

        matched = await listener.wait_for(
            lambda s: any(a.actionId == "c2" and a.actionStatus == ActionStatus.FAILED for a in s.instantActionStates)
        )
        assert any(e.errorType == "noOrderToCancel" for e in matched.errors)


async def test_pause_halts_movement_and_resume_continues(running_fleet, fm):
    # Long trip: must still be mid-transit well past a 0.6s test window
    # (test speed is 5 m/s) so pausing has something to actually halt.
    nodes, edges = make_route([(0.0, 0.0), (100.0, 0.0)])
    order = make_order(order_id="pause-1", order_update_id=0, model=MODEL, serial=SERIAL, nodes=nodes, edges=edges)
    await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, order)
    await asyncio.sleep(0.05)

    pause = make_instant_actions(model=MODEL, serial=SERIAL, actions=[make_action("p1", "startPause")])
    await publish_instant_actions(fm, TEST_PREFIX, MODEL, SERIAL, pause)
    await asyncio.sleep(0.05)

    states_paused_1 = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 1)
    await asyncio.sleep(0.2)
    states_paused_2 = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 1)
    assert states_paused_1[-1].paused is True
    assert states_paused_1[-1].mobileRobotPosition.x == states_paused_2[-1].mobileRobotPosition.x

    resume = make_instant_actions(model=MODEL, serial=SERIAL, actions=[make_action("p2", "stopPause")])
    await publish_instant_actions(fm, TEST_PREFIX, MODEL, SERIAL, resume)
    await asyncio.sleep(0.3)

    states_resumed = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 1)
    assert states_resumed[-1].paused is False
    assert states_resumed[-1].mobileRobotPosition.x > states_paused_2[-1].mobileRobotPosition.x
