"""pick/drop/detectObject/finePositioning as real NODE/EDGE-scoped actions
(spec Table 4) — pick/drop populate/depopulate state.loads for real, instead
of the old generic-instant-action passthrough."""

from __future__ import annotations

from helpers import (
    TEST_PREFIX,
    make_action,
    make_action_param,
    make_node,
    make_order,
    publish_order,
    state_listener,
)

from vda5050_sim.schemas import ActionStatus

MODEL, SERIAL = "spot", "test-spot-01"


async def test_pick_then_drop_updates_loads(running_fleet, fm):
    pick = make_action_param("pick1", "pick", {"loadId": "pallet-42", "loadType": "EPAL"})
    nodes = [make_node("n0", 0, 0.0, 0.0, actions=[pick])]
    order = make_order(order_id="pick-1", order_update_id=0, model=MODEL, serial=SERIAL, nodes=nodes, edges=[])

    async with state_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, order)
        matched = await listener.wait_for(
            lambda s: any(a.actionId == "pick1" and a.actionStatus == ActionStatus.FINISHED for a in s.actionStates)
        )
        assert any(load.loadId == "pallet-42" for load in matched.loads)

    drop = make_action("drop1", "drop")
    nodes2 = [make_node("n0", 0, 0.0, 0.0, actions=[drop])]
    order2 = make_order(order_id="drop-1", order_update_id=0, model=MODEL, serial=SERIAL, nodes=nodes2, edges=[])
    async with state_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, order2)
        matched = await listener.wait_for(
            lambda s: any(a.actionId == "drop1" and a.actionStatus == ActionStatus.FINISHED for a in s.actionStates)
        )
        assert not any(load.loadId == "pallet-42" for load in matched.loads)


async def test_pick_rejected_for_robot_without_capability(running_fleet, fm):
    # test-go2-01 in the running_fleet fixture has no supported_actions.
    go2_serial = "test-go2-01"
    pick = make_action("pick2", "pick")
    nodes = [make_node("n0", 0, 0.0, 0.0, actions=[pick])]
    order = make_order(order_id="pick-2", order_update_id=0, model="go2", serial=go2_serial, nodes=nodes, edges=[])

    async with state_listener(fm, TEST_PREFIX, "go2", go2_serial) as listener:
        await publish_order(fm, TEST_PREFIX, "go2", go2_serial, order)
        matched = await listener.wait_for(
            lambda s: any(a.actionId == "pick2" and a.actionStatus == ActionStatus.FAILED for a in s.actionStates)
        )
        assert matched is not None
