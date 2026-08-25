"""Table 4 scope enforcement — a catalog actionType used outside the
instant/node/edge/zone scope the spec defines for it is rejected (FAILED),
not silently accepted. Also covers the capability gate (core=False actions
require an explicit RobotConfig.supported_actions declaration)."""

from __future__ import annotations

from helpers import (
    TEST_PREFIX,
    make_action,
    make_instant_actions,
    make_node,
    make_order,
    publish_instant_actions,
    publish_order,
    state_listener,
)

from vda5050_sim.schemas import ActionStatus

MODEL, SERIAL = "spot", "test-spot-01"


async def test_pick_sent_as_instant_action_rejected(running_fleet, fm):
    # pick is NODE/EDGE-scoped only (Table 4) — never a valid instant action.
    async with state_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        pick = make_instant_actions(model=MODEL, serial=SERIAL, actions=[make_action("bad-pick", "pick")])
        await publish_instant_actions(fm, TEST_PREFIX, MODEL, SERIAL, pick)

        matched = await listener.wait_for(
            lambda s: any(a.actionId == "bad-pick" and a.actionStatus == ActionStatus.FAILED for a in s.instantActionStates)
        )
        assert any(e.errorType == "validationError" for e in matched.errors)


async def test_state_request_sent_as_node_action_rejected(running_fleet, fm):
    # stateRequest is INSTANT-only (Table 4) — never valid on a node.
    bad_action = make_action("bad-sr", "stateRequest")
    nodes = [make_node("n0", 0, 0.0, 0.0, actions=[bad_action])]
    order = make_order(order_id="bad-scope-1", order_update_id=0, model=MODEL, serial=SERIAL, nodes=nodes, edges=[])

    async with state_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, order)
        matched = await listener.wait_for(
            lambda s: any(a.actionId == "bad-sr" and a.actionStatus == ActionStatus.FAILED for a in s.actionStates)
        )
        assert matched is not None


async def test_undeclared_custom_node_action_rejected(running_fleet, fm):
    # "unknown-gesture" isn't in this robot's supported_actions — outside the
    # catalog entirely, so it needs an explicit capability declaration.
    custom = make_action("bad-custom", "unknown-gesture")
    nodes = [make_node("n0", 0, 0.0, 0.0, actions=[custom])]
    order = make_order(order_id="bad-scope-2", order_update_id=0, model=MODEL, serial=SERIAL, nodes=nodes, edges=[])

    async with state_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, order)
        matched = await listener.wait_for(
            lambda s: any(a.actionId == "bad-custom" and a.actionStatus == ActionStatus.FAILED for a in s.actionStates)
        )
        assert matched is not None
