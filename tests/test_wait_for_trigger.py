"""waitForTrigger/trigger (spec Table 4) — waitForTrigger is NODE/ZONE-scoped
(not EDGE, despite the intuitive guess) and never auto-times-out; it's
released only by an explicit `trigger` instant action or FAILED if the order
is cancelled while waiting."""

from __future__ import annotations

import asyncio

from helpers import (
    TEST_PREFIX,
    make_action,
    make_action_param,
    make_instant_actions,
    make_node,
    make_order,
    publish_instant_actions,
    publish_order,
    state_listener,
)

from vda5050_sim.agv import RobotConfig
from vda5050_sim.schemas import ActionStatus

MODEL = "spot"


async def test_wait_for_trigger_blocks_until_released(fm, fleet_factory):
    cfg = RobotConfig(id="wft-bot-01", model=MODEL, supported_actions=["waitForTrigger"])
    await fleet_factory([cfg])

    wait = make_action_param("wft1", "waitForTrigger", {"triggerType": ["FLEET_CONTROL"]})
    nodes = [make_node("n0", 0, 0.0, 0.0, actions=[wait])]
    order = make_order(order_id="wft-1", order_update_id=0, model=MODEL, serial=cfg.id, nodes=nodes, edges=[])

    async with state_listener(fm, TEST_PREFIX, MODEL, cfg.id) as listener:
        await publish_order(fm, TEST_PREFIX, MODEL, cfg.id, order)
        await listener.wait_for(
            lambda s: any(a.actionId == "wft1" and a.actionStatus == ActionStatus.RUNNING for a in s.actionStates)
        )

        # Stays RUNNING well past a normal action_duration_s (0.1s in tests)
        # — never auto-times-out.
        await asyncio.sleep(0.3)
        recent = [a for s in listener.received[-3:] for a in s.actionStates if a.actionId == "wft1"]
        assert all(a.actionStatus == ActionStatus.RUNNING for a in recent)

        trigger = make_instant_actions(model=MODEL, serial=cfg.id, actions=[make_action("t1", "trigger")])
        await publish_instant_actions(fm, TEST_PREFIX, MODEL, cfg.id, trigger)

        await listener.wait_for(
            lambda s: any(a.actionId == "wft1" and a.actionStatus == ActionStatus.FINISHED for a in s.actionStates)
        )


async def test_trigger_with_no_waiter_is_rejected(running_fleet, fm):
    model, serial = "spot", "test-spot-01"
    async with state_listener(fm, TEST_PREFIX, model, serial) as listener:
        trigger = make_instant_actions(model=model, serial=serial, actions=[make_action("t2", "trigger")])
        await publish_instant_actions(fm, TEST_PREFIX, model, serial, trigger)

        matched = await listener.wait_for(
            lambda s: any(a.actionId == "t2" and a.actionStatus == ActionStatus.FAILED for a in s.instantActionStates)
        )
        assert matched is not None
