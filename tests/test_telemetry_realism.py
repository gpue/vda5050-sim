"""Phase 4: telemetry realism — battery drain/charging, injectable safety-field
violations, operating-mode faults, and retry for retriable actions. Cosmetic/
resilience-testing value, not protocol correctness — verified against direct
RobotConfig/FaultProfile overrides for determinism (fault probabilities are
flipped to 1.0 to trigger immediately, then back to 0.0 mid-test so a
one-shot fault can actually clear and stay clear rather than re-triggering
every tick).
"""

from __future__ import annotations

import asyncio

from helpers import (
    TEST_PREFIX,
    collect_states,
    make_action,
    make_action_param,
    make_instant_actions,
    make_node,
    make_order,
    make_route,
    publish_instant_actions,
    publish_order,
    state_listener,
)

from vda5050_sim.agv import FaultProfile, RobotConfig
from vda5050_sim.schemas import ActionStatus, BlockingType, OperatingMode

MODEL, SERIAL = "spot", "fv-bot-01"


async def test_battery_drains_while_driving(fm, fleet_factory):
    cfg = RobotConfig(id="battery-bot-01", model=MODEL, supported_actions=[], battery_drain_percent_per_meter=1.0)
    await fleet_factory([cfg])

    nodes, edges = make_route([(0.0, 0.0), (2.0, 0.0)])
    order = make_order(order_id="drain-1", order_update_id=0, model=MODEL, serial=cfg.id, nodes=nodes, edges=edges)
    await publish_order(fm, TEST_PREFIX, MODEL, cfg.id, order)

    await asyncio.sleep(0.3)  # 2m at 5 m/s test speed ~= 0.4s; sample mid-trip
    states = await collect_states(fm, TEST_PREFIX, MODEL, cfg.id, 1)
    assert states[-1].powerSupply.stateOfCharge < 100.0

    await asyncio.sleep(0.3)
    states = await collect_states(fm, TEST_PREFIX, MODEL, cfg.id, 1)
    assert states[-1].mobileRobotPosition.x == 2.0  # trip finished
    assert states[-1].powerSupply.stateOfCharge <= 98.5  # ~1%/m * 2m, minus float slop


async def test_charging_increases_battery_until_stopped(fm, fleet_factory):
    cfg = RobotConfig(id="charge-bot-01", model=MODEL, supported_actions=[], initial_battery=50.0, battery_charge_percent_per_s=50.0)
    await fleet_factory([cfg])

    start = make_instant_actions(model=MODEL, serial=cfg.id, actions=[make_action("c1", "startCharging")])
    await publish_instant_actions(fm, TEST_PREFIX, MODEL, cfg.id, start)
    await asyncio.sleep(0.3)

    states = await collect_states(fm, TEST_PREFIX, MODEL, cfg.id, 1)
    charged_soc = states[-1].powerSupply.stateOfCharge
    assert charged_soc > 55.0  # ~50%/s * 0.3s = +15%
    assert states[-1].powerSupply.charging is True

    async with state_listener(fm, TEST_PREFIX, MODEL, cfg.id) as listener:
        stop = make_instant_actions(model=MODEL, serial=cfg.id, actions=[make_action("c2", "stopCharging")])
        await publish_instant_actions(fm, TEST_PREFIX, MODEL, cfg.id, stop)

        # Sample twice *after* charging is confirmed off — avoids racing the
        # NATS round-trip window where a few more charge-ticks can land
        # between "stop published" and it actually taking effect.
        stopped = await listener.wait_for(lambda s: s.powerSupply.charging is False)
        soc_at_stop = stopped.powerSupply.stateOfCharge

        await asyncio.sleep(0.3)
        states = await collect_states(fm, TEST_PREFIX, MODEL, cfg.id, 1)
        assert states[-1].powerSupply.charging is False
        assert states[-1].powerSupply.stateOfCharge == soc_at_stop  # no further increase


async def test_field_violation_halts_movement_then_clears(fm, fleet_factory):
    cfg = RobotConfig(
        id="fv-bot-01", model=MODEL, supported_actions=[], fault_profile=FaultProfile(field_violation_probability=1.0)
    )
    fleet = await fleet_factory([cfg])
    agv = fleet.runtimes[cfg.id].agv

    nodes, edges = make_route([(0.0, 0.0), (1.0, 0.0)])
    order = make_order(order_id="fv-1", order_update_id=0, model=MODEL, serial=cfg.id, nodes=nodes, edges=edges)

    async with state_listener(fm, TEST_PREFIX, MODEL, cfg.id) as listener:
        await publish_order(fm, TEST_PREFIX, MODEL, cfg.id, order)

        matched = await listener.wait_for(lambda s: s.safetyState.fieldViolation is True)
        assert matched.driving is False
        x_during_violation = matched.mobileRobotPosition.x

        # Stop re-triggering (prob=1.0 would otherwise re-violate the instant
        # it clears) so it can actually clear and stay clear.
        agv.cfg.fault_profile.field_violation_probability = 0.0

        await listener.wait_for(lambda s: s.safetyState.fieldViolation is False)

    await asyncio.sleep(0.3)
    states = await collect_states(fm, TEST_PREFIX, MODEL, cfg.id, 1)
    assert states[-1].mobileRobotPosition.x > x_during_violation  # resumed moving


async def test_service_mode_fault_suspends_order_then_recovers(fm, fleet_factory):
    cfg = RobotConfig(
        id="mode-bot-01", model=MODEL, supported_actions=[], fault_profile=FaultProfile(service_mode_probability=1.0)
    )
    fleet = await fleet_factory([cfg])
    agv = fleet.runtimes[cfg.id].agv

    nodes, edges = make_route([(0.0, 0.0), (1.0, 0.0)])
    order = make_order(order_id="mode-1", order_update_id=0, model=MODEL, serial=cfg.id, nodes=nodes, edges=edges)

    async with state_listener(fm, TEST_PREFIX, MODEL, cfg.id) as listener:
        await publish_order(fm, TEST_PREFIX, MODEL, cfg.id, order)

        matched = await listener.wait_for(lambda s: s.operatingMode != OperatingMode.AUTOMATIC)
        assert matched.operatingMode == OperatingMode.SERVICE
        assert matched.driving is False
        x_during_fault = matched.mobileRobotPosition.x

        agv.cfg.fault_profile.service_mode_probability = 0.0

        await listener.wait_for(lambda s: s.operatingMode == OperatingMode.AUTOMATIC)

    await asyncio.sleep(0.3)
    states = await collect_states(fm, TEST_PREFIX, MODEL, cfg.id, 1)
    assert states[-1].mobileRobotPosition.x > x_during_fault  # order execution resumed


async def test_retriable_action_fails_then_retries_and_recovers(fm, fleet_factory):
    # Real spec state machine (Table 4/5 + line 1258-1274): a retriable action
    # that fails while RUNNING goes to RETRIABLE, not FAILED — and only
    # leaves RETRIABLE via an explicit `retry`/`skipRetry` instant action,
    # never an automatic timer.
    cfg = RobotConfig(
        id="retry-bot-01", model=MODEL, supported_actions=["pick"], fault_profile=FaultProfile(error_injection_probability=1.0)
    )
    fleet = await fleet_factory([cfg])
    agv = fleet.runtimes[cfg.id].agv

    action = make_action("retry-a0", "pick", blocking=BlockingType.HARD, retriable=True)
    nodes = [make_node("n0", 0, 0.0, 0.0, actions=[action])]
    order = make_order(order_id="retry-1", order_update_id=0, model=MODEL, serial=cfg.id, nodes=nodes, edges=[])

    async with state_listener(fm, TEST_PREFIX, MODEL, cfg.id) as listener:
        await publish_order(fm, TEST_PREFIX, MODEL, cfg.id, order)

        retriable = await listener.wait_for(
            lambda s: any(a.actionId == "retry-a0" and a.actionStatus == ActionStatus.RETRIABLE for a in s.actionStates)
        )
        assert retriable is not None

        # Stop refailing it, then have fleet control explicitly retry —
        # matches the real spec mechanism, not an automatic timer.
        agv.cfg.fault_profile.error_injection_probability = 0.0
        retry = make_instant_actions(
            model=MODEL, serial=cfg.id, actions=[make_action_param("retry-cmd", "retry", {"actionId": "retry-a0"})]
        )
        await publish_instant_actions(fm, TEST_PREFIX, MODEL, cfg.id, retry)

        await listener.wait_for(
            lambda s: any(a.actionId == "retry-a0" and a.actionStatus == ActionStatus.FINISHED for a in s.actionStates)
        )


async def test_skip_retry_fails_action_permanently(fm, fleet_factory):
    cfg = RobotConfig(
        id="skip-retry-bot-01", model=MODEL, supported_actions=["pick"], fault_profile=FaultProfile(error_injection_probability=1.0)
    )
    await fleet_factory([cfg])

    action = make_action("skip-a0", "pick", blocking=BlockingType.HARD, retriable=True)
    nodes = [make_node("n0", 0, 0.0, 0.0, actions=[action])]
    order = make_order(order_id="skip-1", order_update_id=0, model=MODEL, serial=cfg.id, nodes=nodes, edges=[])

    async with state_listener(fm, TEST_PREFIX, MODEL, cfg.id) as listener:
        await publish_order(fm, TEST_PREFIX, MODEL, cfg.id, order)
        await listener.wait_for(
            lambda s: any(a.actionId == "skip-a0" and a.actionStatus == ActionStatus.RETRIABLE for a in s.actionStates)
        )

        skip = make_instant_actions(
            model=MODEL, serial=cfg.id, actions=[make_action_param("skip-cmd", "skipRetry", {"actionId": "skip-a0"})]
        )
        await publish_instant_actions(fm, TEST_PREFIX, MODEL, cfg.id, skip)

        await listener.wait_for(
            lambda s: any(a.actionId == "skip-a0" and a.actionStatus == ActionStatus.FAILED for a in s.actionStates)
        )


async def test_non_retriable_running_action_not_marked_failed_on_fault(fm, fleet_factory):
    cfg = RobotConfig(
        id="fault-bot-01", model=MODEL, supported_actions=["pick"], fault_profile=FaultProfile(error_injection_probability=1.0)
    )
    await fleet_factory([cfg])

    action = make_action("plain-a0", "pick", blocking=BlockingType.HARD, retriable=None)
    nodes = [make_node("n0", 0, 0.0, 0.0, actions=[action])]
    order = make_order(order_id="fault-1", order_update_id=0, model=MODEL, serial=cfg.id, nodes=nodes, edges=[])

    async with state_listener(fm, TEST_PREFIX, MODEL, cfg.id) as listener:
        await publish_order(fm, TEST_PREFIX, MODEL, cfg.id, order)

        def _has_fault_while_action_running(s):
            has_fault = any(e.errorType == "hardwareFault" for e in s.errors)
            action = next((a for a in s.actionStates if a.actionId == "plain-a0"), None)
            return has_fault and action is not None and action.actionStatus == ActionStatus.RUNNING

        matched = await listener.wait_for(_has_fault_while_action_running)
        running = next(a for a in matched.actionStates if a.actionId == "plain-a0")
        assert running.actionStatus == ActionStatus.RUNNING  # not FAILED — retry only applies to retriable actions
