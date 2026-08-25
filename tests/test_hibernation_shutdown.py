"""Gap-closing pass: hibernation/shutdown/stateRequest/clear*/logReport/
updateCertificate — spec Section 6.2.3, Table 4 predefined actions previously
missing entirely."""

from __future__ import annotations

import asyncio

from conftest import test_settings as build_settings
from helpers import (
    TEST_PREFIX,
    connection_listener,
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

from vda5050_sim.agv import RobotConfig
from vda5050_sim.schemas import ActionStatus, ConnectionState

MODEL, SERIAL = "spot", "test-spot-01"


async def test_start_stop_hibernation_ignores_commands_while_hibernating(running_fleet, fm):
    agv = running_fleet.runtimes[SERIAL].agv
    start = make_instant_actions(model=MODEL, serial=SERIAL, actions=[make_action("h1", "startHibernation")])
    await publish_instant_actions(fm, TEST_PREFIX, MODEL, SERIAL, start)
    await asyncio.sleep(0.1)
    assert agv.hibernating is True

    # While hibernating, orders and non-stopHibernation instant actions are
    # ignored outright (spec: "shall not respond to any other commands").
    order = make_order(
        order_id="ignored-1", order_update_id=0, model=MODEL, serial=SERIAL, nodes=[make_node("n0", 0, 1.0, 0.0)], edges=[]
    )
    await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, order)
    await asyncio.sleep(0.1)
    assert agv.order_id == ""

    stop = make_instant_actions(model=MODEL, serial=SERIAL, actions=[make_action("h2", "stopHibernation")])
    await publish_instant_actions(fm, TEST_PREFIX, MODEL, SERIAL, stop)
    await asyncio.sleep(0.1)
    assert agv.hibernating is False


async def test_shutdown_requires_idle_robot(running_fleet, fm):
    async with state_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        nodes, edges = make_route([(0.0, 0.0), (50.0, 0.0)])
        order = make_order(order_id="busy-1", order_update_id=0, model=MODEL, serial=SERIAL, nodes=nodes, edges=edges)
        await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, order)
        await asyncio.sleep(0.05)

        shutdown = make_instant_actions(model=MODEL, serial=SERIAL, actions=[make_action("sd1", "shutdown")])
        await publish_instant_actions(fm, TEST_PREFIX, MODEL, SERIAL, shutdown)

        matched = await listener.wait_for(
            lambda s: any(a.actionId == "sd1" and a.actionStatus == ActionStatus.FAILED for a in s.instantActionStates)
        )
        assert matched is not None


async def test_shutdown_from_idle_publishes_offline(fm, fleet_factory):
    cfg = RobotConfig(id="shutdown-bot-01", model=MODEL, supported_actions=[])
    fleet = await fleet_factory([cfg])
    await asyncio.sleep(0.1)

    async with connection_listener(fm, TEST_PREFIX, MODEL, cfg.id) as listener:
        shutdown = make_instant_actions(model=MODEL, serial=cfg.id, actions=[make_action("sd2", "shutdown")])
        await publish_instant_actions(fm, TEST_PREFIX, MODEL, cfg.id, shutdown)

        matched = await listener.wait_for(lambda c: c.connectionState == ConnectionState.OFFLINE)
        assert matched is not None
    assert fleet.runtimes[cfg.id].agv.shutdown_requested is True


async def test_state_request_forces_immediate_publish(fm, fleet_factory):
    cfg = RobotConfig(id="staterequest-bot-01", model=MODEL, supported_actions=[])
    settings = build_settings(state_hz=0.5)  # slow cadence — stateRequest must still wake it fast
    await fleet_factory([cfg], settings)

    async with state_listener(fm, TEST_PREFIX, MODEL, cfg.id) as listener:
        listener.received.clear()
        req = make_instant_actions(model=MODEL, serial=cfg.id, actions=[make_action("sr1", "stateRequest")])
        await publish_instant_actions(fm, TEST_PREFIX, MODEL, cfg.id, req)
        # Would take up to 2s at the configured 0.5 state_hz if stateRequest
        # didn't wake the loop early.
        matched = await listener.wait_for(lambda s: True, timeout=1.0)
        assert matched is not None


async def test_clear_instant_actions_removes_terminal_entries(running_fleet, fm):
    agv = running_fleet.runtimes[SERIAL].agv
    pause = make_instant_actions(model=MODEL, serial=SERIAL, actions=[make_action("cia-p1", "startPause")])
    await publish_instant_actions(fm, TEST_PREFIX, MODEL, SERIAL, pause)
    await asyncio.sleep(0.05)
    assert any(s.actionId == "cia-p1" for s in agv.instant_action_states)

    clear = make_instant_actions(model=MODEL, serial=SERIAL, actions=[make_action("cia-clear", "clearInstantActions")])
    await publish_instant_actions(fm, TEST_PREFIX, MODEL, SERIAL, clear)
    await asyncio.sleep(0.05)
    assert not any(s.actionId == "cia-p1" for s in agv.instant_action_states)
    # resume, else the robot is left paused for subsequent tests sharing this fixture
    resume = make_instant_actions(model=MODEL, serial=SERIAL, actions=[make_action("cia-resume", "stopPause")])
    await publish_instant_actions(fm, TEST_PREFIX, MODEL, SERIAL, resume)


async def test_log_report_finishes_with_log_name(running_fleet, fm):
    async with state_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        log = make_instant_actions(
            model=MODEL, serial=SERIAL, actions=[make_action_param("lr1", "logReport", {"reason": "diagnostics"})]
        )
        await publish_instant_actions(fm, TEST_PREFIX, MODEL, SERIAL, log)

        matched = await listener.wait_for(
            lambda s: any(a.actionId == "lr1" and a.actionStatus == ActionStatus.FINISHED for a in s.instantActionStates)
        )
        state = next(a for a in matched.instantActionStates if a.actionId == "lr1")
        assert state.actionResult and state.actionResult.startswith("log_")


async def test_update_certificate_simulated_lifecycle(running_fleet, fm):
    async with state_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        action = make_action_param(
            "uc1",
            "updateCertificate",
            {"service": "MQTT", "keyDownloadLink": "https://example.test/key", "certificateDownloadLink": "https://example.test/cert"},
        )
        msg = make_instant_actions(model=MODEL, serial=SERIAL, actions=[action])
        await publish_instant_actions(fm, TEST_PREFIX, MODEL, SERIAL, msg)

        matched = await listener.wait_for(
            lambda s: any(a.actionId == "uc1" and a.actionStatus == ActionStatus.FINISHED for a in s.instantActionStates)
        )
        assert matched is not None
