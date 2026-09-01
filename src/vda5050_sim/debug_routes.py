"""Debug-only endpoints for deterministic test scenarios.

Fault injection and battery drain/charge are otherwise entirely
probabilistic/tick-based (see agv.py's maybe_inject_fault and
fleet.default.yaml's fault_profile), which is realistic but not something a
test can wait on — there's no way to assert "the fleet manager correctly
shows a low-battery robot" without waiting out real probability. These
routes trigger the same effects immediately and deterministically, without
touching the core protocol-conformance simulation logic in agv.py/fleet.py
(only a couple of small public methods were added there for this to call).
"""

from __future__ import annotations

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel

from vda5050_sim import errors as sim_errors
from vda5050_sim.agv import (
    _EMERGENCY_STOP_DURATION_S,
    _FIELD_VIOLATION_DURATION_S,
    _SERVICE_MODE_FAULT_DURATION_S,
)
from vda5050_sim.fleet import Fleet, RobotRuntime
from vda5050_sim.schemas import ConnectionState, EmergencyStopType, OperatingMode

router = APIRouter()


def _get_runtime(request: Request, manufacturer: str, serial: str) -> RobotRuntime:
    fleet: Fleet = request.app.state.fleet
    runtime = fleet.find_runtime(manufacturer, serial)
    if runtime is None:
        raise HTTPException(status_code=404, detail=f"No robot {manufacturer}/{serial} in this fleet")
    return runtime


class SetBatteryRequest(BaseModel):
    percent: float


@router.post("/debug/{manufacturer}/{serial}/battery")
async def set_battery(manufacturer: str, serial: str, payload: SetBatteryRequest, request: Request) -> dict:
    runtime = _get_runtime(request, manufacturer, serial)
    runtime.agv.battery_soc = max(0.0, min(100.0, payload.percent))
    return {"status": "ok", "batterySoc": runtime.agv.battery_soc}


class SetConnectionRequest(BaseModel):
    state: ConnectionState


@router.post("/debug/{manufacturer}/{serial}/connection")
async def set_connection(manufacturer: str, serial: str, payload: SetConnectionRequest, request: Request) -> dict:
    runtime = _get_runtime(request, manufacturer, serial)
    await runtime.force_connection_state(payload.state)
    return {"status": "ok", "connectionState": payload.state.value}


class TriggerFaultRequest(BaseModel):
    type: str  # "hardware_fault" | "field_violation" | "service_mode" | "emergency_stop"


@router.post("/debug/{manufacturer}/{serial}/fault")
async def trigger_fault(manufacturer: str, serial: str, payload: TriggerFaultRequest, request: Request) -> dict:
    runtime = _get_runtime(request, manufacturer, serial)
    agv = runtime.agv

    if payload.type == "hardware_fault":
        # Same one-shot shape as the probabilistic path: appears in the
        # next state message's `errors`, then clears itself (see
        # SimulatedAgv.build_state_message clearing pending_errors after
        # every publish) — mirrors real transient-fault semantics.
        agv.pending_errors.append(sim_errors.make_error(sim_errors.HARDWARE_FAULT, "debug-triggered fault"))
    elif payload.type == "field_violation":
        agv.field_violation = True
        agv._field_violation_until = agv._fault_elapsed_s + _FIELD_VIOLATION_DURATION_S
    elif payload.type == "service_mode":
        agv.operating_mode = OperatingMode.SERVICE
        agv._operating_mode_fault_until = agv._fault_elapsed_s + _SERVICE_MODE_FAULT_DURATION_S
    elif payload.type == "emergency_stop":
        agv.emergency_stop = EmergencyStopType.MANUAL
        agv._emergency_stop_until = agv._fault_elapsed_s + _EMERGENCY_STOP_DURATION_S
    else:
        raise HTTPException(
            status_code=400,
            detail="type must be one of: hardware_fault, field_violation, service_mode, emergency_stop",
        )

    return {"status": "ok", "type": payload.type}
