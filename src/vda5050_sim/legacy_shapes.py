"""Downgrades already-serialized v3.0.0 `state`/`connection` dicts to the
wire shape a pre-3.0 (1.1.0/2.0.0/2.1.0) VDA5050 robot would actually send.

Those three older spec versions are structurally identical to each other for
the fields covered here — the only shape differences that matter sit at the
2.1.0 -> 3.0.0 boundary (confirmed against the VDA5050 project's own 3.0.0
release notes): `agvPosition` was renamed `mobileRobotPosition`,
`batteryState` was renamed `powerSupply`, `safetyState.eStop` was renamed
`safetyState.activeEmergencyStop`, `batteryCurrent` and the
`CONNECTIONBROKEN` -> `CONNECTION_BROKEN` spelling were introduced in 3.0.0.

The internal `SimulatedAgv` engine always builds v3.0.0-shaped Pydantic
messages regardless of a robot's configured `protocol_version` — these
functions run on the `.model_dump()`'d dict right before it goes over the
wire, so the simulation logic itself never needs a second code path.
"""

from __future__ import annotations

from typing import Any


def _is_legacy(version: str) -> bool:
    major = version.split(".", 1)[0]
    return major in ("1", "2")


def downgrade_state(d: dict[str, Any], version: str) -> dict[str, Any]:
    d = dict(d)
    d["version"] = version
    if not _is_legacy(version):
        return d

    pos = d.pop("mobileRobotPosition", None)
    if pos is not None:
        d["agvPosition"] = {
            "x": pos["x"],
            "y": pos["y"],
            "theta": pos["theta"],
            "mapId": pos["mapId"],
            "positionInitialized": pos.get("localized", True),
        }

    power = d.pop("powerSupply", None)
    if power is not None:
        battery_state = {
            "batteryCharge": power["stateOfCharge"],
            "charging": power["charging"],
        }
        if power.get("batteryVoltage") is not None:
            battery_state["batteryVoltage"] = power["batteryVoltage"]
        if power.get("batteryHealth") is not None:
            battery_state["batteryHealth"] = power["batteryHealth"]
        if power.get("range") is not None:
            battery_state["reach"] = power["range"]
        # batteryCurrent was only added in 3.0.0 — deliberately dropped.
        d["batteryState"] = battery_state

    safety = d.get("safetyState")
    if safety is not None:
        d["safetyState"] = {
            "eStop": safety["activeEmergencyStop"],
            "fieldViolation": safety["fieldViolation"],
        }

    return d


def downgrade_connection(d: dict[str, Any], version: str) -> dict[str, Any]:
    d = dict(d)
    d["version"] = version
    if not _is_legacy(version):
        return d

    if d.get("connectionState") == "CONNECTION_BROKEN":
        d["connectionState"] = "CONNECTIONBROKEN"
    elif d.get("connectionState") == "HIBERNATING":
        # This simulator never needs a legacy robot to hibernate — simplest
        # correct fallback rather than modeling a legacy hibernation flow.
        d["connectionState"] = "OFFLINE"

    return d
