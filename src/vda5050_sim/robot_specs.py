"""Real physical specifications for the default fleet's robot archetypes,
sourced from each manufacturer's public datasheet — used to build accurate
VDA5050 factsheets and to derive a realistic simulated top speed.

Footprint polygons are simple rectangles derived from length/width, centred
on the robot origin. All dimensions in metres, speeds in m/s.
"""

from __future__ import annotations

from typing import Any

from vda5050_sim.action_catalog import ACTION_CATALOG, ActionDef
from vda5050_sim.schemas import (
    ActionParameterDefinition,
    ActionScope,
    Envelope2d,
    FactsheetMessage,
    LoadSpecification,
    MaximumArrayLengths,
    MaximumStringLengths,
    MobileRobotActionDef,
    MobileRobotGeometry,
    PhysicalParameters,
    ProtocolFeatures,
    ProtocolLimits,
    Timing,
    TypeSpecification,
    Vertex2d,
)

ROBOT_SPECS: dict[str, dict[str, Any]] = {
    "spot": {
        "manufacturer": "BostonDynamics",
        "seriesName": "Spot",
        "seriesDescription": "Boston Dynamics Spot quadruped robot",
        "mobileRobotKinematics": "OMNIDIRECTIONAL",
        "mobileRobotClass": "CARRIER",
        "maximumLoadMass": 14.0,
        "localizationTypes": ["NATURAL"],
        "navigationTypes": ["FREELY_NAVIGATING"],
        "maximumSpeed": 1.6,
        "maximumAngularSpeed": 1.5,
        "maximumAcceleration": 2.0,
        "maximumDeceleration": 2.0,
        "length": 1.1,
        "width": 0.5,
        "minimumHeight": 0.19,
        "maximumHeight": 0.61,
    },
    "go2": {
        "manufacturer": "Unitree",
        "seriesName": "Go2",
        "seriesDescription": "Unitree Go2 quadruped robot",
        "mobileRobotKinematics": "OMNIDIRECTIONAL",
        "mobileRobotClass": "CARRIER",
        "maximumLoadMass": 8.0,
        "localizationTypes": ["NATURAL"],
        "navigationTypes": ["FREELY_NAVIGATING"],
        "maximumSpeed": 3.5,
        "maximumAngularSpeed": 2.5,
        "maximumAcceleration": 2.0,
        "maximumDeceleration": 2.0,
        "length": 0.70,
        "width": 0.31,
        "minimumHeight": 0.20,
        "maximumHeight": 0.40,
    },
    "tiago": {
        "manufacturer": "PALRobotics",
        "seriesName": "TIAGo",
        "seriesDescription": "PAL Robotics TIAGo mobile manipulator",
        "mobileRobotKinematics": "DIFFERENTIAL",
        "mobileRobotClass": "CARRIER",
        "maximumLoadMass": 5.0,
        "localizationTypes": ["NATURAL"],
        "navigationTypes": ["FREELY_NAVIGATING"],
        "maximumSpeed": 1.0,
        "maximumAngularSpeed": 1.5,
        "maximumAcceleration": 1.0,
        "maximumDeceleration": 1.0,
        "length": 0.54,
        "width": 0.54,
        "minimumHeight": 1.10,
        "maximumHeight": 1.10,
    },
    "h1": {
        "manufacturer": "Unitree",
        "seriesName": "H1",
        "seriesDescription": "Unitree H1 humanoid robot",
        "mobileRobotKinematics": "OMNIDIRECTIONAL",
        "mobileRobotClass": "CARRIER",
        "maximumLoadMass": 30.0,
        "localizationTypes": ["NATURAL"],
        "navigationTypes": ["FREELY_NAVIGATING"],
        "maximumSpeed": 3.3,
        "maximumAngularSpeed": 2.0,
        "maximumAcceleration": 2.0,
        "maximumDeceleration": 2.0,
        "length": 0.45,
        "width": 0.40,
        "minimumHeight": 1.0,
        "maximumHeight": 1.80,
    },
    "pidog": {
        "manufacturer": "SunFounder",
        "seriesName": "PiDog",
        "seriesDescription": "SunFounder PiDog robot dog on Raspberry Pi",
        "mobileRobotKinematics": "DIFFERENTIAL",
        "mobileRobotClass": "CARRIER",
        "maximumLoadMass": 0.0,
        "localizationTypes": ["NATURAL"],
        "navigationTypes": ["FREELY_NAVIGATING"],
        "maximumSpeed": 0.3,
        "maximumAngularSpeed": 1.0,
        "maximumAcceleration": 0.5,
        "maximumDeceleration": 0.5,
        "length": 0.20,
        "width": 0.12,
        "minimumHeight": 0.08,
        "maximumHeight": 0.15,
    },
}


def get_manufacturer(robot_model: str) -> str:
    spec = ROBOT_SPECS.get(robot_model.lower())
    return spec["manufacturer"] if spec else "Generic"


def _rect_footprint(length: float, width: float) -> list[Vertex2d]:
    hl, hw = length / 2, width / 2
    return [
        Vertex2d(x=hl, y=hw),
        Vertex2d(x=hl, y=-hw),
        Vertex2d(x=-hl, y=-hw),
        Vertex2d(x=-hl, y=hw),
    ]


def action_def(
    action_type: str,
    description: str = "",
    *,
    scopes: list[ActionScope] | None = None,
    blocking: list[str] | None = None,
    pause_allowed: str = "false",
    cancel_allowed: str = "true",
) -> MobileRobotActionDef:
    return MobileRobotActionDef(
        actionType=action_type,
        actionDescription=description or None,
        actionScopes=scopes or [ActionScope.INSTANT],
        blockingTypes=blocking or ["NONE"],
        pauseAllowed=pause_allowed,
        cancelAllowed=cancel_allowed,
    )


# action_catalog.py's ActionParamDef.value_data_type uses the spec's own
# Table 4 parameter-column notation ("float64", "string", "string[]") for
# doc value — the *factsheet wire format* requires a different, fixed enum
# (factsheet.schema.json: BOOL/NUMBER/INTEGER/STRING/OBJECT/ARRAY), so it's
# translated here rather than duplicating Table 4's notation.
_VALUE_DATA_TYPE_MAP = {
    "string": "STRING",
    "float64": "NUMBER",
    "string[]": "ARRAY",
}


def factsheet_action_def(entry: ActionDef) -> MobileRobotActionDef:
    """Build a MobileRobotActionDef straight from a verified action_catalog
    entry — actionType/scopes/params come from the spec's Table 4; blocking/
    pause/cancel capability flags are this simulator's own declared choices
    (see action_catalog.py's module docstring)."""
    return MobileRobotActionDef(
        actionType=entry.action_type,
        actionDescription=entry.description,
        actionScopes=list(entry.scopes),
        blockingTypes=list(entry.blocking_types),
        pauseAllowed="true" if entry.pause_allowed else "false",
        cancelAllowed="true" if entry.cancel_allowed else "false",
        actionParameters=[
            ActionParameterDefinition(
                key=p.key,
                valueDataType=_VALUE_DATA_TYPE_MAP.get(p.value_data_type, "STRING"),
                description=p.description or None,
                isOptional=p.optional,
            )
            for p in entry.params
        ],
    )


def build_supported_actions(supported_actions: list[str]) -> list[MobileRobotActionDef]:
    """Every `core=True` catalog action (always available) plus this robot's
    configured optional capabilities — catalog-backed ones get accurate
    scope/param metadata, manufacturer-custom ones (not in the VDA5050
    standard vocabulary, e.g. a robot-specific gesture action) fall back to a
    generic instant-only declaration."""
    core = [factsheet_action_def(e) for e in ACTION_CATALOG.values() if e.core]
    custom = [
        factsheet_action_def(ACTION_CATALOG[a]) if a in ACTION_CATALOG else action_def(a, a) for a in supported_actions
    ]
    return core + custom


def build_factsheet(
    robot_model: str,
    robot_id: str,
    header_id: int,
    timestamp: str,
    *,
    supported_actions: list[MobileRobotActionDef] | None = None,
) -> FactsheetMessage:
    spec = ROBOT_SPECS.get(robot_model.lower(), {})
    manufacturer = get_manufacturer(robot_model)
    length = spec.get("length", 0.5)
    width = spec.get("width", 0.5)

    if supported_actions is None:
        supported_actions = build_supported_actions([])

    return FactsheetMessage(
        headerId=header_id,
        timestamp=timestamp,
        manufacturer=manufacturer,
        serialNumber=robot_id,
        typeSpecification=TypeSpecification(
            seriesName=spec.get("seriesName", robot_model),
            seriesDescription=spec.get("seriesDescription"),
            mobileRobotKinematics=spec.get("mobileRobotKinematics"),
            mobileRobotClass=spec.get("mobileRobotClass", "CARRIER"),
            maximumLoadMass=spec.get("maximumLoadMass", 0.0),
            localizationTypes=spec.get("localizationTypes", ["NATURAL"]),
            navigationTypes=spec.get("navigationTypes", ["FREELY_NAVIGATING"]),
        ),
        physicalParameters=PhysicalParameters(
            minimumSpeed=0.0,
            maximumSpeed=spec.get("maximumSpeed", 0.0),
            minimumAngularSpeed=0.0,
            maximumAngularSpeed=spec.get("maximumAngularSpeed"),
            maximumAcceleration=spec.get("maximumAcceleration", 0.0),
            maximumDeceleration=spec.get("maximumDeceleration", 0.0),
            minimumHeight=spec.get("minimumHeight", 0.0),
            maximumHeight=spec.get("maximumHeight", 0.0),
            width=width,
            length=length,
        ),
        protocolLimits=ProtocolLimits(
            maximumStringLengths=MaximumStringLengths(
                maximumIdLength=256, maximumTopicSerialLength=256, maximumTopicElementLength=256
            ),
            maximumArrayLengths=MaximumArrayLengths(instantActions=10),
            timing=Timing(
                minimumOrderInterval=0.5,
                minimumStateInterval=0.5,
                defaultStateInterval=1.0,
                visualizationInterval=0.2,
            ),
        ),
        protocolFeatures=ProtocolFeatures(mobileRobotActions=supported_actions, optionalParameters=[]),
        mobileRobotGeometry=MobileRobotGeometry(
            envelopes2d=[
                Envelope2d(
                    envelope2dId="footprint",
                    description=f"Rectangular footprint {length}m x {width}m",
                    vertices=_rect_footprint(length, width),
                )
            ]
        ),
        loadSpecification=LoadSpecification(),
    )
