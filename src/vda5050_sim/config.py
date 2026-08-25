from __future__ import annotations

from typing import Literal

from pydantic import AliasChoices, Field
from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    model_config = SettingsConfigDict(env_prefix="", extra="ignore")

    # "nats" (default): run as a Nova app on the platform's NATS bus, wire-
    # compatible with nova-nav's VDA5050 dashboard.
    # "mqtt": standalone mode — speak plain VDA5050-over-MQTT to any broker,
    # so any real fleet manager can connect without Nova/NATS at all.
    transport: Literal["nats", "mqtt"] = "nats"

    # -- NATS (transport=nats) --------------------------------------------
    # NATS_URL is accepted as a fallback alias so this matches nova-nav's
    # own env convention (NATS_BROKER preferred, NATS_URL fallback).
    nats_broker: str = Field(
        default="nats://localhost:4222",
        validation_alias=AliasChoices("NATS_BROKER", "NATS_URL"),
    )
    vda5050_prefix: str = "vda5050.v3"

    # -- MQTT (transport=mqtt) --------------------------------------------
    mqtt_host: str = "localhost"
    mqtt_port: int = 1883
    mqtt_username: str | None = None
    mqtt_password: str | None = None
    mqtt_tls: bool = False
    # Standard VDA5050 topic layout: {prefix}/{manufacturer}/{serialNumber}/{topic}
    mqtt_topic_prefix: str = "vda5050/v3"

    fleet_config_path: str = "fleet.default.yaml"

    # Publish cadences (Hz). No MQTT-style retained/LWT over plain NATS core,
    # so connection/state/visualization are heartbeated continuously.
    state_hz: float = 1.0
    visualization_hz: float = 2.0
    connection_heartbeat_s: float = 2.0

    # Movement/order-processing tick.
    tick_s: float = 0.2
    # Fixed duration for a simulated node/edge/instant action (WAITING->RUNNING->FINISHED).
    action_duration_s: float = 1.0
    # Simulated linear travel speed when no per-robot override is set (m/s).
    default_speed_mps: float = 0.5
    # Simulated angular travel speed when no per-robot override is set (rad/s).
    default_angular_speed_rad_s: float = 1.0
    # Proactively set newBaseRequest once remaining released nodes drop to/below
    # this count, rather than only once an unreleased node/edge is actually hit.
    horizon_threshold_nodes: int = 2

    # -- Telemetry realism (Phase 4) — cosmetic/resilience-testing, not
    # protocol correctness. Defaults keep drain "always on" for demo realism
    # but fault probabilities off (0.0) unless a robot's fleet.yaml entry
    # opts in via its own fault_profile.
    default_battery_drain_percent_per_meter: float = 0.05
    default_battery_charge_percent_per_s: float = 5.0

    base_path: str = ""
    port: int = 8000
    log_buffer_size: int = 500


def get_settings() -> Settings:
    return Settings()
