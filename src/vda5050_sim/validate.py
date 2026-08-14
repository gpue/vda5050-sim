"""JSON Schema validation against the vendored official VDA5050 schemas."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import jsonschema

_SCHEMA_DIR = Path(__file__).parent / "json_schemas"
_schema_cache: dict[str, dict] = {}


def _load_schema(name: str) -> dict:
    if name not in _schema_cache:
        path = _SCHEMA_DIR / f"{name}.schema.json"
        if not path.exists():
            raise FileNotFoundError(f"VDA5050 schema not found: {path}")
        _schema_cache[name] = json.loads(path.read_text())
    return _schema_cache[name]


def validate_message(data: dict[str, Any], message_type: str) -> list[str]:
    """Validate a dict against a VDA5050 JSON schema.

    Args:
        data: Message payload as a dict.
        message_type: One of 'state', 'order', 'instantActions', 'connection',
                      'visualization', 'factsheet'.

    Returns:
        List of validation error messages (empty if valid).
    """
    schema = _load_schema(message_type)
    validator = jsonschema.Draft7Validator(schema)
    return [err.message for err in validator.iter_errors(data)]
