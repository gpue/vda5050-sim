# Vendored VDA5050 JSON Schemas

Sourced from the official standard, unmodified except for fixing unambiguous
upstream typos: [github.com/VDA5050/VDA5050](https://github.com/VDA5050/VDA5050),
tag `3.0.0`, `json_schemas/*.schema` (MIT licensed).

`factsheet.schema`, `order.schema`, and `visualization.schema` each contain a
trailing-comma JSON syntax error in the upstream 3.0.0 release itself (they
don't parse as JSON at all as published) — fixed here so they're actually
usable.

`zoneSet.schema`'s `PRIORITY`/`PENALTY` zone conditional blocks require
`priorityFactor`/`penaltyFactor` in their `required` array, but only define
the property under a key with a stray trailing space (`"priorityFactor "`,
`"penaltyFactor "`) — no payload could ever satisfy that schema section as
literally published. Fixed here (trailing space removed) for the same reason
as the trailing-comma fixes above: an unambiguous typo, not a meaningful spec
choice.

No other content was changed. See `tests/test_schema_validation.py` for
further known-but-unfixable upstream issues (a self-contradictory
`connectionState` enum, and a `typeSpecification` required-field name typo)
that are documented there instead of silently patched.
