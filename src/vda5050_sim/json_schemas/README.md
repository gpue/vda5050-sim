# Vendored VDA5050 JSON Schemas

Sourced from the official standard, unmodified except for one thing:
[github.com/VDA5050/VDA5050](https://github.com/VDA5050/VDA5050), tag `3.0.0`,
`json_schemas/*.schema` (MIT licensed).

`factsheet.schema`, `order.schema`, and `visualization.schema` each contain a
trailing-comma JSON syntax error in the upstream 3.0.0 release itself (they
don't parse as JSON at all as published) — fixed here so they're actually
usable; no other content was changed. See `tests/test_schema_validation.py`
for two further known-but-unfixable upstream issues (a self-contradictory
`connectionState` enum, and a `typeSpecification` required-field name typo)
that are documented there instead of silently patched.
