# Runtime deployment

Use the root Setup / Start / Stop entries described in
[Getting started](../../docs/getting-started.md) and the
[startup contract](../../docs/architecture/startup-setup-contract.md).

Profiles select native Linux or Windows simulator + WSL application. Host paths
belong in ignored local/locations.json; Windows selects its separate WSL checkout
through local/launcher.json. Source fingerprints exclude host configuration,
generated outputs and acquired vendor payloads. Those payloads use their pinned
asset verifier. Python inference environments and native builds are host-owned.
