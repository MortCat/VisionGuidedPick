# Included assets and external acquisition

| Content | Source / terms |
| --- | --- |
| Conveyor, room/layout, blocks, boxes, camera brackets and authored materials | MortCat / VisionGuidedPick, original contributions under MIT |
| Original bear sculpt | Project-authored; [design references](../../assets/design/teddy_bear/README.md). Retailer photographs are excluded |
| Panda ROS descriptions and meshes | Fixed MoveIt Resources source; [retained license](../../third_party/panda/LICENSE), [source and checksums](../../third_party/panda/source.json) |
| Bunny geometry / references / physical variants | Stanford Computer Graphics Laboratory, Greg Turk and Marc Levoy. Separate research/free-redistribution terms; commercial permission required. [Stanford source](https://graphics.stanford.edu/data/3Dscanrep/#acknowledge) |
| NVIDIA robot/camera USD, materials and textures | Excluded. Acquire the pinned files independently using an authorized NVIDIA source and `prepare_scene_assets.py` (official download by default; authorized local source also supported) |
| SAM / RF-DETR base weights and training dependencies | Upstream acquisition; excluded from this source repository |
| Training images, trained weights and experiment outputs | Excluded from source; a standalone inference-only Bunny model is available as a separate Release asset |

The composed scene retains its relative references, including inactive branches,
so the required local dependencies remain present. The manifest records all
125 dependency entries and two built-in MDL names supplied by Isaac Sim.
The 54 excluded NVIDIA files are pinned by checksum and ignored by Git after
local staging. The repository does not bundle the simulator.

See [third-party notices](../../THIRD_PARTY_NOTICES.md) and the
[installation guide](../getting-started.md). Source and authored assets do not
imply that the entire combined scene or its derivatives are uniformly MIT.
