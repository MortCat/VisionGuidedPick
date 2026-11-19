# Third-party notices and licensing scope

Original VisionGuidedPick code, documentation, authored USD layers, original
meshes and materials are MIT licensed, copyright 2026 MortCat. Internal `ptc_`
package names are retained. This does not relicense third-party content.

## Panda robot descriptions and meshes

The 20 unmodified Panda meshes under `config/factory_demo/assets/meshes/` were
byte-verified against MoveIt Resources revision
`e1657c128ea61621777a041e3783c9c7c93db4ca`. The upstream `panda_description/LICENSE`
is retained at [third_party/panda/LICENSE](third_party/panda/LICENSE), with
[source checksums and modification notes](third_party/panda/source.json) and
the [upstream package metadata](third_party/panda/package.xml).

Upstream package metadata declares BSD while its directory license is
Apache-2.0; both source records are preserved. Panda descriptions originate
from the Franka/MoveIt resources. Project changes include namespaced links,
TCP/camera mounts, configured limits and finger-pad geometry. Project
modifications do not remove upstream rights or notices.

## Stanford Bunny

Stanford Bunny geometry and derivatives remain subject to the
[Stanford 3D Scanning Repository conditions](https://graphics.stanford.edu/data/3Dscanrep/#acknowledge),
not MIT. Credit: Stanford University Computer Graphics Laboratory, Greg Turk
and Marc Levoy. Stanford allows research use and free redistribution with
acknowledgement; commercial use requires its permission. The original data
contains holes and imperfections.

Affected content includes `config/factory_demo/assets/stanford_good.ply`,
other Bunny reference clouds, rabbit meshes under
`deployment/factory_rabbit_20261005/assets/workspace/GeneralForAgent/assets/`,
their physical/wood/defect variants, and media depicting these models.
Orientation, scale, UV/material and physical-wrapper changes are project
contributions; they do not convert the underlying geometry to MIT.
See the [original geometry notes](deployment/factory_rabbit_20261005/assets/workspace/GeneralForAgent/assets/rabbit_original_wood/v2/README.md).

## NVIDIA simulator assets

NVIDIA Panda/D455 USD, MDL and textures are **not bundled**. Users obtain the
54 pinned files through an authorized NVIDIA source and run the staging
helper, or run `python3 prepare_scene_assets.py` to acquire them directly from NVIDIA. Their paths and hashes are in
[isaac_assets.sha256.json](deployment/runtime/isaac_assets.sha256.json).
Consult the terms accompanying those files and the
[Isaac Sim additional materials license](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/common/license-isaac-sim-additional.html).
References in an authored USD layer do not grant rights over its dependencies.

## ML dependencies and weights

SAM 2.1 is fetched by the setup script at a pinned source revision, with a
verified checkpoint hash. See [Meta's license and notices](https://github.com/facebookresearch/sam2#license).
RF-DETR 1.9.1 designates RFDETRSegSmall and its weights as Apache-2.0; see
[the versioned upstream license](https://github.com/roboflow/rf-detr/blob/1.9.1/LICENSE)
and [model table](https://github.com/roboflow/rf-detr/tree/1.9.1#segmentation).
The demo model's MODEL_CARD.md includes the full upstream license and identifies
the inference-only export changes. Training dependencies retain their own terms.
No upstream source checkout, checkpoint or training dataset is bundled in this
source tree. The separate demo-model Release retains RF-DETR and depicted-content
terms; it is not relicensed wholesale under MIT. A wrapper's MIT license does not replace their terms.

## Authored bear and documentation media

The bear sculpt and blocks are project-authored. Preserve the
[bear design-reference notes](assets/design/teddy_bear/README.md); no retailer
photographs or manufacturer meshes are bundled. Screenshots show actual
simulation/UI content; [media credits](docs/media/README.md) identify their
third-party scene content. The MIT grant applies to original contributions,
without relicensing depicted third-party models.
