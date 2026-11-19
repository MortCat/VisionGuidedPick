# Stanford Bunny — original geometry with wood material

This version preserves the official Stanford `bun_zipper.ply` geometry.
It is separate from the repaired RabbitGood v1 and does not replace that asset.

- 35,947 vertices and 69,451 triangles, retaining the source face indices.
- No remeshing, smoothing modifier, ear reconstruction, hole filling or base cut.
- Only a rigid orientation change, uniform scaling to 75 mm height, translation
  to a base-centered origin, UV coordinates and wood material are added.
- Source holes, original ear chip and all scanned surface details remain.
- Meters, Z up, nominal front -Y. No physics is enabled.

Files:

- `stanford_bunny_original_wood.blend`: editable model and preview scene.
- `stanford_bunny_original_wood.usdc`: visual asset; keep `textures/` beside it.
- `stanford_bunny_original_wood_asset.zip`: portable runtime bundle.
- `stanford_bunny_original_reference.ply`: transformed original reference mesh.
- `rabbit_hero.png`, `rabbit_side.png`: previews.
- `validation.json`: source fidelity checks.
- `original_and_repaired_in_factory.png`: live Isaac Sim comparison.

Credit: Stanford University Computer Graphics Laboratory; Greg Turk and Marc Levoy.
Source: https://graphics.stanford.edu/data/3Dscanrep/
Official PLY SHA-256:
`b1acc63bece78444aa2e15bdcc72371a201279b98c6f5d4b74c993d02f0566fe`

Source terms allow research use and free redistribution with acknowledgement.
Commercial use requires permission from the Stanford source owners.

Build with Blender 5.2:

```powershell
& '<Blender installation>/blender.exe' --background --factory-startup `
  --python '<asset directory>/build_original.py' -- '<asset directory>'
```

The build reads the official PLY preserved in the adjacent `rabbit_good_v1`
directory. Placement writes to the current Isaac Sim session layer and never
saves or overwrites the factory stage.
