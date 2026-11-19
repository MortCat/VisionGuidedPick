# Approved teddy bear C — 2026-10-30

User approved the shoulder/arm revision C. Height75 mm; original smooth seated sculpt mesh with small round ears, short muzzle, recessed eyes, carved mouth, relaxed arms and forward paws. FreeCAD review FCStd stores the mesh; it is not a parametric BRep/STEP solid. STL and Blender sources are retained. The CAD preview uses representative oak color; scene/clutter/teddy_bear.usdc now uses standalone MatteOak (warm tan, roughness0.72, no stretched Rabbit texture), smooth vertex normals and no external material dependencies. Bear and Rabbit alternate every6 simulation seconds.

Shape references/inspiration: https://luciekaas.com/en-us/products/wooden-animals-teddy-teddy-oak and https://us.rosendahl.com/products/kay-bojesen-bear . This is an original sculpt, not a downloaded manufacturer mesh. Regenerate with Blender5.1 background using tools/modeling/teddy_bear_sculpt.py and its NumPy-only sibling helper. Older A/B trials are outside the project dataset.

User accepted the matte offline preview. `TeddyBear_matte_offline_preview.png` was rendered locally in Blender from the updated USD; it is not an Isaac Sim screenshot. Linux was shut down before this final material change, so deploy the committed asset on next startup and verify under the actual factory lighting. The earlier CAD review image is retained as shape history.
