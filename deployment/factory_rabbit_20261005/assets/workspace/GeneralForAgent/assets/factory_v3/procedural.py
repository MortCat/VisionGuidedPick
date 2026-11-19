"""Seeded, surface-local dirt and orientation-aware inlet placement."""
import json
import math
import random

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade


class DirtGenerator:
    def __init__(self, source_path, settings):
        self.settings = settings
        asset = Usd.Stage.Open(str(source_path))
        mesh = next(p for p in asset.Traverse() if p.IsA(UsdGeom.Mesh))
        self.mesh_suffix = str(mesh.GetPath())[len(str(asset.GetDefaultPrim().GetPath())):]
        geometry = UsdGeom.Mesh(mesh)
        assert all(c == 3 for c in geometry.GetFaceVertexCountsAttr().Get())
        self.points = np.asarray(geometry.GetPointsAttr().Get(), dtype=float)
        self.triangles = np.asarray(geometry.GetFaceVertexIndicesAttr().Get(), dtype=int).reshape(-1, 3)
        triangle_points = self.points[self.triangles]
        self.centroids = triangle_points.mean(axis=1)
        cross = np.cross(triangle_points[:, 1] - triangle_points[:, 0],
                         triangle_points[:, 2] - triangle_points[:, 0])
        lengths = np.linalg.norm(cross, axis=1)
        self.areas = lengths / 2
        self.normals = cross / np.maximum(lengths[:, None], 1e-15)
        candidates = (self.centroids[:, 2] > .012) & (self.normals[:, 2] > -.65) & (self.areas > 1e-13)
        self.candidates = np.flatnonzero(candidates)
        self.weights = self.areas[candidates] / self.areas[candidates].sum()
        self.total_area = float(self.areas.sum())

    def sample(self, seed):
        rng = np.random.default_rng(seed)
        count = int(rng.integers(self.settings['patch_count_min'], self.settings['patch_count_max'] + 1))
        score = np.full(len(self.triangles), -np.inf)
        patches = []
        for _ in range(count):
            face = int(rng.choice(self.candidates, p=self.weights))
            center, normal = self.centroids[face], self.normals[face]
            reference = np.array([1., 0., 0.]) if abs(normal[2]) > .85 else np.array([0., 0., 1.])
            tangent = np.cross(normal, reference); tangent /= np.linalg.norm(tangent)
            bitangent = np.cross(normal, tangent)
            angle = float(rng.uniform(0, 2 * math.pi))
            u = tangent * math.cos(angle) + bitangent * math.sin(angle)
            v = np.cross(normal, u)
            diameter = float(rng.uniform(*self.settings['diameter_mm'])) / 1000
            radii = np.array([diameter / 2, diameter / 2 * float(rng.uniform(.65, 1.))])
            displacement = self.centroids - center
            uv = np.column_stack((displacement @ u, displacement @ v)) / radii
            phase = float(rng.uniform(0, 2 * math.pi))
            distortion = 1 + .12 * np.sin(uv[:, 0] * 4 + phase) * np.cos(uv[:, 1] * 5 - phase)
            local_score = 1 - (uv ** 2).sum(axis=1) / distortion ** 2
            same_surface = ((self.normals @ normal) > .45) & (np.abs(displacement @ normal) < max(.0012, diameter * .25))
            score = np.maximum(score, np.where(same_surface, local_score, -np.inf))
            patches.append({'center_face': face, 'center_local_m': center.tolist(), 'normal_local': normal.tolist(),
                            'tangent_u': u.tolist(), 'tangent_v': v.tolist(), 'radii_m': radii.tolist(),
                            'diameter_mm': diameter * 1000, 'angle_rad': angle, 'edge_phase': phase})
        selected = np.flatnonzero(score > 0)
        # Enforce actual triangle surface-area coverage, rather than face count.
        ordered = selected[np.argsort(score[selected])[::-1]]
        cumulative = np.cumsum(self.areas[ordered])
        selected = ordered[cumulative <= self.total_area * self.settings['maximum_surface_fraction']]
        assert len(selected) >= 3, 'Generated dirt is too small for this mesh'
        core = selected[score[selected] > .32]
        edge = selected[score[selected] <= .32]
        parameters = {'seed': int(seed), 'coordinate_frame': 'rabbit_mesh_local_meters', 'patches': patches,
                      'surface_fraction': float(self.areas[selected].sum() / self.total_area),
                      'affected_faces': int(len(selected)), 'mesh_face_count': len(self.triangles),
                      'model': 'surface_tangent_ellipses_with_irregular_edges_v1'}
        return parameters, sorted(edge.tolist()), sorted(core.tolist())


def create_dirt_material(stage, path, color, roughness):
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + '/MDL')
    shader.SetSourceAsset(Sdf.AssetPath('OmniPBR.mdl'), 'mdl')
    shader.SetSourceAssetSubIdentifier('OmniPBR', 'mdl')
    shader.CreateInput('diffuse_color_constant', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput('reflection_roughness_constant', Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateOutput('out', Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput('mdl').ConnectToSource(shader.GetOutput('out'))
    preview = UsdShade.Shader.Define(stage, path + '/Preview')
    preview.CreateIdAttr('UsdPreviewSurface')
    preview.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    preview.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(roughness)
    preview.CreateOutput('surface', Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput().ConnectToSource(preview.GetOutput('surface'))
    return material


def apply_dirt(stage, prim_path, generator, seed):
    parameters, edge, core = generator.sample(seed)
    mesh_path = prim_path + generator.mesh_suffix
    for name, indices, color, rough in [('DustEdge', edge, (.20, .12, .055), .88),
                                        ('DirtCore', core, (.052, .032, .015), .96)]:
        material = create_dirt_material(stage, prim_path + '/DefectLooks/' + name, color, rough)
        subset = UsdGeom.Subset.Define(stage, mesh_path + '/' + name)
        subset.CreateElementTypeAttr('face'); subset.CreateFamilyNameAttr('materialBind')
        subset.CreateIndicesAttr(indices)
        UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(material)
        subset.GetPrim().CreateAttribute('inspection:defectType', Sdf.ValueTypeNames.String).Set('surface_contamination')
    prim = stage.GetPrimAtPath(prim_path)
    prim.CreateAttribute('inspection:quality', Sdf.ValueTypeNames.String).Set('defective')
    prim.CreateAttribute('inspection:defectType', Sdf.ValueTypeNames.String).Set('surface_contamination')
    prim.CreateAttribute('inspection:defectParameters', Sdf.ValueTypeNames.String).Set(json.dumps(parameters))
    prim.CreateAttribute('inspection:randomSeed', Sdf.ValueTypeNames.Int64).Set(int(seed))
    return parameters


def oriented_bounds(bounds, yaw_degrees):
    radians = math.radians(yaw_degrees); c, s = math.cos(radians), math.sin(radians)
    corners = [(x * c - y * s, x * s + y * c)
               for x in [bounds[0][0], bounds[1][0]] for y in [bounds[0][1], bounds[1][1]]]
    return [min(p[0] for p in corners), min(p[1] for p in corners)], [max(p[0] for p in corners), max(p[1] for p in corners)]


def sample_inlet(bounds, yaw, belt_bounds, settings, seed):
    rng = random.Random(seed)
    lo, hi = oriented_bounds(bounds, yaw)
    margin = settings['edge_clearance_m']
    left = belt_bounds[0][0] + margin - lo[0]
    right = belt_bounds[1][0] - margin - hi[0]
    physical_center_range = [left, right]
    center_range = settings.get('robot_center_x_range_m')
    footprint_range = settings.get('robot_footprint_x_range_m')
    if center_range is not None:
        assert len(center_range) == 2 and center_range[0] < center_range[1]
        left, right = max(left, center_range[0]), min(right, center_range[1])
    if footprint_range is not None:
        assert len(footprint_range) == 2 and footprint_range[0] < footprint_range[1]
        left = max(left, footprint_range[0] - lo[0])
        right = min(right, footprint_range[1] - hi[0])
    assert right > left, 'Rabbit does not fit the conveyor width with clearance'
    band = (right - left) * settings['edge_band_fraction']
    draw = rng.random(); weight = settings['edge_sampling_probability']
    if draw < weight / 2:
        x, region = rng.uniform(left, left + band), 'left_edge'
    elif draw < weight:
        x, region = rng.uniform(right - band, right), 'right_edge'
    else:
        x = rng.uniform(left, right)
        region = 'robot_work_band' if center_range is not None else 'full_width'
    y = belt_bounds[0][1] + settings['inlet_offset_m']
    assert y + lo[1] > belt_bounds[0][1]
    return [x, y, belt_bounds[1][2] + .005], {
        'seed': int(seed), 'region': region, 'valid_center_x_m': [left, right],
        'physical_valid_center_x_m': physical_center_range,
        'robot_center_x_range_m': center_range,
        'robot_footprint_x_range_m': footprint_range,
        'oriented_footprint_min_m': lo, 'oriented_footprint_max_m': hi,
        'minimum_edge_clearance_m': margin,
        'actual_initial_left_clearance_m': x + lo[0] - belt_bounds[0][0],
        'actual_initial_right_clearance_m': belt_bounds[1][0] - x - hi[0]}
