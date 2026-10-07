"""Fail-closed bundle contracts, separate from simulated/physical task success."""
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
import trimesh

from planter.g4_carrier import solid, sha
from planter.g4_collision_assets import (audit_union, inertial_properties, load_bundle, SCHEMA,
                                         precise_partition, _inside_solid_angle)


class CollisionAssetsTest(unittest.TestCase):
    def test_union_rejects_filled_hollow(self):
        outer = solid(trimesh.creation.box([10, 10, 10]))
        inner = solid(trimesh.creation.box([8, 8, 12]))
        _, audit = audit_union(outer-inner, [trimesh.creation.box([10, 10, 10])])
        self.assertFalse(audit['passed'])
        self.assertGreater(audit['extra_mm3'], 600)

    def test_empty_union_rejected(self):
        with self.assertRaises(ValueError):
            audit_union(solid(trimesh.creation.box()), [])

    def test_precise_partition_preserves_translated_thin_l_shape(self):
        a = solid(trimesh.creation.box([8, .4, 2])).translate([70, 0, 0])
        b = solid(trimesh.creation.box([.4, 8, 2])).translate([70, 0, 0])
        source = a+b
        pieces, stats = precise_partition([source])
        _, audit = audit_union(source, pieces)
        self.assertTrue(audit['passed'])
        self.assertLess(audit['extra_mm3'], .0001)
        self.assertLess(stats['summed_hull_extra_mm3'], .0001)

    def test_source_membership_is_json_serializable(self):
        mesh = trimesh.creation.box([2, 2, 2])
        self.assertTrue(_inside_solid_angle(mesh, [0, 0, 0]))
        self.assertFalse(_inside_solid_angle(mesh, [2, 0, 0]))
        self.assertEqual(json.dumps(_inside_solid_angle(mesh, [0, 0, 0])), 'true')

    def test_mass_units_and_inertia(self):
        result = inertial_properties(trimesh.creation.box([100, 100, 100]), .1)
        np.testing.assert_allclose(np.diag(result['inertia_kg_m2']), [.1*.1**2/6]*3)
        with self.assertRaises(ValueError):
            inertial_properties(trimesh.creation.box(), float('nan'))

    def test_incomplete_bundle_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory)/'manifest.json'
            p.write_text(json.dumps(dict(schema=SCHEMA, status='building')))
            with self.assertRaises(ValueError):
                load_bundle(p)

    def test_hashes_and_path_containment(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            asset = root/'part.obj'
            asset.write_text('source')
            part = dict(visual_stl='part.obj', collision_meshes=['part.obj'], sha256={'part.obj': sha(asset)},
                        source_to_assembly=np.eye(4).tolist(), audit={'passed': True})
            manifest = dict(schema=SCHEMA, status='complete', functional_probes=[dict(passed=True)],
                            parts={n: part.copy() for n in ('trough', 'holder', 'guide', 'carrier')})
            p = root/'manifest.json'
            p.write_text(json.dumps(manifest))
            self.assertTrue(Path(load_bundle(p)['parts']['guide']['visual_stl']).is_absolute())
            asset.write_text('tampered')
            with self.assertRaises(ValueError):
                load_bundle(p)
            manifest['parts']['guide']['visual_stl'] = '../outside.obj'
            p.write_text(json.dumps(manifest))
            with self.assertRaises(ValueError):
                load_bundle(p, verify_hashes=False)


if __name__ == '__main__':
    unittest.main()
