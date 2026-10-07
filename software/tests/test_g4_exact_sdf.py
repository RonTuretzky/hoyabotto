"""Exact source-triangle SDF plugin: build, analytic controls, fail-closed source checks.

These verify plugin behaviour only. They give no contact-model, trajectory or
assembly credit.
"""
import shutil
import tempfile
import unittest
import xml.etree.ElementTree as E
from pathlib import Path

import mujoco
import numpy as np
import trimesh

from planter import g4_exact_sdf as X

HAVE_COMPILER = shutil.which('clang++') or shutil.which('g++')
EXTENTS = np.array([.020, .012, .008])


def box_truth(point):
    q = np.abs(point)-EXTENTS/2
    return float(np.linalg.norm(np.maximum(q, 0))+min(float(q.max()), 0))


def write_scene(path, stl, *, file_attribute, pos='.1 -.2 .3', euler='.2 -.3 .4', instance='box_exact'):
    root = E.Element('mujoco')
    ext = E.SubElement(root, 'extension')
    plugin = E.SubElement(ext, 'plugin', plugin=X.PLUGIN_NAME)
    inst = E.SubElement(plugin, 'instance', name=instance)
    if file_attribute is not None:
        E.SubElement(inst, 'config', key='file', value=str(file_attribute))
    asset = E.SubElement(root, 'asset')
    E.SubElement(asset, 'mesh', name='source', file=str(stl))
    body = E.SubElement(E.SubElement(root, 'worldbody'), 'body', name='source_box', pos=pos, euler=euler)
    E.SubElement(body, 'freejoint')
    E.SubElement(body, 'inertial', pos='0 0 0', mass='.02', diaginertia='.000001 .000001 .000001')
    geom = E.SubElement(body, 'geom', name='source_box', type='sdf', mesh='source')
    E.SubElement(geom, 'plugin', instance=instance)
    E.ElementTree(root).write(path, encoding='unicode')
    return path


@unittest.skipUnless(HAVE_COMPILER, 'no C++ compiler on PATH')
class ExactSdfPluginTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = Path(tempfile.mkdtemp())
        cls.manifest = X.build(cls.tmp)
        cls.handle = X.load(cls.manifest['library'])
        cls.box = cls.tmp/'box.stl'
        trimesh.creation.box(extents=EXTENTS).export(cls.box)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def _model(self, **kw):
        scene = write_scene(self.tmp/f'scene-{len(list(self.tmp.iterdir()))}.xml', self.box, **kw)
        model = mujoco.MjModel.from_xml_path(str(scene))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        return model, data

    def test_manifest_records_hashes(self):
        for key in ('source_sha256', 'library_sha256', 'mujoco_library_sha256', 'command'):
            self.assertTrue(self.manifest[key])
        self.assertEqual(self.manifest['plugin_name'], X.PLUGIN_NAME)

    def test_source_double_mode_matches_analytic_box(self):
        model, data = self._model(file_attribute=self.box)
        info = X.instance_info(self.handle, model, data, 'source_box')
        self.assertEqual(info['mode'], 'source_double')
        self.assertLess(info['compiled_deviation_m'], X.COMPILED_DEVIATION_LIMIT_M)
        self.assertEqual((info['vertices'], info['faces']), (8, 12))
        query = X.PluginQuery(model, data, 'source_box')
        rng = np.random.default_rng(84216)
        worst, grad_worst, wrong = 0., 0., 0
        for point in rng.uniform(-.02, .02, (300, 3)):
            value, gradient = query.body(point)
            truth = box_truth(point)
            worst = max(worst, abs(value-truth))
            wrong += truth*value < 0 and abs(truth) > 1e-8
            eps = 1e-7
            finite = np.array([(query.body(point+np.eye(3)[k]*eps)[0]-query.body(point-np.eye(3)[k]*eps)[0])/(2*eps) for k in range(3)])
            if np.linalg.norm(finite-gradient) < .5:  # away from non-differentiable loci
                grad_worst = max(grad_worst, float(np.linalg.norm(finite-gradient)))
        # trimesh writes float32 STL, so the exact box is itself float32-rounded: allow 10 nm.
        self.assertLess(worst, 1e-8)
        self.assertEqual(wrong, 0)
        self.assertLess(grad_worst, 1e-5)
        self.assertAlmostEqual(query.body([0, 0, 0])[0], -.004, delta=1e-8)

    def test_compiled_mode_without_file(self):
        model, data = self._model(file_attribute=None)
        info = X.instance_info(self.handle, model, data, 'source_box')
        self.assertEqual(info['mode'], 'compiled_float32')
        query = X.PluginQuery(model, data, 'source_box')
        self.assertAlmostEqual(query.body([.012, .008, .006])[0], box_truth(np.array([.012, .008, .006])), delta=1e-7)

    def test_world_and_body_queries_agree_under_rigid_motion(self):
        model, data = self._model(file_attribute=self.box)
        query = X.PluginQuery(model, data, 'source_box')
        point = np.array([.007, -.004, .0031])
        before = query.body(point)
        data.qpos[:3] = [-.03, .02, .11]
        quat = np.array([.81, .12, -.31, .47]); data.qpos[3:7] = quat/np.linalg.norm(quat)
        mujoco.mj_forward(model, data)
        after = query.body(point)
        self.assertAlmostEqual(before[0], after[0], places=12)
        np.testing.assert_allclose(before[1], after[1], atol=1e-9)
        bid = int(model.geom_bodyid[query.g])
        world = data.xmat[bid].reshape(3, 3)@point+data.xpos[bid]
        value, gradient = query.world(world)
        self.assertAlmostEqual(value, after[0], places=12)
        np.testing.assert_allclose(data.xmat[bid].reshape(3, 3)@after[1], gradient, atol=1e-9)

    def test_mismatched_source_file_fails_closed(self):
        other = self.tmp/'other.stl'
        trimesh.creation.box(extents=EXTENTS*1.001).export(other)
        scene = write_scene(self.tmp/'scene-mismatch.xml', self.box, file_attribute=other)
        with self.assertRaises(ValueError):
            mujoco.MjModel.from_xml_path(str(scene))

    def test_open_mesh_fails_closed(self):
        open_mesh = trimesh.creation.box(extents=EXTENTS)
        open_mesh.update_faces(np.arange(len(open_mesh.faces)) != 0)
        path = self.tmp/'open.stl'
        open_mesh.export(path)
        scene = write_scene(self.tmp/'scene-open.xml', path, file_attribute=path)
        with self.assertRaises(ValueError):
            mujoco.MjModel.from_xml_path(str(scene))

    def _plate_model(self, initpoints=40):
        """Fine plate (3,072 faces) pressed, tilted, into the top face of the exact box."""
        plate = self.tmp/'plate.stl'
        if not plate.exists():
            trimesh.creation.box(extents=[.016, .010, .002]).subdivide().subdivide().subdivide().subdivide().export(plate)
        root = E.Element('mujoco')
        E.SubElement(root, 'option', sdf_initpoints=str(initpoints))
        inst = E.SubElement(E.SubElement(E.SubElement(root, 'extension'), 'plugin', plugin=X.PLUGIN_NAME), 'instance', name='box')
        E.SubElement(inst, 'config', key='file', value=str(self.box))
        asset = E.SubElement(root, 'asset')
        E.SubElement(asset, 'mesh', name='source', file=str(self.box))
        E.SubElement(asset, 'mesh', name='plate', file=str(plate))
        world = E.SubElement(root, 'worldbody')
        E.SubElement(E.SubElement(world, 'geom', name='box', type='sdf', mesh='source'), 'plugin', instance='box')
        body = E.SubElement(world, 'body', name='plate', pos='0 0 .0049', euler='0 1.2 0')
        E.SubElement(body, 'freejoint')
        E.SubElement(body, 'geom', name='plate', type='mesh', mesh='plate', mass='.01')
        scene = self.tmp/f'plate-{initpoints}.xml'
        E.ElementTree(root).write(scene, encoding='unicode')
        model = mujoco.MjModel.from_xml_path(str(scene))
        return model, mujoco.MjData(model)

    def _plate_contacts(self, model, data):
        mujoco.mj_forward(model, data)
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'plate')
        mid = int(model.geom_dataid[gid])
        vertices = model.mesh_vert[model.mesh_vertadr[mid]:model.mesh_vertadr[mid]+model.mesh_vertnum[mid]].astype(float)
        rotation = data.geom_xmat[gid].reshape(3, 3)
        world = [rotation@v+data.geom_xpos[gid] for v in vertices]
        # The field is linear (z - top) where the plate penetrates, so the deepest point is a vertex.
        field = X.PluginQuery(model, data, 'box')
        deepest = min(field.world(v)[0] for v in world)
        self.assertAlmostEqual(deepest, min(box_truth(v) for v in world), delta=1e-9)  # float32 STL rounding
        return [data.contact[i] for i in range(data.ncon)], deepest

    def test_exact_collider_reports_deepest_point_on_many_face_contact(self):
        for initpoints in (40, 100):
            with self.subTest(sdf_initpoints=initpoints):
                model, data = self._plate_model(initpoints)
                try:
                    X.set_collider(self.handle, True)
                    contacts, deepest = self._plate_contacts(model, data)
                finally:
                    X.set_collider(self.handle, False)
                self.assertLess(deepest, -2.5e-4)
                self.assertTrue(0 < len(contacts) <= min(initpoints, 50))
                self.assertAlmostEqual(min(c.dist for c in contacts), deepest, delta=1e-12)
                field = X.PluginQuery(model, data, 'box')
                for c in contacts:
                    normal = c.frame[:3]
                    np.testing.assert_allclose(normal, [0, 0, -1], atol=1e-9)  # from the plate into the box
                    witness = c.pos-.5*c.dist*normal  # MuJoCo's mesh/SDF convention: halfway along the normal
                    self.assertAlmostEqual(field.world(witness)[0], c.dist, delta=1e-12)
                positions = np.array([c.pos for c in contacts])
                self.assertEqual(len(np.unique(positions.round(12), axis=0)), len(contacts))

    @unittest.skipUnless(mujoco.__version__.startswith('3.14.'), 'records MuJoCo 3.14 mjc_MeshSDF behaviour')
    def test_stock_collider_truncation_is_reproduced(self):
        # The stock candidate buffer fills from early faces and misses the deepest one.
        model, data = self._plate_model(100)
        contacts, deepest = self._plate_contacts(model, data)
        self.assertGreater(min(c.dist for c in contacts), deepest+1e-4)

    def test_exact_collider_respects_small_contact_budget(self):
        model, data = self._plate_model(initpoints=3)
        try:
            X.set_collider(self.handle, True)
            contacts, deepest = self._plate_contacts(model, data)
        finally:
            X.set_collider(self.handle, False)
        self.assertEqual(len(contacts), 3)
        self.assertAlmostEqual(min(c.dist for c in contacts), deepest, delta=1e-10)

    def test_invalid_starts_fails_closed(self):
        for bad in ('0', '-3', '2.5', 'many'):
            scene = write_scene(self.tmp/f'scene-starts-{bad}.xml', self.box, file_attribute=self.box)
            tree = E.parse(scene)
            E.SubElement(tree.getroot().find('./extension/plugin/instance'), 'config', key='starts', value=bad)
            tree.write(scene, encoding='unicode')
            with self.subTest(starts=bad), self.assertRaises(ValueError):
                mujoco.MjModel.from_xml_path(str(scene))

    def test_collider_switch_restores_stock(self):
        model, data = self._plate_model()
        self.assertTrue(X.set_collider(self.handle, True))
        self.assertFalse(X.set_collider(self.handle, False))
        contacts, deepest = self._plate_contacts(model, data)
        self.assertTrue(contacts)  # stock collider still runs; its depth is not scored here

    def test_reference_field_matches_box(self):
        triangles = X.stl_triangles(self.box)
        for point in ([.0, .0, .0], [.012, .008, .006], [.009, .0, .0], [.0, .0, .0041]):
            ref = X.reference_field(triangles, point)
            self.assertAlmostEqual(ref['signed_distance'], box_truth(np.array(point)), places=9)
        self.assertEqual(abs(round(X.winding_number(triangles, [0, 0, 0]))), 1)
        self.assertEqual(round(X.winding_number(triangles, [.1, 0, 0])), 0)


if __name__ == '__main__':
    unittest.main()
