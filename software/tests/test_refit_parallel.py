import json

import numpy as np
import pytest

from tools.fold_demos_to_lerobot import shard_trials, set_render_pose
from tools.train_refit_parallel import merge_shards


def test_four_shards_partition_training_without_holdouts():
    seeds = [s for s in range(10000, 10320) if s % 10]
    shards = [shard_trials(seeds, i, 4) for i in range(4)]
    assert sorted(s for shard in shards for s in shard) == seeds
    assert all(len(set(a) & set(b)) == 0 for i, a in enumerate(shards) for b in shards[i+1:])
    with pytest.raises(ValueError):
        shard_trials(seeds, 4, 4)


def test_fast_pose_matches_full_physics_render_with_moving_camera():
    import mujoco
    m = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <light pos="0 0 3" mode="targetbody" target="arm"/>
      <geom type="plane" size="2 2 .1"/>
      <body name="arm" pos="0 0 .4"><joint type="hinge" axis="0 1 0"/>
       <geom type="box" size=".3 .05 .05" rgba=".3 .7 .9 1"/>
       <camera name="wrist" pos="0 0 1"/></body>
      <camera name="fixed" pos="0 0 2"/></worldbody></mujoco>''')
    a, b = mujoco.MjData(m), mujoco.MjData(m)
    with mujoco.Renderer(m, 64, 64) as r:
        for q in (0., .4, -.5):
            a.qpos[:] = q
            mujoco.mj_forward(m, a)
            set_render_pose(m, b, [q])
            for cam in ('wrist', 'fixed'):
                r.update_scene(a, camera=cam)
                expected = r.render().copy()
                r.update_scene(b, camera=cam)
                np.testing.assert_array_equal(r.render(), expected)


def test_parallel_image_shards_merge_with_correct_actions_images_and_indices(tmp_path):
    from farm.learning.recorder import EpisodeRecorder
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    roots = [tmp_path/f'shard-{i}' for i in range(4)]
    for i, p in enumerate(roots):
        rec = EpisodeRecorder(p,f'local/refit_shard_{i}',10,['front'],(16,16),['joint','joint2'],
                              image_writer_threads=2)
        rec.start_episode('fold')
        for frame in range(3):
            assert rec.tick({'joint':float(i),'joint2':0.},{'joint':float(i+1),'joint2':1.},
                            {'front':np.full((16,16,3),i*40,np.uint8)})
        assert rec.end_episode(True)==3
        rec.close()
        (p/'conversion.json').write_text(json.dumps(dict(task='fold',cameras={'front':'front'},
            joints=['joint','joint2'],fps=10,episodes=[dict(seed=i+1,frames=3,trial=f'trial-{i}')],simulation_only=True)))
        (p/'holdout.json').write_text(json.dumps([dict(seed=10)]))
    result = merge_shards(roots,tmp_path/'merged','local/merged',minimum=4)
    assert result==dict(training_episodes=4,frames=12,holdouts=1)
    ds = LeRobotDataset('local/merged',root=tmp_path/'merged')
    for i in range(4):
        r = ds[3*i]
        assert float(r['observation.state'][0])==i
        assert float(r['action'][0])==i+1
        assert float(r['observation.images.front'].mean())==pytest.approx(i*40/255,abs=1e-6)
    bad = json.loads((roots[0]/'holdout.json').read_text()) + [dict(seed=1)]
    for p in roots:
        (p/'holdout.json').write_text(json.dumps(bad))
    with pytest.raises(RuntimeError,match='leakage'):
        merge_shards(roots,tmp_path/'invalid','local/invalid',minimum=4)
