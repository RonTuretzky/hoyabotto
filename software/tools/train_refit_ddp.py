"""Distributed ACT entrypoint; bf16 autocast, full-precision saved weights."""
import torch
import os
import time
from tools.refit_hub_reporting import DeferredUploads
from pathlib import Path
from lerobot.scripts import lerobot_train
from carton.refit_camera_contract import load_contract, save_contract


def install_camera_checkpoint_hook():
    original = lerobot_train.save_checkpoint
    contract = load_contract()

    def save_with_camera(*args, **kwargs):
        result = original(*args, **kwargs)
        directory = kwargs.get('checkpoint_dir', args[0] if args else None)
        save_contract(Path(directory) / 'pretrained_model', contract)
        return result

    lerobot_train.save_checkpoint = save_with_camera

def install_deferred_checkpoint_uploads():
    original = lerobot_train.push_checkpoint_to_hub
    uploads = DeferredUploads(blocked_until=float(os.environ.get('REFIT_UPLOAD_NOT_BEFORE', '0')))

    def defer(directory, repo_id, **kwargs):
        directory = Path(directory)
        step = int(directory.name)
        # Retain evaluation milestones; coalesce other checkpoints while the Hub is unavailable.
        key = f'milestone-{step}' if step % 5000 == 0 else 'latest-checkpoint'
        uploads.enqueue(key, lambda: original(directory, repo_id, **kwargs))
        uploads.tick()

    lerobot_train.push_checkpoint_to_hub = defer
    original_update = lerobot_train.update_policy
    def update_and_report(*args, **kwargs):
        result = original_update(*args, **kwargs)
        if int(os.environ.get('RANK', '0')) == 0:
            uploads.tick()
        return result
    lerobot_train.update_policy = update_and_report
    return uploads


if __name__ == '__main__':
    torch.set_autocast_dtype('cuda', torch.bfloat16)
    install_camera_checkpoint_hook()
    uploads = install_deferred_checkpoint_uploads()
    lerobot_train.main()
    if int(os.environ.get('RANK', '0')) == 0:
        deadline = float(os.environ.get('REFIT_UPLOAD_DEADLINE', str(time.time()+3600)))
        uploads.flush(deadline)
