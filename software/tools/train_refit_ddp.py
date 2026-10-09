"""Four-process ACT entrypoint; bf16 autocast, full-precision saved weights."""
import torch
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

if __name__ == '__main__':
    torch.set_autocast_dtype('cuda', torch.bfloat16)
    install_camera_checkpoint_hook()
    lerobot_train.main()
