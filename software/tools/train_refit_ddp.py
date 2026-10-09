"""Four-process ACT entrypoint; bf16 autocast, full-precision saved weights."""
import torch
from lerobot.scripts.lerobot_train import main

if __name__ == '__main__':
    torch.set_autocast_dtype('cuda', torch.bfloat16)
    main()
