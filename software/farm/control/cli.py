"""Exercise live Jev decisions against simulated motors. Real use runs in the motor owner."""
import argparse
import json
import sys

from farm.config import load_env
from farm.llm.decisions import DecisionsClient
from .jev import Limits, MotionController, SimActuator, action_catalog


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--catalog", action="store_true")
    ap.add_argument("--simulate", action="store_true", help="live Jev, simulated actuators; never opens hardware")
    ap.add_argument("--goal", default="")
    ap.add_argument("--provider", choices=("typesafe", "openrouter"), default="typesafe")
    ap.add_argument("--input", help="fresh observation JSONL; default stdin")
    args = ap.parse_args(argv)
    if args.catalog:
        print(json.dumps({n: a.description for n, a in action_catalog(Limits()).items()}, indent=2))
        return
    if not args.simulate or not args.goal:
        ap.error("use --simulate --goal; physical execution must be installed inside the existing motor owner")
    load_env()
    source = open(args.input) if args.input else sys.stdin
    try:
        with DecisionsClient(provider=args.provider) as backend:
            controller = MotionController(SimActuator(), backend, args.goal)
            try:
                for line in source:
                    if len(line) > 64_000:
                        raise ValueError("observation exceeds 64000 characters")
                    out = controller.step(json.loads(line))
                    print(json.dumps({**out, "simulated": True}, allow_nan=False), flush=True)
                    if controller.stopped.is_set():
                        break
            finally:
                controller.request_stop()
    finally:
        if source is not sys.stdin:
            source.close()


if __name__ == "__main__":
    main()
