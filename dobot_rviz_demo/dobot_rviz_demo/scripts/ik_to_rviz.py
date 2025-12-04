#!/usr/bin/env python3
import argparse, os, sys, subprocess
sys.path.append(os.path.dirname(__file__))
from fk_ik_sympy import ik_numeric

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, required=True)
    ap.add_argument("--y", type=float, required=True)
    ap.add_argument("--z", type=float, required=True)
    ap.add_argument("--elbow", choices=["up","down"], default="up")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--topic", default="/joint_states")
    ap.add_argument("--urdf", default=None)
    a = ap.parse_args()

    sol = ik_numeric(a.x, a.y, a.z, elbow=a.elbow)
    if sol is None:
        print("Unreachable IK target.", file=sys.stderr); sys.exit(2)
    th = [f"{v:.3f}" for v in sol]
    print("IK (deg):", th)

    cmd = [sys.executable, os.path.join(os.path.dirname(__file__), "fk_to_rviz.py"),
           "--topic", a.topic, "--seconds", str(a.seconds), "--thetas", *th]
    if a.urdf: cmd += ["--urdf", a.urdf]
    sys.exit(subprocess.call(cmd))

if __name__ == "__main__":
    main()
