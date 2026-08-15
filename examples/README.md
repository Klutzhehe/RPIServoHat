# Raspberry Pi examples

Run these from the repository root after installing `requirements.txt`.

```sh
python3 examples/read_telemetry.py
python3 examples/set_pose.py
python3 examples/single_servo_sweep.py 0
```

`set_pose.py` is deliberately a small editable dictionary. Start with one
servo, use conservative values, and only add positions confirmed safe for your
mechanism. `single_servo_sweep.py` defaults to the restrained 1400-1600 µs
range; pass `--minimum` and `--maximum` only after verifying your hardware.
