# RBF-PID Calibration and Acceptance

The controller keeps the legacy `systemGain` fields in `bar/(L/min)` units. New
`systemGainKsys` fields use `bar/rpm`; with a valid pump displacement in
`mL/rev`, the process gain is:

```text
K_process = Ksys / (displacement_ml_rev / 1000)
```

Pump nameplate data alone is not a pressure calibration. Without an explicit
legacy `K_process` or `Ksys` plus a positive plant time constant, the runtime
reports `UNCALIBRATED`, suppresses FF/RBF output scheduling, and applies the
conservative PI fallback (`Kp=0.10`, `Ki=0.05`, integral limit `0.10*hardMax`).

Calibrated acceptance uses the physical pressure model or calibrated HIL path:

- fixed control period: `1 ms`;
- real-pressure 90% rise time: `<= 800 ms`, confirmed for 3 consecutive samples;
- real-pressure overshoot: `<= 5%`;
- controller filtered-pressure steady window: last 2 s after settling;
- filtered-pressure peak-to-peak: `<= 1 bar`;
- filtered-pressure absolute steady error: `<= 1 bar`.

Acceptance traces should include calibration status, requested/applied strategy,
`K_process`, `g_du`, effective cap, cap-bound status, adaptation freeze reason,
and real/measured/filtered pressure metrics. Physical-machine commissioning
must repeat the matrix over target steps, K mismatch, sensor noise, pump
saturation, strategy switching, and relief direction before enabling RBF as a
production default.
