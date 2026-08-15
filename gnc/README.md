# Autonomous Inspection GNC Engine

Single-file guidance, navigation and control engine for a non-cooperative
multi-vehicle inspection framework.

**Everything is in `inspection_gnc.py` (3,732 lines).** No other project files
are needed. No ROS.

```
Arducam GS MIPI camera  ->  latency-compensated EKF  ->  3-D True ProNav
                                                              |
                        Pixhawk / Cube Orange  <-  MAVLink velocity setpoints
                                    ^
                        independent safety supervisor (revokes authority)
```

---

## Install

Target: NVIDIA Jetson Orin Nano, Ubuntu 22.04 LTS, JetPack 6.x.

```bash
sudo apt update
sudo apt install -y python3-pip python3-numpy python3-opencv
pip3 install pymavlink pyserial
sudo usermod -aG dialout $USER      # then log out and back in
```

Two things people miss:

**`pyserial` is required and `pymavlink` does not depend on it.** Without it,
opening `/dev/ttyTHS1` fails with `ModuleNotFoundError: No module named
'serial'` — not a device error, so it reads like a wiring fault.

**OpenCV must have GStreamer** or the MIPI pipeline cannot open:

```bash
python3 -c "import cv2; print([l for l in cv2.getBuildInformation().split(chr(10)) if 'GStreamer' in l])"
```

Must print `YES`. Ubuntu's `python3-opencv` normally does. A
`pip install opencv-python` wheel does **not** — don't let pip replace it.

---

## Run

```bash
# Bench, no hardware at all (synthetic camera + local MAVLink)
python3 inspection_gnc.py --backend synthetic --device udpout:127.0.0.1:14550

# Against ArduPilot SITL
python3 inspection_gnc.py --backend synthetic --device tcp:127.0.0.1:5760

# Flight hardware
python3 inspection_gnc.py --backend argus --device /dev/ttyTHS1 --baud 921600
```

`--backend v4l2` for Arducam's raw mono GS drivers (OV9281/OV2311), which
bypass Argus. `--help` lists every option.

---

## How it works

### Threads

| Thread | Rate | Job |
|---|---|---|
| capture | 60 Hz | `cap.read()`, stamp, publish to a one-deep slot |
| detector | 60 Hz | ego-motion differencing → polar measurement → queue |
| control | 60 Hz | fuse → predict → ProNav → MAVLink setpoint |
| supervisor | 200 Hz | independent watchdog, authority + handover |
| MAVLink rx/tx | — | heartbeat out, vehicle state in |

Both hand-offs between stages are **non-blocking and lossy on purpose**. A
frame that can't be consumed in time is worthless to an estimator; queueing it
just grows a latency tail. Measured: the camera stalled 235 ms and the control
loop's worst cycle was 0.65 ms.

What is *not* lost is time. Every frame is stamped at grab and that stamp rides
into the measurement, so a detector that occasionally takes 40 ms produces a
correctly back-dated observation, not a mis-timed one.

### Estimator

6-state `[x, vx, y, vy, z, vz]` in BODY FRD. Two things make it an EKF rather
than a linear KF:

- **Polar measurement model** `[azimuth, elevation, range]` with an analytic
  Jacobian. A camera's bearings are sub-milliradian while range error grows as
  distance squared; a Cartesian update would average that anisotropy into an
  isotropic blob.
- **Rotating-frame propagation** carrying `-ω×r` and `-ω×v` from the autopilot
  ATTITUDE stream, so the track doesn't smear when the airframe manoeuvres.

**Latency compensation** handles the fixed 2-frame (33.3 ms) pipeline lag by
rolling a ring buffer back to the epoch bracketing the true capture time,
injecting the delayed sample there, and replaying forward. Measured at 6 m/s
closure: naive fusion leaves a +0.193 m along-track bias; compensated is
−0.007 m, matching a hypothetical zero-latency sensor to 5e-05 m.

Joseph-form update, chi-square innovation gating, and a gate that force-opens
after a rejection streak so a diverged filter can't lock out valid data
forever.

### Guidance

True Proportional Navigation, `a = N · Vc · (ω × r̂)` with N = 4.0.

Note the ordering — `ω × r̂`, not `ω × v`. The literature form `N(ω × V_r)`
defines `V_r` as pursuer-relative-to-target; this filter stores the opposite
convention, so using `v` directly would invert the lateral command (steering
*away* from the target's drift) and leave a spurious radial term.

Saturation is direction-preserving: 6 G (58.8399 m/s²) and 120 mph
(53.6448 m/s). Per-axis clipping would rotate the command away from the ProNav
solution exactly when the manoeuvre is most demanding.

A separate LOS-axial channel holds the standoff range (`--standoff`, default
3 m).

### Camera front-end

Ego-motion-compensated differencing: sparse LK flow + RANSAC partial affine
warps a reference frame onto the current one before differencing, so a moving
airframe doesn't light up every edge in the scene.

The reference is **4 frames back**, not the previous frame. At 60 Hz
consecutive frames are nearly identical and a slow target's per-frame
displacement sits under the noise floor. This one change took detection from
~8% to 91%.

Motion **locates** the target but does not **size** it. The residual of a
displaced object is a crescent whose bounding box is inflated by the
displacement — measured at 1.4× the true silhouette, a −30% range bias. Extent
is therefore re-derived by Otsu segmentation of the current frame at full
resolution. Bias drops to −2.1%.

### Safety supervisor

Independent 200 Hz thread with its own clock. **Push-fed**: components call
`note_*()` and the watchdog only reads its own timestamps, so it can never
block on a component's lock, and it trips correctly when a feeder dies.

| Fault | Threshold | Source |
|---|---|---|
| `CAMERA_STALLED` | 100 ms | capture thread — frames from the sensor |
| `TARGET_LOST` | 500 ms | frames arriving but no usable measurement |
| `CONTROL_LOOP_STALLED` | 150 ms | control cycle liveness |
| `MAVLINK_HEARTBEAT_LOST` | 3.0 s | autopilot HEARTBEAT age |
| `FILTER_DIVERGED` | σ_pos > 12 m, or non-PD | Cholesky on the covariance |
| `FILTER_NOT_FINITE` | NaN/Inf | state or covariance |

Camera and target liveness are deliberately **separate**. Feeding the 100 ms
budget from measurement production instead of frame arrival conflates a dead
camera with a lost target and nuisance-trips on every detector p99 spike —
measured 6 such spikes per 10 s under load.

On trip: revoke authority first, send a zero-velocity hold, then walk a
handover ladder (`LOITER → BRAKE → RTL → ALT_HOLD`), **verifying each by
reading the flight mode back from HEARTBEAT**. Position-dependent modes are
skipped without a 3-D GPS fix rather than burning the confirm timeout on a
refusal. Latched: recovery is a deliberate human action.

**If the link itself is the fault there is nothing to command**, so the
supervisor deliberately stops transmitting and lets ArduPilot's GUIDED timeout
fire. Verified: zero mode commands attempted on a dead link.

### Authority

Two independent conditions must both hold before any setpoint is sent:

1. the supervisor has not tripped, **and**
2. the autopilot is in GUIDED.

The second is what stops this process fighting a pilot. Flip the mode switch
and ArduPilot leaves GUIDED; the loop stops transmitting on the next cycle. No
handshake, no negotiation, no race.

---

## Verified

175 automated checks. Live run against a real MAVLink peer:

```
Autopilot online: system 1, component 1
capture 60.0 Hz | detect 4-5 ms | hit rate 97% | end-to-end 38 ms
cycles 964, overruns 0, worst cycle 2.94 ms
setpoint: frame=8 (BODY_NED) mask=1479 v=(+49.48,-2.81,-3.35)
-- peer goes silent --
SAFETY TRIP [mavlink_heartbeat_lost] -- guidance authority revoked
```

Setpoints froze the instant authority was revoked. Real-wire round-trip also
confirmed `mask=1031` with acceleration populated and NaN/Inf scrubbed to zero
before transmission.

---

## Before you fly

Props off for the first hardware run, then:

1. **Confirm `GUID_TIMEOUT`, RC failsafe and battery failsafe on the airframe.**
   The dead-link path depends entirely on them.
2. **Measure detector p99 on your Orin.** Published numbers come from a 4-core
   container where the synthetic source burns 25% of a core. If p99 exceeds
   ~400 ms, raise `max_measurement_gap_s`.
3. **Grant `CAP_SYS_NICE`** so the capture and supervisor threads get
   `SCHED_FIFO`. Without it they log a warning and run `SCHED_OTHER` — a
   watchdog that can be starved by the workload it watches is a weaker
   watchdog.
4. **Calibrate `extent_bias_px`** against a known target at surveyed ranges.
   The 0.5 px default comes from pixel-centre geometry; real blur and
   thresholding differ, and a *bias* is the one error an EKF cannot average
   away.
5. **Leave `--send-acceleration` off** until you've confirmed your Copter
   firmware honours acceleration in `SET_POSITION_TARGET_LOCAL_NED`. Older
   versions ignore those fields silently — no error, no benefit.

---

## Known limits

**Monocular range is size-scale range.** `r = f·W_assumed / w_pixels`. If your
assumed target width is 20% wrong, every range is 20% wrong — a bias, not
noise. The size-prior uncertainty is folded into the range covariance
(`σ_r ⊃ r·σ_W/W`) so the filter de-weights range and leans on bearings instead
of trusting a confidently wrong number. At 50 m with a ±0.1 m prior on a 0.5 m
target, that term alone is 10 m of the 13.5 m total σ. For unknown-size
targets this is the binding constraint; stereo or a ranging sensor is the only
real fix. Tune with `--target-width` / `--target-width-sigma`.

**This is a companion-computer process on a general-purpose kernel.** It cannot
preempt a kernel stall, an OOM kill, or power loss. It is a layer *above* the
autopilot's failsafes, never a replacement for them.

**Velocity saturating at 53.6 m/s on the synthetic backend is a fixture
artifact**, not expected inspection behaviour. The synthetic camera is a movie
that doesn't react to velocity commands, so the ProNav integrator winds up
against a LOS rate that never responds. With a real vehicle the loop closes.

---

## History

The modular four-file version, including ROS 2 node wrappers, is in git
history:

```bash
git show 018f3d1:gnc/relative_nav_pronav.py > relative_nav_pronav.py
git show 018f3d1:gnc/camera_frontend.py     > camera_frontend.py
git show 018f3d1:gnc/safety_supervisor.py   > safety_supervisor.py
git show 018f3d1:gnc/inspection_engine.py   > inspection_engine.py
```
