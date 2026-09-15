# Walking transport and next-robot wiring — 2026-09-05

The existing CPU and single 1 Mbps servo bus have demonstrated the transport
speed needed for 100 Hz commands. Restore and measure the proven combined
command/snapshot path first. New buses, motors, or wiring are not established
requirements for fixing today's timing regression. For the next robot, prioritize
serviceable wiring and correctly rated power distribution; extra buses are an
optional architectural choice, not the current walking fix.

## What exists and what was measured

The documented as-built path is Linux `/dev/ttyHS1` at 921,600 baud → Uno Q MCU →
D0/D1 and FE-URT adapter → one half-duplex 1 Mbps TTL bus, servo IDs 2–19.
Power already has six leg branches; within each leg the feed enters the hip and
tees to yaw/knee. Logic has a separate battery tap and shares ground. Sources:
`firmware/WIRING.md:27`, `:594`; `linux_control/mcu_feetech_bus.py:55`.

**Historical hardware timing:** the August 19 combined `step_all` (`S`) bench
completed **647/647 transactions at 162 Hz**, about 6 ms each. That report also
notes occasional 800 ms timeouts (~0.2%). After the August 26 cached-snapshot
fix, a read-only `S` test completed **3,000/3,000 requests at 100 Hz**, without
bus errors: **5.8 ms mean, 6.1 ms p99, 6.7 ms maximum**, with position/IMU ages
around 6 ms or less. These demonstrate transport capacity; they are not proof
of sustained smooth walking or independent fresh current/temperature samples
on every tick. Sources: [August 19 run log](../rl_move/RUNLOG.md),
[August 26 timing report](../rl_docs/BUS_AND_TIMING_DEBUG_2026-08-26.md);
commits `4f22bac48`, `bb49b5d90`, `d84a04e95`.

On August 30, `f59125f88` changed drive mode from combined `S` transactions to
separate writes and a 10 Hz snapshot thread; `08891671f` then capped writes at
50 Hz. The firmware's earlier host-byte pumping and cached-`S` fixes remain
present. This history points first to the changed controller/transport path,
not to an inherent servo speed or baud limit.

Saved run `rl_move/hardware_traces/rl_walk_trial_20260904_232355/robot_rl_drive_20260905_062416_summary.json:251`
used 100 Hz policy, 50 Hz writes and 10 Hz asynchronous snapshots. Policy inference
averaged **0.563 ms** (max 1.304); writes averaged **3.986 ms** (max 19.085);
snapshot updates averaged **19.177 ms** (p95 40.610). There were 97 overruns in
192 ticks. This short, older run identifies transport/scheduling as the better
first target than buying a faster computer. It does not measure the current
integrated controller, and the summary's nominal duration is not proof of
continuous wall-clock walking.

The service runs Python under normal Linux scheduling; no real-time priority or
CPU affinity is configured (`linux_control/systemd/hexapod-web.service:14`).
Replacing inference hardware will not remove UART lock waits or MCU acquisition
latency.

## Wire-time budget

Assume 8N1, 10 bits/byte, healthy replies, no return-delay gaps, retries or software
overhead. The manufacturer's SDK uses `8 + N` bytes for a SyncRead request and
`N × (6 + L)` reply bytes for an L-byte register block. SyncWritePosEx writes
seven registers plus ID per servo: `8 + 8N` bytes.
[Feetech packet implementation](https://github.com/ftservo/FTServo_Arduino/blob/main/src/SCS.cpp),
[position-write implementation](https://github.com/ftservo/FTServo_Arduino/blob/main/src/SMS_STS.cpp).

| Independent bus population | Goal write | Position + speed, L=4 | Full state, L=15 | Write + fast read |
|---|---:|---:|---:|---:|
| 18 servos, current bus | 152 B / 1.52 ms | 206 B / 2.06 ms | 404 B / 4.04 ms | 3.58 ms |
| 6 servos, three parallel buses | 56 B / 0.56 ms | 74 B / 0.74 ms | 140 B / 1.40 ms | 1.30 ms |
| 3 servos, six parallel buses | 32 B / 0.32 ms | 41 B / 0.41 ms | 74 B / 0.74 ms | 0.73 ms |

The parallel rows are elapsed wire time only if buses run concurrently. One
blocking loop visiting three UARTs sequentially does not achieve that speedup.
On today's single bus, 100 goal writes + 90 fast scans + 10 full scans per second
occupy about **37.8%** of nominal wire bandwidth. Thus a scheduled 100 Hz outer
joint loop fits the nominal bandwidth, consistent with the historical combined
transaction measurements. This arithmetic excludes software waits and retries;
it does not certify the current controller's deadlines or loaded motor response.

On the separate host link, a combined S command is 113 bytes out + 133 back:
**2.67 ms** wire time at 921,600, **21.35 ms** at the supported legacy 115,200
fallback. A read-only S exchange is 138 bytes / **1.50 ms**; a full F exchange
with default IDs is 244 bytes / **2.65 ms**. Check the actual negotiated host
baud when interpreting runs. Byte layouts: `mcu_feetech_bus.py:65`, `:72`, `:900`.

Feetech specifies **1 Mbps maximum** for C018: increasing the servo baud beyond
that is unsupported. Its rated torque is 10 kg·cm, versus 30 kg·cm stall torque;
stall current is 2.7 A per motor. These are actuator ratings, not evidence that
today's motor torque or speed limits cause the observed timing failures.
[Feetech C018 specifications](https://www.feetechrc.com/525603.html).

## Smallest useful experiments now

1. Compare the existing combined `S` path with the separate write/snapshot path
   using the same transport and measured latency percentiles. Restore one
   command/snapshot transaction per tick when it reproduces the fast behavior;
   retain current fault handling and fresh health checks. If direct `S` also
   stalls, locate that shared transport delay before changing controller rates.
2. Use existing transaction timings and MCU DBG counters to locate time spent
   waiting for a bus lock, first reply byte, payload, or a parked command. The
   current 5 ms serial read timeout is a maximum wait, not an automatic 5 ms tax
   on every read; lowering it without evidence may only add wakeups.
3. Once command timing is restored, compare estimator coefficients with the
   same policy, command, and transport cadence. Measure actual state ages and
   motion quality; avoid mixing a filter change with a different polling scheme.

Preserve full-health acquisition while restoring fast `S` traffic. In the older
firmware, the `HOST_S_CONTROL_IDLE_MS = 30` early return suppresses background full-state
acquisition while S requests keep arriving faster than 30 ms
(`feetech_bridge.ino:150`, pre-fix loop). Commit `d57ead668` changes
the pending snapshot refresh to run a due full-state pass instead of a fast pass,
and lets due health acquisition bypass the quiet-window return. This is a
specific health-scheduling correction, not evidence that the old fast command
path lacked bandwidth. It does not add a physical acquisition
timestamp/sequence to F replies; successive host polls are still not independent
proof of successive physical health acquisitions. The present fast four-byte
position/speed read and slow fifteen-byte health read are already the right
payload separation. Keep current/temperature monitoring when restoring the
existing combined path; a new protocol is not the first remedy.

## Next-robot harness

- **Optional: independently driven buses for fault isolation and extra margin.**
  Three buses of six servos, or one per leg, reduce per-bus wire time only when
  the controller actually serves them concurrently. They add interfaces,
  connectors, and scheduling work. The existing single bus has already met
  100 Hz transport timing; retain it unless measurements or a new requirement
  justify the added hardware. Verify exposed UART pins before selecting a split.
- **Six power branches, each distributing directly to its three servos.** Keep
  the existing separate logic tap and shared reference. Use a branch splice or
  board before the three pigtails so a hip socket does not carry all three
  motors. Three stalled C018s draw 8.1 A; the existing claim that one whole leg
  is always below a 3 A connector rating is not valid in that condition. This
  sum of stall ratings is a sizing case, not a measured walking branch current
  or a demonstrated cause of today's UART delay.
  Molex specifies 3.0 A with AWG22 and 2.5 A with AWG24 for the cited Mini-SPOX
  family; exact clone terminals/crimps may differ.
  [Molex specification, section 4.2](https://www.molex.com/content/dam/molex/molex-dot-com/products/automated/en-us/productspecificationpdf/526/5264/52641001-PS-000.pdf).
- **Separate short signal/reference pairs from high-current motor paths.** Use
  labelled, strain-relieved connectors and service loops at joints. Keep the
  existing motor-rail disconnect and make leg branches individually removable.
  Preserve signal ground; splitting data buses does not imply isolated grounds.
- **Measure voltage at the far servo under load and trunk/branch current.** Size
  conductors, connectors and branch protection from those measurements and their
  ratings. The current 20 A main-fuse and 9–13 A walking budget originate in the
  July simulated current model (`firmware/WIRING.md:684`, `:728`), not a measured
  qualification of sustained walking. A larger fuse is not a cure for voltage
  drop or an overloaded pigtail.

These harness recommendations concern serviceability and electrical margin.
They do not establish a need to rebuild the robot before restoring smooth walks.
