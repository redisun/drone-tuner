# drone-tuner

A Rust CLI that reads a Betaflight blackbox log, analyses it for PID/filter
problems, and writes safer values back to the flight controller over MSP.

It's been validated end-to-end on two real quads (Jeno STM32H743, TBS
Source One STM32F7x2): pull a log → analyse → apply recommended PIDs
→ persist to EEPROM. Both quads survived multiple tune iterations and
power cycles.

## Status

| Capability | State |
|---|---|
| Blackbox parser (Betaflight BBL) | Working |
| FFT + oscillation detection | Working |
| PID and filter recommendations | Working, calibrated against real flights |
| Step-response analysis | Working |
| MSP read/write (PID, filter config, EEPROM) | Working on hardware (Jeno H743, TBS Source One F7) |
| Filter writeback via MSP2_COMMON_SET_SETTING | Implemented + simulator-tested; real-hardware verification pending |
| Onboard dataflash pull (`--pull-bbl`) | Implemented + simulator-tested; real-hardware verification pending |
| Desktop GUI (Tauri) | Not started |
| ML pattern recognition | Not started |
| Tune marketplace | Not started |

`docs/PROJECT_ASSESSMENT.md` keeps the prioritised roadmap.

## How it works, step by step

The interesting command is `tune`. Here's what it does, in order, when
you run it against a real FC.

### 1. Resolve the blackbox file

Two sources are supported:

- **Local file**: `drone-tuner tune path/to/flight.bbl --connection /dev/ttyACM0`
- **Pull from FC**: `drone-tuner tune --pull-bbl --connection /dev/ttyACM0`

With `--pull-bbl`, the tool issues `MSP_DATAFLASH_SUMMARY` (cmd 70) to
get the chip's used/total bytes, then loops `MSP_DATAFLASH_READ` (cmd 71)
until the whole blob is in memory. A live progress bar shows
bytes/sec and ETA. The pulled bytes are written to a tempdir
(`/tmp/drone-tuner-pull-<ts>.bbl`) — pass `--keep-bbl <PATH>` to land
them somewhere persistent.

V1 framing caps each chunk at ~240 bytes, so a 4 MB log is ~17 000
roundtrips. On real serial at 115 200 baud this is the slow part of the
pipeline (tens of seconds).

### 2. Parse

`SimpleBlackboxParser` walks the BBL header, picks the requested session
(default: most recent; override with `--session N` or
`--session-strategy {first,last,longest}`), and decodes the I/P frame
stream into a `FlightSession`. Sample rate is derived from the looptime
header.

A spinner shows "parsing …" and the final line reports `parsed N samples
(M ms)`.

### 3. Analyse

`AnalysisEngine::analyze` runs three things in parallel-conceptually:

1. **FFT (Welch's method)** on each gyro axis, 1024-sample windows with
   50% overlap, hanning-windowed. Peaks are extracted from the resulting
   PSD.
2. **Oscillation classification.** Each peak is bucketed by frequency
   band: < ~30 Hz is treated as flight dynamics, 30–80 Hz as P-term,
   80–250 Hz as D-term, 250–500 Hz as mechanical/motor resonance.
3. **Step-response detector** scans `rcCommand` for sticks crossing 30%
   of full deflection within 50 ms. For each step it captures the
   pre/post window, computes rise time, overshoot, settling time, and
   steady-state error in deg/s.

The output is an `AnalysisReport` with PID and filter recommendations,
each tagged with a priority (`Critical`/`High`/`Medium`/`Low`) and a
plain-language reason. PID recs are derived from the step-response data
when available, falling back to oscillation heuristics otherwise.

### 4. Display recommendations

The report is printed regardless of whether you intend to apply
anything. Each rec has an icon, a "current → recommended" line, and a
reason:

```
🎛️ PID Adjustments:
    🔴 Yaw P: 76.0 → 53.2
      Reason: Reduce P-term to decrease overshoot from 32.5% to target 15.0%

🔧 Filter Adjustments:
    🟡 Dynamic Notch: 1 notches, 16-500 Hz (Q: 10)
      Expected 15.0% reduction in frame resonance by expanding dynamic notch range
```

### 5. Apply (only with explicit opt-in)

The default is **read-only**. To actually write to the FC you have to
pass one of:

- `--auto-apply-safe` — only applies `Low`/`Medium` priority recs.
- `--apply-all` — applies every rec (including `Critical`/`High`).

The apply phase:

1. Opens the FC connection (USB serial via `serialport`, or the
   in-process `simulator://` scheme for dry runs).
2. Reads the current full PID payload via `MSP_PID` (cmd 112) — all 30
   bytes. We round-trip the entire payload so axes the tool doesn't
   touch (LEVEL/MAG/NAV/etc.) come back unchanged.
3. Computes the new payload by mutating only the recommended (axis,
   term) tuples.
4. Calls `apply_pid_with_rollback`, which is **read → write → on
   write-or-ack failure, restore the backup**. The pre-change snapshot
   is also returned to the caller for forensics.
5. With `--backup <PATH>`, dumps that pre-change snapshot to JSON.
6. With `--save-eeprom`, sends `MSP_EEPROM_WRITE` (cmd 250) so the
   change survives a power cycle. Without it, RAM-only changes will
   revert on next reboot — itself a useful safety net.
7. Appends one row to `~/.local/share/drone-tuner/history.jsonl` (or
   `$XDG_DATA_HOME/drone-tuner/history.jsonl`) recording the FC
   identity, the BBL fingerprint, the before/after PIDs, and whether
   the change was persisted.

Each step gets a section banner and a result line on stdout, so you can
see exactly where you are even when run under CI or with output piped
to a file:

```
✏️ ── Apply ──
  ✓ connected: BTFL 4.5.1 (api 1.46.0, target STM32H743)
  ✓ current: roll=(42, 85, 35) pitch=(46, 90, 38) yaw=(45, 90, 0)
  📝 1 PID change(s) staged:
    Yaw   P/I/D: (45, 90, 0) → (53, 90, 0)
  ✓ PIDs written; backup retained in memory
  💾 backup → tune-backup-20260426-130045.json
  ✓ changes persisted across power cycles
📒 Tune logged to ~/.local/share/drone-tuner/history.jsonl
  ✅ Tune complete.
```

### 6. Filter writeback (parameter-by-name)

After PIDs are written, filter recommendations are also applied — but
not via the binary `MSP_SET_FILTER_CONFIG` (cmd 93) blob, whose payload
layout shifts between Betaflight 4.x versions. Instead, the CLI
translates each `FilterRecommendation` into one or more
`(setting_name, value_bytes)` writes and dispatches them via
**`MSP2_COMMON_SET_SETTING`** (cmd `0x1004`) — the same name-based
parameter interface Configurator's CLI uses. The FC resolves the name
through its own settings table, so we don't have to track binary
offsets per firmware version.

Coverage:

- `gyro_lpf{1,2}_static_hz` + `gyro_lpf{1,2}_type`
- `dterm_lpf{1,2}_static_hz`, `dterm_lpf1_dyn_min_hz`,
  `dterm_lpf1_dyn_max_hz`, `dterm_lpf{1,2}_type`
- `yaw_lowpass_hz`
- `dyn_notch_count`, `dyn_notch_min_hz`, `dyn_notch_max_hz`
- `rpm_filter_harmonics`, `rpm_filter_min_hz`

Skipped (encoding shifts too much across firmware versions):

- Notch Q-factor (`dyn_notch_q`, `rpm_filter_q`) — scale changes 4.x → 4.x
- Static gyro notches (`gyro_notch{1,2}_hz`/`_cutoff`) — Hz/cutoff
  derivation changes

Per-setting failures (e.g. an older firmware that doesn't recognise a
name) are logged and the batch continues — one stale name doesn't take
out the rest. Pass `--skip-filters` to opt out entirely and write only
PIDs.

## Connection schemes

The `--connection` argument supports three forms:

- `/dev/ttyACM0`, `/dev/ttyUSB0`, `COM3` — bare device path → USB serial.
- `serial:///dev/ttyACM0` — same thing, explicit scheme.
- `simulator://` — in-process MSP simulator. Handshake works, PID
  read/write round-trip, but dataflash is empty.
- `simulator://path/to/file.bbl` — in-process simulator with its
  dataflash preloaded from a real BBL. Lets you exercise the full
  `--pull-bbl` chain without serial hardware.

## Commands

```bash
drone-tuner info                                  # version + capability check
drone-tuner analyze logs/flight.bbl                # parse + analyse, print report
drone-tuner analyze logs/                          # batch over a directory
drone-tuner analyze logs/flight.bbl --list-sessions
drone-tuner analyze logs/flight.bbl --detailed --show-details

drone-tuner compare flight1.bbl flight2.bbl flight3.bbl

drone-tuner validate logs/ --check-issues

drone-tuner monitor /dev/ttyACM0 --rate 100 --duration 30

drone-tuner tune path/to/flight.bbl                          # analyse, no writes
drone-tuner tune path/to/flight.bbl --connection /dev/ttyACM0 --dry-run
drone-tuner tune --pull-bbl --connection /dev/ttyACM0        # download + analyse
drone-tuner tune --pull-bbl --connection /dev/ttyACM0 --keep-bbl ~/logs/flight.bbl
drone-tuner tune path/flight.bbl --connection /dev/ttyACM0 --auto-apply-safe --save-eeprom
drone-tuner tune path/flight.bbl --connection /dev/ttyACM0 --apply-all --save-eeprom \
            --backup ./pre-tune.json

drone-tuner export flight.bbl --output dump.json --format json --include-fft
```

Global flags: `--verbose`, `--detailed-info`, `--output-format
{pretty,json,csv}`.

## Safety design

The tool can change parameters that affect flight safety. The defaults
are deliberately paranoid:

- **No writes without an explicit flag.** `tune` reads but does not
  apply unless `--auto-apply-safe` or `--apply-all` is set.
- **Atomic writeback with rollback.** Every PID write goes
  read → write → on-failure-restore-backup. The backup snapshot is also
  returned to the caller so it can be persisted to disk.
- **EEPROM persistence is opt-in.** Without `--save-eeprom`, changes are
  RAM-only and revert on the next power cycle.
- **Filter writeback is gated** until per-firmware-version offset
  detection lands (see step 6).

The history JSONL log gives you a paper trail per FC across every tune
iteration, keyed on `board_id` + `target_name`.

## Project layout

```
crates/
├── drone-tuner-core/         # analysis library
│   └── src/
│       ├── analysis.rs       # FFT + oscillation detection + filter optimiser
│       ├── analysis/pid.rs   # step-response analysis + PID recommendations
│       ├── blackbox/         # custom Betaflight BBL parser
│       ├── domain.rs         # FlightSession, AnalysisReport, recommendation types
│       ├── filters.rs        # Butterworth / notch / biquad design
│       ├── realtime.rs       # MSP framing, FlightControllerConnection, simulator
│       └── error.rs
└── drone-tuner-cli/          # binary `drone-tuner`
    ├── src/main.rs
    ├── src/history.rs        # ~/.local/share/drone-tuner/history.jsonl
    └── tests/                # integration + command-specific suites

test_data/                    # real .bbl fixtures used by calibration tests
docs/                         # PRD, technical doc, project assessment
```

There are **no Cargo feature flags**. `tune`, `monitor`, MSP serial,
and the in-process `simulator://` simulator are all default-on.

## Library usage

```rust
use drone_tuner_core::{AnalysisEngine, BlackboxParser};

let bytes = std::fs::read("flight.bbl")?;
let mut parser = BlackboxParser::new();
let session = parser.parse_file(&bytes)?;

let mut engine = AnalysisEngine::new();
let report = engine.analyze(&session)?;

println!("Quality score: {:.1}", report.tune_quality_score);
for rec in &report.pid_recommendations {
    println!("{:?} {:?}: {:.1} → {:.1}",
             rec.axis, rec.term, rec.current_value, rec.recommended_value);
}
```

Realtime usage:

```rust
use drone_tuner_core::realtime::FlightControllerConnection;

let mut fc = FlightControllerConnection::connect("/dev/ttyACM0").await?;
let pids = fc.read_pid().await?;
let summary = fc.read_dataflash_summary().await?;
let blob = fc.pull_dataflash(|done, total| {
    eprintln!("{done}/{total}");
}).await?;
```

## Building and testing

```bash
cargo build --release                # release binary at target/release/drone-tuner

cargo test -p drone-tuner-core       # 94 unit tests
cargo test -p drone-tuner-core --test calibration   # real-flight regression fixtures
cargo test -p drone-tuner-cli                       # ~70 CLI tests

cargo clippy
cargo fmt
```

## Acknowledgments

- RustFFT for the FFT backbone
- Betaflight for the blackbox format and MSP protocol
- The FPV community
