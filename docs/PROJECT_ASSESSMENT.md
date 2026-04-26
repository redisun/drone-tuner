# Drone-Tuner — Project Assessment & Path Forward

_Generated 2026-04-26. Captures the state of the workspace at HEAD plus uncommitted work._

## 1. What this project is

A Rust workspace aiming to be an "FPV Tuning Intelligence Platform" — parse Betaflight blackbox logs, run FFT, detect oscillations, recommend PID/filter changes, write the recommendations back to the FC over MSP. `CLAUDE.md` originally promised a Tauri desktop app + ML + realtime auto-tune; reality has caught up to the realtime layer (validated on two real FCs) but the desktop and ML pieces are still aspirational.

```
crates/
  drone-tuner-core/    # library — the brains
    src/
      lib.rs
      domain.rs
      analysis.rs                          (FFT + oscillation classification)
      analysis/{pid,oscillation,filter_optimizer}.rs
      blackbox/
        mod.rs
        simple_parser.rs                   (custom Betaflight BBL parser)
        converter.rs
      filters.rs                           (Butterworth / notch / biquad)
      error.rs
      realtime.rs                          (MSP, validated on Jeno H743 + TBS F7)
  drone-tuner-cli/     # binary
    src/main.rs                            (info, analyze, compare, validate, monitor, tune, export)
    tests/                                 (integration + command-specific + performance)
```

## 2. What works end-to-end (verified)

```bash
cargo build --workspace
cargo run -p drone-tuner-cli -- analyze test_data/btfl_all_old.bbl
```

Parses a real BBL → FFT → oscillation detection → prioritised filter + PID recommendations in seconds. The `info`, `analyze`, `validate`, `compare`, `tune` (with `--apply-all` + `--save-eeprom`) commands work. MSP writeback is validated end-to-end on two flight controllers and survives power cycles.

## 3. Shortcomings (verified)

| Severity | Issue | Evidence |
|---|---|---|
| MED | The filter optimiser proposes dyn-notch range expansion for sub-60Hz peaks, which dyn-notch can't reach (those are frame-mode / prop-wash) | observed in tune output |
| MED | `expected_response = Δstick × 500` is hardcoded (`pid.rs:387`) and ignores the FC's configured rates / super-rate | small-Δstick events report ~-100% overshoot |
| MED | `analysis.rs` PSD lacks `Σw²` window-energy correction, so amplitude thresholds aren't fully portable across logging rates |  |
| MED | Bilinear-transform filter design (`filters.rs:233`) only fully covers the 2-pole branch; higher orders silently degrade |  |
| MED | Only Betaflight format; no INAV / EmuFlight / ArduPilot |  |
| LOW | No `cargo deny` / supply-chain audit (CI runs fmt + clippy + tests, but no advisory check) |  |

## 4. Path forward — prioritised

### Phase 0 — Stop the bleeding (1 day) — DONE 2026-04-26
1. Fixed `AddNotchFilter` test references; `cargo test --workspace` green.
2. Committed the untracked `analysis/` tests and `cli/tests/` directories.
3. Deleted `blackbox_old.rs`. Removed unused `OscillationDetector` methods, `window_buffer_pool`, `bandwidth` field.
4. Updated `CLAUDE.md` to reflect the workspace layout.

### Phase 1 — Solidify the core (1–2 weeks) — DONE 2026-04-26
1. Split `analysis.rs` into `analysis/{pid,oscillation,filter_optimizer}.rs`.
2. CI workflow: `rustfmt`, `clippy -D clippy::all`, `cargo test`.
3. Calibration scaffold: `crates/drone-tuner-core/tests/calibration.rs` snapshots a drift-resistant `ReportSummary` via `insta`.

### Phase 2 — Realtime, for real (2 weeks) — DONE 2026-04-26
1. MSP framing implemented (V1 + V2), `simulator://` in-process MSP simulator added for tests.
2. PID writeback via `MSP_SET_PID` (cmd 202) with read → write → rollback safety.
3. Validated end-to-end on Jeno H743 + TBS Source One F7 (multiple tune iterations, EEPROM persistence across power cycles).

### Phase 2.5 — Filter writeback (cmd 92/93 binary blob) — DONE 2026-04-26
The original "by parameter name via `MSP2_COMMON_SET_SETTING`" approach was abandoned after discovering that command (0x1004) is **not implemented in Betaflight 4.5.x at all** — verified by grepping `src/main/msp/` at the 4.5.1 tag. Configurator's CLI works because the CLI is a separate REPL, not an MSP path. Replaced with `MSP_FILTER_CONFIG` (cmd 92, read) → mutate known offsets → `MSP_SET_FILTER_CONFIG` (cmd 93, write) with rollback. Bytes outside the known-stable set round-trip verbatim, keeping the path version-safe.

### Phase 3 — Algorithm refinement (next)
1. Filter optimiser: gate dyn-notch recs on `peak_frequency >= dyn_notch_min_supported`; sub-60Hz peaks should drive gyro LPF cutoff or be flagged as frame-mode rather than triggering dyn-notch.
2. Step-response: replace hardcoded `Δstick × 500` with rates / super-rate aware scaling.
3. PSD window-energy correction (`Σw²`) so magnitude thresholds become physical.

### Phase 4 — UI (3–4 weeks, optional)
1. Tauri scaffold + React frontend: file picker → FFT spectrum plot → oscillation overlay → recommendation review.

### Phase 5 — ML (defer)
1. Skip until a labelled dataset of pilot-rated tunes exists.

## 5. Take

The analysis engine is genuinely good and the realtime path is now real-hardware proven. The remaining drag is the optimiser proposing changes that don't help (sub-dyn-notch peaks) and a small set of layout-specific MSP holes on stripped vendor firmware. Both are tractable. Recommend tightening the optimiser's recommendation gates first, then adding the binary-blob filter fallback, then UI.
