# ax1ev-port-radar branch composition

This is an integration branch that combines several work branches on top of `sunnypilot/opendbc` master.
It is not in a state that can go upstream as-is, so this file records which branches came in and how.
When merging or cherry-picking another branch, add a row to the tables below.

## Base

| Item | Value |
|---|---|
| Base | `origin/master` (mirror of `sunnypilot/opendbc` master, merged periodically) |
| Last master merge | `335adbf0` (2026-10-21, master `5be75071`) |

## Merged branches

| Branch | Source | How | Commit | Contents |
|---|---|---|---|---|
| `elantra-2025-port` | Jason Wen (`origin/elantra-2025-port`, `c8810ccf`) | squash (part of `14c49615`) | `14c49615` | Hyundai CAN "refreshed" message support: `hyundai_can_refresh.dbc`, `HyundaiRefreshPlatformConfig`, `CAN_REFRESH_MSGS` flag, Elantra 2024-25 / Elantra Hybrid 2024-25 |
| `elantra-2025-port-sp` | Alex4386 (`origin/elantra-2025-port-sp`, `4b316aa4`) | squash (part of `14c49615`) | `14c49615` | sunnypilot version of the branch above, plus initial Inster support |
| `ax1ev-port-sp` | Alex4386 (`origin/ax1ev-port-sp`, `89c7a3bf`) | squash (part of `14c49615`) | `14c49615` | Inster (AX1EV) port: `ALT_AX1EV_BUTTONS`, `AX1_CRUISE_BUTTONS` / `AX1_LDA_BUTTON`, safety |
| (LDA hack) | Alex4386 | squash | `14c49615` | Inster LDA button to MADS, split of `_hyundai_can_common.dbc` |
| `hyundai-radar-tracks` | royjr (`origin/hyundai-radar-tracks`, `82683e54`) | merge | `335adbf0` | Master merge fixes: AX1EV buttons moved to `msg_matches()`, `CAN_REFRESH_MSGS` / `ALT_AX1EV_BUTTONS` moved to bits 28/29 (`HAS_BSM` took 27), restore `ignore_frequency_check` in `safety_tick()` (dropped by the upstream sync) so the 5 Hz AX1EV button rx checks stay valid |
| `fcb6405e` | Fingerprint-based Hyundai radar track detection (`500_51F`, `210_21F`, `235_248`, `3A5_3C4`, `602_617`). Inster uses the Mando MRR35 (`0x3A5`-`0x3C4`) on bus 1 |

`14c49615` squashes the `elantra-2025-port` -> `elantra-2025-port-sp` -> `ax1ev-port-sp` line of work into one commit,
so git ancestry does not connect it to the original branches. The sources were identified by comparing file contents.

## Work done on this branch

| Commit | Contents |
|---|---|
| `fca0d767` | Forward the stock HDA state when sending `LFAHDA_MFC` (fixes the HDA indicator disappearing), `Navi_HU.Navi_RoadClass`, `AX1_CRUISE_BUTTONS` value table |
| `9c7f1381` | Decode `HDA11_MFC` (0x484) lane info: left/right lane status, lane crossing pulses, lane angle |
| `fcb6405e` | Fixes on top of the radar-tracks merge: drop deprecated `RadarPoint` fields, remove debug `print()`s, default when the `HyundaiRadar` param is unset, publish radar data at 20 Hz |

## Known issues / TODO

- `hyundai-radar-tracks` removed the `MANDO_RADAR` flag from every platform, but `_initialize_radar_tracks()` in
  `sunnypilot/car/interfaces.py` still uses that flag to decide whether to enable radar tracks (UDS `0x7d0`), so it never runs.
  Mando radar platforms that need tracks enabled (Sonata, Santa Fe, Telluride, ...) may not get any tracks.
- MRR35 `REL_ACCEL` does not match `d(REL_SPEED)/dt` on Inster data (correlation 0.57, slope 0.19). Currently unused.

## Reviewed but not merged

| Branch | Source | Reason |
|---|---|---|
| `ccnc-lane-change-smonth` | winstar0070/opendbc | ccNC cluster only (CAN-FD `CCNC_0x161` / `CCNC_0x162`). The Inster is classic CAN and has neither message. The closest Inster equivalent is `HDA11_MFC` (`0x484`), whose DBC decode is included |
