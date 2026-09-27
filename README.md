# BUAV Parachute Payload Model

MATLAB models I wrote for the **Binghamton Unmanned Aerial Vehicle (BUAV)** team to size and check the parachutes on our drone's payload drops. It covers two payloads: a strobing beacon (0.155 kg) and a water bottle (0.255 kg), both released from 160 ft.

Portfolio: [ggps4.github.io](https://ggps4.github.io/)

## What the script does

`parachute_code_v5.m` is split into five sections. You can run them one at a time in MATLAB:

| Section | Question it answers |
|---|---|
| **Parachute size and drag** | What canopy area and diameter give a 5 m/s landing speed for each payload? |
| **Parachute inflation** | How fast does the canopy open (using ripstop nylon permeability), and how quickly does the payload slow down? |
| **Parachute shock and slowdown** | What opening shock force hits the payload and the release mechanism? (NSWC-style inflation curve, solid-cloth exponent j = 6) |
| **Canopy size comparison** | How do 0.40 to 0.80 m canopies compare in velocity decay? |
| **Wind effect on landing** | For winds from 0 to 15 mph, what is the highest opening altitude that still lands within the 50 ft target radius, and how long until touchdown? |

## Key numbers

| | Beacon (0.155 kg) | Water bottle (0.255 kg) |
|---|---|---|
| Canopy area for 5 m/s (Cd = 0.75) | 0.132 m² | 0.218 m² |
| Equivalent diameter | 0.41 m (1.35 ft) | 0.53 m (1.73 ft) |
| Descent speed with the chosen 0.58 m canopy | 3.5 m/s | 4.5 m/s |
| Free-fall speed at 160 ft if the chute never opens | 30.9 m/s | 30.9 m/s |

We picked one 0.58 m canopy so a single design lands both payloads under the 5 m/s target.

## Files

- `parachute_code_v5.m`: the full model as a plain MATLAB script
- `Parachute_Code_V5.mlx`: the original MATLAB Live Script, with outputs

## Running it

Open either file in MATLAB R2022a or newer. The inflation section uses `syms`/`dsolve`, which needs the Symbolic Math Toolbox. Every other section runs on base MATLAB. The inputs for each payload are at the top of each section.

## Assumptions

- Sea-level air density (1.225 kg/m³) and a flat, solid-cloth canopy (Cd = 0.75)
- Canopy area grows quadratically during inflation; the shock model uses a (t/t₀)^j curve
- Steady horizontal wind with no gusts, and the payload drifts at full wind speed
- No swinging (pendulum motion) under the canopy

*Benjamin Novofastovsky, Mechanical Engineering, Binghamton University*
