# Path Planning Assignment

A simple path planner for a Formula Student-style driverless car. Given blue cones (left boundary), yellow cones (right boundary) and the car pose `(x, y, yaw)`, it returns a list of `(x, y)` waypoints that stays between the two boundaries.

## How to run

```
python -m venv .venv
source .venv/bin/activate      # macOS/Linux
# .venv\Scripts\activate       # Windows
pip install -r requirements.txt
python -m src.run --scenario 1
```

Scenarios `1` to `23` are available. `1`-`20` are the original cases (up to 2 blue and 2 yellow cones). `21`-`23` are the new Part 2 cases (three cones on one side).

## Approach (Part 1)

The algorithm is in `src/path_planning.py`, in `PathPlanning.generatePath()`.

1. **Split cones by color** (yellow = 0 = right, blue = 1 = left) and sort each list by distance from the car.
2. **No cones:** drive 3 m straight along the car's heading.
3. **Form gates:** list every blue/yellow pair, sort by distance between the two cones, and greedily pair them up, using each cone only once. Pairs farther apart than `max_gate_width` (6 m) are not treated as a gate.
4. **Gate midpoints** become waypoints, since the middle of a gate is the middle of the track.
5. **Leftover cones** (no partner on the other side) are shifted sideways by the half track width. Blue is the left boundary, so it is shifted right. Yellow is the right boundary, so it is shifted left. The boundary direction comes from a neighbouring cone of the same color. With only one cone, the direction from the last waypoint is used.
6. **Clean up:** remove duplicate waypoints and order them nearest-first. If the first waypoint is behind the car (more than 90 degrees off its heading), insert a short forward point so the path starts by driving ahead.

## Part 2: three cones on one side

When all the cones are the same color, there are no gates, so the planner uses the leftover-cone logic from step 5. The line through neighbouring cones gives the boundary direction, and each cone is offset by the half-width perpendicular to it. This produces a path parallel to the boundary, following its curve if the cones curve.

**Why this solution:** it is simple, needs no curve fitting or extra libraries, and reuses the same code as the one-sided cases in Part 1. Three cones are enough to estimate a direction and a rough curvature.

**New test cases (`src/scenarios.py`):**

| Scenario | Description |
|---|---|
| 21 | 3 blue cones in a straight line, car facing along them |
| 22 | 3 yellow cones in a straight line, car facing along them |
| 23 | 3 yellow cones forming a curve |

## Assumptions

- The track half-width is half the width of the nearest gate. If there is no gate, it defaults to 1.5 m.
- A blue/yellow pair more than 6 m apart is not a gate.
- Sorting cones by distance to the car approximates their order along the track.
- Cone positions are exact (no sensor noise).

## Limitations

- **Distance sorting** can give the wrong order on sharp turns, hairpins or tracks that loop back near the car.
- **Greedy pairing** can match the wrong blue and yellow cones when they are close together or unevenly spaced.
- **Half-width is an estimate.** With one-sided cones and no gate, the default 1.5 m may not match the real track.
- **One-sided cones don't reveal which way the track curves**, so the offset side is assumed from the cone color alone.
- **The path is a plain polyline**, not smoothed, so a real controller would need to smooth it.
- **No noise handling:** missing, duplicated or misclassified cones are not dealt with.
