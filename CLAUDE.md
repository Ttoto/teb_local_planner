# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```bash
cd build && cmake .. && make -j$(nproc)
./teb
```

Dependencies: g2o, Eigen3, Boost (system, thread, graph), Qt5 Widgets, nlohmann/json (header-only at `/usr/include/nlohmann/`), SuiteSparse.

## Architecture

This is a non-ROS port of the TEB (Timed Elastic Band) local planner. The original ROS version is at [rst-tu-dortmund/teb_local_planner](https://github.com/rst-tu-dortmund/teb_local_planner).

**Core algorithm flow:**

1. `TebConfig` (`inc/teb_config.h`) holds ~80 parameters across 7 nested structs (Trajectory, Robot, GoalTolerance, Obstacles, Optimization, HomotopyClasses, Recovery). Serialization to/from JSON via nlohmann `NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE` macros.
2. `TebOptimalPlanner` (`inc/optimal_planner.h`) is the main planner, inheriting from `PlannerInterface`. It builds a g2o hypergraph where vertices represent robot poses and time intervals, and edges encode constraints (velocity, acceleration, obstacles, kinematics, time optimality, etc.).
3. `TimedElasticBand` (`inc/timed_elastic_band.h`) stores the trajectory as sequences of `VertexPose*` and `VertexTimeDiff*`. It handles initialization, adding/removing poses, and trajectory querying.
4. Custom g2o edges/vertices live in `inc/g2o_types/` — each edge type encodes one constraint (velocity limits, obstacle avoidance, etc.).
5. `PoseSE2` (`inc/pose_se2.h`) is the SE(2) pose representation with mutable `x()`, `y()`, `theta()` accessors.

**Key types:** `ObstaclePtr`, `RobotFootprintModelPtr`, `TebVisualizationPtr`, `ViaPointContainer`, `PoseSE2` (all in `teb_local_planner` namespace).

**Visualization layer** (`TebVisualization` in `src/visualization.cpp`) is intentionally stubbed out (ROS-free). The Qt5 GUI in `main.cpp` draws trajectories directly via `QPainter`.

## GUI (main.cpp)

The Qt5 app (`TebDisplayWidget`) draws a 500x500 viewport with coordinate mapping `pixel = world * 100 + 250`. A timer fires the planner every 30ms. The "Edit Config" button opens a dialog editing `teb_config.json` directly — saving recreates the planner with the new parameters.

Theta sliders use integer range [-314, 314] → radians via `* 0.01` (≈ [-π, π]).
