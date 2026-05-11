# Engineering QA Notes

Use this file to record problems, design decisions, and fixes that may be useful for the final report.

## 2026-05-11 initial Part 3 setup

- Created minimal ROS2 Python package `auto_nav_part3`.
- Added Pioneer URDF copy at `src/auto_nav_part3/urdf/pioneer.urdf` from the provided project path.
- Added topic/service contract in `docs/TOPICS.md` to reduce cross-member conflicts.
- Added task allocation in `docs/TASK_ALLOCATION.md` with three independent ownership areas.
- Added development plan in `docs/PART3_DEVELOPMENT_PLAN.md`.

Open technical notes:

- The provided URDF references meshes using `package://auto_nav/...`; visual mesh resolution may require the original `auto_nav` package or path rewriting later.
- The current ROS2 nodes are executable placeholders. They prove service/topic boundaries before SLAM, perception, and route optimisation are implemented.
