# AUTO4508 Part 3 ROS2 Minimal Project

This repository is the Team 18 Part 3 starting point for mapping and discovery.
It provides a minimal executable ROS2 Python package, shared topic/service contract documentation, and a team workflow plan.

## Quick start

```bash
# from repository root
colcon build --symlink-install
source install/setup.bash
ros2 launch auto_nav_part3 part3_minimal.launch.py
```

In another terminal:

```bash
source install/setup.bash
ros2 service call /part3/mapping/start std_srvs/srv/Trigger {}
ros2 service call /part3/waypoint/start std_srvs/srv/Trigger {}
ros2 topic echo /part3/system/state
```

## Key documents

- `docs/TOPICS.md` — ROS2 topic and service contract. Update this before changing any interface.
- `docs/TASK_ALLOCATION.md` — 3-person task split designed to minimise code conflicts.
- `docs/PART3_DEVELOPMENT_PLAN.md` — detailed Part 3 development plan.
- `docs/qa.md` — running engineering QA notes for report preparation.

## Git workflow

Develop only on your personal branch:

```bash
git checkout member/<your-name>
# work, test, commit
git merge main
# resolve conflicts locally, then open a merge request / ask team to merge into main
```

Keep `main` as the integration branch.
