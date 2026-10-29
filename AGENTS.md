# Cane Planner

The current focus is planner 3 on `mpc-lfpc-prototype`. Read `README.md` for
architecture, launch commands and supported modes.

## Build and verification

- Use `catkin build`; never mix it with `catkin_make` in this workspace.
- Run checks relevant to the change. Update documentation when behavior changes.

## ROS processes

- Preserve user-owned ROS masters and nodes unless the user requests a restart.
- Unattended tests use an unused private master port, verify process ownership,
  and clean up only their own processes.

## Project references

- Planning contracts: `docs/development/planning-runtime/index.md`.
- RViz/operator guidance: `docs/development/operator-visualization/index.md`.
- Social-force simulation: read `docs/PEDSIM_IMPLEMENTATION_PROGRESS.md` before
  continuing map, pedsim, multi-pedestrian or scene-editor work; update its status
  and next step when ending a stage.
- Trellis is retired in this project. `docs/archive/trellis/` contains historical
  records, not active instructions. Work directly without its task, approval,
  hook or journal workflow.
