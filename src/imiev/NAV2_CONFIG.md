# Nav2 Configuration for imiev

This document outlines the current Navigation2 configuration settings for the `imiev` robot.

## Global Planner

**Plugin**: `nav2_smac_planner::SmacPlannerHybrid`

The Global Planner is configured to generate kinematically feasible paths for an Ackermann vehicle using the Reeds-Shepp motion model.

- **Motion Model**: `REEDS_SHEPP`
- **Minimum Turning Radius**: 3.65m
- **Analytic Expansion Ratio**: 3.5 (Allows effectively switching to analytic expansion closer to goal)
- **Cost Travel Multiplier**: 2.0
- **Interpolation**: Enabled (`downsample_costmap: false`, `angle_quantization_bins: 72`)

## Local Controller

**Plugin**: `nav2_mppi_controller::MPPIController`

The Model Predictive Path Integral (MPPI) controller is used to follow the global path while avoiding dynamic obstacles.

- **Motion Model**: `DiffDrive` (Note: MPPI handles Ackermann-like constraints via `AckermannConstraints` critic)
- **Batch Size**: 2000 trajectories
- **Time Steps**: 56
- **Model DT**: 0.05s
- **Velocities**:
    - Max Vx: 0.5 m/s
    - Min Vx: -0.1 m/s
    - Max Wz: 1.9 rad/s
- **Critics**:
    - `ConstraintCritic`, `CostCritic`, `GoalCritic`, `GoalAngleCritic`, `PathAlignCritic`, `PathFollowCritic`, `PathAngleCritic`, `PreferForwardCritic`
- **Ackermann Constraints**:
    - `min_turning_r`: 3.65m (Matched to physical limit and planner)

## Smoother

**Plugin**: `nav2_smoother::SimpleSmoother`

Used to smooth plans from the global planner.

- **Tolerance**: 1.0e-10
- **Max Iterations**: 1000
- **Do Refinement**: True

## Robot Footprint

The robot footprint is defined as a bounding box in both local and global costmaps:

- **Dimensions**: ~3.56m x 1.86m
- **Coordinates**: `[ [1.78, 0.93], [1.78, -0.93], [-1.78, -0.93], [-1.78, 0.93] ]`

## Behavior Server

Standard behaviors are enabled (Spin removed for Ackermann compatibility):
- `backup`, `drive_on_heading`, `assisted_teleop`, `wait`

## Velocity Smoother

- **Feedback**: OPEN_LOOP
- **Max Velocity**: [0.26, 0.0, 1.0]
- **Min Velocity**: [-0.26, 0.0, -1.0]
- **Max Accel**: [2.5, 0.0, 3.2]

## Configuration Files

The primary configuration files are located in `src/imiev/config/`:
- `nav2_no_map_params.yaml`: Configuration for GPS-based navigation without a static map.
- `nav2_slam_params.yaml`: Configuration for SLAM-based navigation.
