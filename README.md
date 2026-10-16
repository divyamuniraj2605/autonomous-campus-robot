# Alliance Campus Autonomous Navigation Robot

A ROS 2-based autonomous robot simulation for Alliance Campus, featuring a simulated mobile robot, LiDAR-based obstacle avoidance, campus destination data, and planned SLAM-based mapping and autonomous navigation.

> **Implementation status:** The supplied project archive contains the campus world, robot model, LiDAR/odometry configuration, obstacle-avoidance node, destination configuration, destination lookup script, and Gazebo campus-label plugin. SLAM mapping and map-based goal navigation must be integrated and verified before they are described as completed features.

## Project Overview

This project aims to simulate a mobile robot navigating a campus-like environment. The robot model includes a simulated 2D LiDAR and odometry interface. A ROS 2 obstacle-avoidance node uses LiDAR readings to respond to nearby obstacles, while a YAML configuration stores named campus locations and their coordinates.

The planned SLAM integration will allow the robot to estimate its pose while building a 2D occupancy-grid map. The generated map can then be used with localization and a navigation planner to move toward selected campus destinations.

## Current Features

- Campus simulation world represented using SDF files.
- Custom four-wheel robot model defined in URDF.
- Simulated 2D LiDAR configuration publishing to `/scan`.
- Odometry configuration publishing to `/odom`.
- Reactive LiDAR-based obstacle-avoidance node.
- YAML file containing named campus destinations, coordinates, and aliases.
- Console script for matching destination names and aliases.
- Gazebo GUI plugin for campus location labels and a location-selection panel.

## Planned Features / Work in Progress

- Integration of a compatible 2D SLAM package.
- Generation and saving of an occupancy-grid map.
- Verification of robot pose estimates and TF frame relationships.
- Map-based localization.
- Global path planning and goal-oriented navigation to campus destinations.
- Controlled experiments for sensor/odometry noise, loop closure, and environment variation.
- Quantitative evaluation and plots for mapping and navigation performance.

## Repository Structure

The exact layout may vary as the project develops. The main files in the supplied archive include:

```text
alliance-campus-autonomous-navigation/
├── config/
│   └── locations.yaml
├── urdf/
│   └── campus_robot.urdf
├── worlds/
│   ├── alliance_campus.sdf
│   └── alliance_campus_gazebo11.sdf
├── plugins/
│   └── campus_labels.cpp
├── alliance_campus_robot/
│   └── obstacle_avoidance.py
├── scripts/
│   └── destination_manager.py
├── launch/                 # Add verified launch files
├── slam_config/             # Add selected SLAM configuration
├── maps/                    # Store generated maps and metadata
├── nav2_config/             # Add if using Nav2
├── results/                 # Store experiment data and logs
├── plots/                   # Store generated graphs
└── report/                  # Store project report
```

The archive contains duplicate project copies and two world variants. Choose one maintained project directory and clearly document which world file is used for final experiments.

## Software and Dependencies

The exact requirements depend on the ROS 2 distribution, simulator, and SLAM package selected for the final implementation.

Record the following in the final version of this README:

- Operating system and version.
- ROS 2 distribution.
- Gazebo or Webots version.
- Required ROS 2 packages and Python dependencies.
- Selected SLAM package and version.
- Navigation stack and planner, if used.
- Computer specifications used for benchmarking.

**Compatibility note:** The supplied robot URDF references Gazebo Classic-style plugins. Confirm that these plugins are supported by the simulator installed on your system. If using a newer Gazebo Sim setup, the model and plugin configuration may need to be adapted.

## Setup

Add the exact installation and build commands after testing them on the target computer. A typical ROS 2 workspace workflow may look like this, but package names and commands must be checked against your project:

```bash
# From the ROS 2 workspace root
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Do not run these commands until the project is placed in a correctly structured ROS 2 workspace and its package manifests declare the required dependencies.

## Running the Project

Document the verified commands for each component here:

1. Launch the campus world.
2. Spawn or launch the robot.
3. Confirm that `/scan` and `/odom` publish valid messages.
4. Start the obstacle-avoidance node.
5. Start the selected SLAM package after its configuration is complete.
6. Save the generated map.
7. Launch localization and navigation, if integrated.
8. Send a goal corresponding to a verified campus destination.

The archive does not provide a verified end-to-end launch command for all these stages. Replace this section with tested commands before submission.

## Important Integration Notes

### Topics and coordinate frames

The supplied robot configuration is intended to publish LiDAR scans on `/scan` and odometry on `/odom`. Verify the live topic names, message types, timestamps, and TF tree in the selected simulator. Confirm that the SLAM package's scan topic and frame parameters match the running system.

### Velocity-command ownership

The current obstacle-avoidance node publishes velocity commands to `/cmd_vel`. A navigation stack may also publish velocity commands. Do not run two independent controllers on the same command topic without an explicit command-arbitration or multiplexer design.

### Campus destination coordinates

`config/locations.yaml` contains configured destination coordinates and aliases. Verify that each coordinate matches the selected world and the coordinate frame used by localization/navigation. Check that goals lie in reachable free space and not inside buildings or obstacles.

### Destination lookup

The supplied `scripts/destination_manager.py` performs console-based matching of a typed destination name or alias. It is not, by itself, a speech-recognition system or a navigation-goal client. The script's file path should be made portable before the repository is shared.

## SLAM and Navigation Evaluation

The semester assessment calls for the following experiments:

1. **Baseline mapping:** Run the assigned SLAM method on the standard route and save the map and trajectory evidence.
2. **Sensor/odometry noise:** Add controlled LiDAR and/or wheel-odometry noise and compare results with the baseline.
3. **Loop closure:** Revisit a previously mapped location and compare map/trajectory consistency before and after the revisit.
4. **Environment variation:** Change the layout or obstacle arrangement and observe the effect on mapping and navigation.

Where feasible, record:

- Map-quality score or error, including the scoring method.
- Absolute Trajectory Error (ATE) and/or Relative Pose Error (RPE).
- Map-update latency or clearly defined elapsed time.
- CPU and memory usage.
- Loop-closure behaviour and robustness.
- Planned path length and actual travelled distance, clearly distinguished.
- Planning time and expanded nodes, if available.
- Goal success and collision count.

Use consistent robot, sensor, route, map-resolution, and test-duration settings for comparisons. Repeat trials where practical and report failures as well as successes. Do not invent missing measurements.

## Results

Add the actual results after running the experiments. Recommended files include:

```text
results/
├── experiment_log.csv
├── raw_data/
└── run_notes.md

plots/
├── trajectory_comparison.png
├── processing_time_comparison.png
└── robustness_comparison.png
```

Include at least two graphs based on measured data in the final assessment. Explain the method and conditions behind each metric.

## Limitations

- SLAM and map-based navigation are not verified as completed in the supplied archive.
- The current obstacle-avoidance logic is reactive and uses configured road boundaries; it is not a global path planner.
- Destination coordinates must be validated against the world and map coordinate frames.
- The GUI location-selection controls are not shown connected to a navigation-goal action in the supplied archive.
- Simulator/plugin compatibility must be confirmed.
- Duplicate project trees and alternative world files should be consolidated or clearly documented.
- No measured experiment results are included in the supplied archive.

## Reproducibility Checklist

- [ ] Record operating system, ROS 2 distribution, simulator version, and package versions.
- [ ] Select and document the canonical campus world file.
- [ ] Verify robot movement, `/scan`, `/odom`, and required TF frames.
- [ ] Verify SLAM and save the map plus metadata.
- [ ] Verify localization and goal navigation before claiming them as implemented.
- [ ] Record algorithm parameters and exact noise settings.
- [ ] Save raw measurements, screenshots, maps, trajectories, and plots.
- [ ] Test README commands from a fresh terminal/session.
- [ ] Cite external repositories, packages, and licences.
- [ ] Replace all draft instructions with tested commands before final submission.

## Attribution and References

List all external repositories, ROS 2 packages, simulator resources, research papers, and documentation used by the project. Follow the relevant licences and clearly identify team modifications.

- ROS 2 documentation: https://docs.ros.org/
- Gazebo documentation: https://gazebosim.org/docs
- Add the official documentation for the selected SLAM package.
- Add the official documentation for the navigation stack/planner, if used.
- Add the original source repository and licence for reused code.

## Authors

- Team:
- `Divya M`
- `Keerthana R`
- `Vedika Chand`
- `Harshitha B L`
- `HArshitha B S`
- `Bhoomi jaiswal`
- `Sanskriti Srivastava`

