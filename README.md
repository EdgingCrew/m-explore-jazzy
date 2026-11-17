# m-explore ROS2 port

ROS2 package port for multi-robot autonomous exploration of [m-explore](https://github.com/hrnr/m-explore). This fork of [m-explore-ros2](https://github.com/robo-friends/m-explore-ros2) targets **ROS 2 Jazzy** (gz sim, slam_toolbox) and provides map merging and exploration for the [edgeAI robot workspace](#Using-it-from-the-edgeAI-workspace).

### Contents
1. [Autonomous exploration](#Autonomous-exploration)
    * [Demo in simulation with a TB3 robot](#Simulation-with-a-TB3-robot)    
    * [Demo with a JetBot](#On-a-JetBot-with-realsense-cameras)
    * [Instructions for the simulation demo](#Running-the-explore-demo-with-TB3)    
2. [Multirobot map merge](#Multirobot-map-merge)
    * [Simulation demo with known initial poses](#Known-initial-poses)
    * [slam_toolbox: parameters, simulation vs real world](#slam_toolbox-parameters-simulation-vs-real-world)
    * [Simulation demo with unknown initial poses](#Unknown-initial-poses)
    * [ROS2 requirements](#ROS2-requirements)
    * [Instructions for simulation demos](#Running-the-demo-with-TB3)
3. [Using it from the edgeAI workspace](#Using-it-from-the-edgeAI-workspace)

## Autonomous exploration

### Simulation with a TB3 robot
https://user-images.githubusercontent.com/8033598/128805356-be90a880-16c6-4fc9-8f54-e3302873dc8c.mp4


### On a JetBot with realsense cameras
https://user-images.githubusercontent.com/18732666/128493567-6841dde0-2250-4d81-9bcb-8b216e0fb34d.mp4


Installing
----------

No binaries yet.

Building
--------

The repository is a [pixi](https://pixi.sh) workspace with ROS 2 Jazzy from [RoboStack](https://robostack.github.io):
```
pixi run import   # TurtleBot3 sources for the simulations (turtle.repos -> turtle/src)
pixi run build
pixi shell
```
`pixi shell` sources `install/setup.bash`, so start a new shell after the first build.

Without pixi, build as a standard colcon package (use rosdep to resolve dependencies in ROS).

RUNNING
-------
To run with a params file just run it with
```
ros2 run explore_lite explore --ros-args --params-file $(ros2 pkg prefix explore_lite)/share/explore_lite/config/params.yaml
```

### Running the explore demo with TB3
The pixi workspace already contains nav2 and the TB3 simulation (`nav2_minimal_tb3_sim`). Otherwise install them following the [tutorial](https://navigation.ros.org/getting_started/index.html#installation).

Then just run the nav2 stack with slam:

```
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True \
  params_file:=$(ros2 pkg prefix explore_lite)/share/explore_lite/config/nav2_params_tb3_sim.yaml
```

`nav2_params_tb3_sim.yaml` is the stock nav2_bringup params file tuned for exploration (relaxed collision monitor, higher TF tolerances, Regulated Pure Pursuit instead of MPPI). With the default params the robot tends to get stuck next to obstacles. Use `headless:=False` to see the Gazebo GUI.

And run this package with
```
ros2 launch explore_lite explore.launch.py
```

You can open rviz2 and add the exploration frontiers marker (topic is `explore/frontiers`) to see the algorithm working and the frontier chosen to explore.

### Additional features
#### Stop/Resume exploration
By default the exploration node will start right away the frontier-based exploration algorithm. Alternatively, you can stop the exploration by publishing to a `False` to `explore/resume` topic. This will stop the exploration and the robot will stop moving. You can resume the exploration by publishing to `True` to `explore/resume`.

#### Returning to initial pose
The robot will return to its initial pose after exploration if you want by defining the parameter `return_to_init` to `True` when launching the node.

#### TB3 troubleshooting
The TurtleBot3 packages are built from the Jazzy branches listed in `turtle.repos`. If one is missing (e.g. `turtlebot3_gazebo` for the multirobot world), run `pixi run import && pixi run build`.

______________________________________________________________________
## Multirobot map merge

This package works with known and unknown initial poses of the robots. It merges the maps of the robots and publishes the merged map. Some results in simulation are shown below.

### Known initial poses

This modality gives normally the best results. The original ROS1 code only supports [slam_gmapping](https://github.com/ros-perception/slam_gmapping) type of maps for the merge. The following shows the result with that.

https://user-images.githubusercontent.com/8033598/144522712-c31fb4bb-bb5a-4859-b3e1-8ad665f80696.mp4

We also support using [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox), see [its settings](#slam_toolbox-parameters-simulation-vs-real-world). The following demo shows the map merging using the currently supported and most used ROS2-SLAM library.

https://user-images.githubusercontent.com/8033598/170846935-cfae9f3f-5edd-43ea-b993-7b3ba1db921b.mp4

### slam_toolbox: parameters, simulation vs real world

[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) publishes `/map` topics whose **width, height, and `info.origin` change over time**, unlike classic [slam_gmapping](https://github.com/ros-perception/slam_gmapping) maps that match the older merge assumptions more closely. The merge node therefore supports extra options (see `map_merge/config/params.yaml`). Below, **expansion** means embedding each robot’s occupancy grid into **one shared canvas** (same resolution, width, height, and origin) using only each message’s `OccupancyGrid.info`, similar in spirit to [this map-expansion approach](https://github.com/gingineer95/Multi-Robot-Exploration-and-Map-Merging/blob/main/src/map_expansion.cpp) and [issue #10](https://github.com/robo-friends/m-explore-ros2/issues/10).

| Parameter | Meaning |
|-----------|---------|
| `expand_slam_maps_to_common_canvas` | **Recommended `true` for slam_toolbox.** Resamples every robot map into a **common grid** so the OpenCV merge sees equal-sized layers. **`false`** falls back to the legacy path (better matched to **gmapping**-style stable maps); with slam_toolbox alone it often looks wrong. |
| `expand_slam_maps_margin_m` | Extra margin (meters) around the union bounding box when building the shared canvas. |
| `expand_slam_maps_apply_init_pose` | Only relevant when **expansion is on**. If **`true`** (default), each robot map is placed on the canvas by its **`init_pose_*`** (x, y, yaw; the rotation pivots about that robot's map origin). If **`false`**, all robot map frames are assumed to **coincide** and `init_pose_*` is ignored. |
| `known_init_poses` | When **`true`**, each robot must declare `map_merge/init_pose_*` parameters so the node can register robots (see `params.yaml`). |
| `/robotN/map_merge/init_pose_x` … `init_pose_yaw` | Pose of robot **N**'s map frame relative to robot **1**'s, which for slam_toolbox is robot N's **start pose** relative to robot 1's: robot **1** is `(0,0,0)`, robot **2+** are **relative** offsets (meters and yaw in radians). |
| `world_frame` | `frame_id` published on the merged map. Set this to a frame that **exists** in your TF tree for RViz (e.g. a namespaced `map`); a non-existent frame (such as `world` if you never publish it) can make the merged map awkward to visualize. |

**Why `init_pose_*` is needed**  
Each slam_toolbox instance starts its `map` frame at its robot's **start pose**, so two maps of the same building are offset by the robots' relative start poses. This holds for real robots and for the Jazzy gz simulation (odometry starts at zero at each spawn pose). Keep **`expand_slam_maps_apply_init_pose: true`** and set **`init_pose_*`** to the start offsets: the spawn poses in simulation (`params.yaml` matches the known-poses spawns of `multi_tb3_simulation_launch.py`), or measured offsets on real robots (robot 1 at the origin of one chosen floor reference, others at measured Δx, Δy, yaw). Use **`false`** only if all robots' map frames already share one origin (e.g. odometry published in a common world frame).

**gmapping vs slam_toolbox (quick)**  
- **gmapping:** often works with **`expand_slam_maps_to_common_canvas: false`** and **`init_pose_*`** only (classic behavior).  
- **slam_toolbox:** use **`expand_slam_maps_to_common_canvas: true`** and **`expand_slam_maps_apply_init_pose: true`** with the robots' start offsets.

### Unknown initial poses 
It works better if the robots start very close (< 3 meters) to each other so their relative positions can be calculated properly.

https://user-images.githubusercontent.com/8033598/144522696-517d54fd-74d0-4c55-9aca-f1b9679afb3e.mp4

### ROS2 requirements

The TB3 simulation launch files in `map_merge/launch/tb3_simulation` target **ROS 2 Jazzy** with Gazebo (gz sim), `nav2_bringup` and `nav2_minimal_tb3_sim`. The default world is `turtlebot3_house`, whose model comes from `turtlebot3_gazebo` (Jazzy branch of [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations); with the pixi workspace: `pixi run import && pixi run build`).

#### SLAM
[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) is available as a binary and is the recommended SLAM; see [slam_toolbox: parameters, simulation vs real world](#slam_toolbox-parameters-simulation-vs-real-world) for merge settings.

The original ROS1 merge logic was built for [slam_gmapping](https://github.com/ros-perception/slam_gmapping), which hasn't been ported officially to ROS2. There is an unofficial port but it lacks to pass a namespace to its launch file. For that, this repo was tested with one of the authors of this package's [fork](https://github.com/charlielito/slam_gmapping/tree/feature/namespace_launch). To use it, git clone it to your workspace and build it with colcon:

```
cd <your/ros2_ws/src>
git clone https://github.com/charlielito/slam_gmapping.git --branch feature/namespace_launch
cd ..
colcon build --symlink-install --packages-up-to slam_gmapping
```

### Running the demo with TB3
First, you'll need to launch the whole simulation stack, nav2 stacks and slam stacks per robot. For that just launch:
```
ros2 launch multirobot_map_merge multi_tb3_simulation_launch.py slam_toolbox:=True
```
Each robot runs in its own namespace (`/robot1`, `/robot2`) with its own TF tree, slam_toolbox and nav2 (parameters: `tb3_simulation/config/nav2_multirobot_params.yaml`, shared by all robots). Use `headless:=True` to skip the Gazebo GUI and `use_rviz:=False` to skip the per-robot rviz2 windows. For gmapping use `slam_gmapping:=True` instead of `slam_toolbox:=True`.

Now run the merging node:
```
ros2 launch multirobot_map_merge map_merge.launch.py
```

By default, the demo runs with known initial poses. You can change that by launching again both launch commands with the flag `known_init_poses:=False`

To explore autonomously, start explore_lite for each robot:
```
ros2 launch explore_lite explore.launch.py namespace:=robot1
ros2 launch explore_lite explore.launch.py namespace:=robot2
```
Each robot explores its own map, so the robots don't coordinate.

#### Exploring together on the merged map
To let the robots explore the **global** frontiers of the merged map and spread out, launch the simulation with `use_merged_map:=True` (it also starts the merging node) and explore_lite with `params_multirobot.yaml`:
```
ros2 launch multirobot_map_merge multi_tb3_simulation_launch.py slam_toolbox:=True use_merged_map:=True
ros2 launch explore_lite explore.launch.py namespace:=robot1 params_file:=$(ros2 pkg prefix explore_lite)/share/explore_lite/config/params_multirobot.yaml
ros2 launch explore_lite explore.launch.py namespace:=robot2 params_file:=$(ros2 pkg prefix explore_lite)/share/explore_lite/config/params_multirobot.yaml
```
How it fits together:
- map_merge places the merged `/map` in its `world_frame` by `init_pose_*` and, with `publish_robot_map_tf`, adds `world -> map` to every robot's TF tree (`/robotN/tf_static`). This needs `known_init_poses` and `expand_slam_maps_to_common_canvas`.
- With `publish_robot_maps`, map_merge also publishes `/robotN/merged_map`: the merged map with that robot's own map on top. Other robots map a robot as an obstacle; on the plain merged map it would stand inside that obstacle and fail to plan.
- Each robot's nav2 global costmap uses its merged map (`global_frame: world`, static layer on `/robotN/merged_map`), so a robot can plan to frontiers only another robot has seen.
- explore_lite searches frontiers on the robot's merged map and shares its current goal on `/explore/robot_goals`. A frontier within `coordination_radius` of another robot's goal is left to that robot, and `coordination_scale` makes frontiers far from the other robots' goals cheaper. A robot keeps its frontier while it exists and isn't claimed; if two robots pick the same one, the one with the smaller node name keeps it.
- `completion_timeout` keeps a robot looking for a while before it finishes, since a frontier can be hidden temporarily (e.g. another robot standing in a doorway is mapped as an obstacle).

Or move each robot with its corresponding rviz2 interface by sending nav2 goals. To see the map merged just launch rviz2:
```
rviz2 -d $(ros2 pkg prefix multirobot_map_merge)/share/multirobot_map_merge/launch/map_merge.rviz
```

______________________________________________________________________
## Using it from the edgeAI workspace

The [edgeai_robot_ws](https://github.com/EdgingCrew/edgeai_robot_ws) runs the two-robot simulation (`multi_robot_simulation.launch.py`, see its README); map merging and exploration run from this workspace in its own pixi environment. Both environments use CycloneDDS on `ROS_DOMAIN_ID=0`, so the nodes see each other.

Map merging with the robots' start poses of the edgeAI simulation:
```
ros2 launch multirobot_map_merge map_merge.launch.py \
  params_file:=<path/to/edgeai_robot_ws>/src/edgeai_robot_description/config/map_merge_params.yaml
```
Exploration, one terminal per robot:
```
ros2 launch explore_lite explore.launch.py namespace:=robot1 \
  params_file:=$(ros2 pkg prefix explore_lite)/share/explore_lite/config/params_multirobot.yaml
ros2 launch explore_lite explore.launch.py namespace:=robot2 \
  params_file:=$(ros2 pkg prefix explore_lite)/share/explore_lite/config/params_multirobot.yaml
```
If you change the spawn poses there, update `init_pose_*` in its `map_merge_params.yaml` to match (robot2's start pose relative to robot1's).

WIKI
----
No wiki yet.

COPYRIGHT
---------

Packages are licensed under BSD license. See respective files for details.
