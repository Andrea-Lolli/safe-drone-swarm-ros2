# SAFE Swarm Project: Scalable Multi-Drone Architecture for Search and Rescue Operations using ROS 2 and Gazebo

Rapid and efficient localization of victims is critical in post-disaster Search and Rescue (SAR) operations. This report presents the design and implementation of a highly scalable, multi-UAV autonomous simulation environment, developed as part of the SAFE project, to optimize airborne SAR missions. 

Leveraging the ROS 2 middleware and the Gazebo physics engine, the system employs a distributed, horizontally scalable architecture utilizing dynamic namespaces for both the drone swarm and parametric emergency beacon nodes.

To automate mission logistics, a custom Ground Control Station (GCS) algorithm processes standard GeoJSON geospatial data, converting GPS coordinates into a local Cartesian frame to generate Boustrophedon (lawnmower) coverage paths. The system dynamically partitions the trajectory among the available drones, proactively managing battery life constraints by generating multi-part missions that accommodate automated "hot-swap" pit stops. 

During the simulated flight, the UAVs capture Radio Signal Strength Indicator (RSSI) data, which is realistically degraded using a Log-Distance Path Loss model with environmental noise. Finally, the swarm's aggregated telemetry is processed via cubic interpolation to generate high-fidelity, global heatmaps. The results demonstrate the system's robustness in seamlessly transitioning from geographic planning to distributed swarm execution, ultimately providing rapid and accurate target estimation in complex disaster scenarios.

## Key Features

* Automated Ground Control Station (GCS): Processes standard WGS84 GeoJSON data to generate optimized Boustrophedon coverage paths.
* Dynamic Mission Partitioning: Automatically splits trajectories among available drones, calculating multi-part missions for automated battery pit stops.
* Realistic RF Physics: Simulates airborne telemetry using a Log-Distance Path Loss model to realistically degrade RSSI data with environmental noise.
* Global Heatmap Generation: Aggregates swarm telemetry and processes it via cubic interpolation to generate high-fidelity spatial heatmaps.
* Horizontal Scalability: Fully decentralized ROS 2 architecture allows deploying anywhere from 1 to N drones using dynamic namespaces.

## Prerequisites

This project is built for the modern ROS 2 software stack. You will need:
* OS: Ubuntu 24.04 (Native or WSL2)
* ROS 2: Jazzy Jalisco (Desktop Install)
* Simulator: Gazebo Harmonic
* Python 3

## Installation and Setup

1. Clone the repository

Open a terminal and clone the repository into your home directory:
```bash
cd ~
git clone https://github.com/Andrea-Lolli/safe-drone-swarm-ros2.git
cd safe-drone-swarm-ros2
```
2. Install ROS 2 and Gazebo dependencies

Ensure your ROS 2 Jazzy environment is sourced, then install the required ROS-to-Gazebo bridge packages:
```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz python3-colcon-common-extensions -y
```
3. Install Python Data Science Libraries

Because Ubuntu 24.04 strictly manages Python environments, install the required libraries via apt:
```bash
sudo apt install python3-pandas python3-scipy python3-shapely python3-matplotlib python3-pyproj python3-numpy -y
```
4. Build the ROS 2 Workspace

Compile the custom interfaces, simulation assets, and drone core logic:
```bash
cd ~/safe-drone-swarm-ros2/safe_ws
colcon build
source install/setup.bash
```

## Running a Mission

1. Generate Flight Paths

Before launching the drones, use the Python script to read the disaster area GeoJSON and generate the distributed flight plans. (Example for 3 drones):
```bash
cd ~/safe-drone-swarm-ros2/safe_ws
python3 generate_path.py -n 3
```
2. Launch the Simulation environment

Open a new terminal, source the workspace, and launch Gazebo with the specified number of drones:
```bash
source ~/safe-drone-swarm-ros2/safe_ws/install/setup.bash
ros2 launch safe_simulation safe_bringup.launch.py num_drones:=3
```
3. Deploy the Swarm

Once the drones are spawned and awaiting commands in the Safe Zone, open a third terminal and trigger the global start command:
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /swarm/start std_msgs/msg/Bool "{data: true}"
```
4. Generate Results

After the drones complete their mission and log the RSSI telemetry, generate the analytical heatmaps:
```bash
cd ~/safe-drone-swarm-ros2/safe_ws
python3 generate_heatmap.py
```
****
Developed as part of the SAFE Project Architecture.
