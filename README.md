# Traffic-dashboard
Dashboard that displays Google Maps Route information using ROS

## Features
- Publishes commute routes via `/route_data`
- Displays ETA and arrival time
- Color-coded traffic segments 

## Requirements
- ROS Noetic (Python 3)
- Python deps: `pip install requests polyline`
- Google Cloud API key with the **Routes API** enabled

## Setup
```bash
cd ~/catkin_ws/src
git clone https://github.com/floydpiper/traffic_dashboard.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
