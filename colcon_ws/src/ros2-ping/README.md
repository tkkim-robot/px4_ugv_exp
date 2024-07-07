# ROS2 Ping


## Installation

Clone the repo to your `colcon_ws`
```
cd colcon_ws/src
git clone https://github.com/dasc-lab/ros2-ping
```

Build the workspace
```
cd colcon_ws
colcon build
```

Source the workspace 
```
source install/setup.bash
```


## Usage

One one terminal, start `pong` node
```
ros2 run ping pong
```
This node waits for messages on the `/ping` topic. When it receives something, it will republish the same message to the `/pong` topic. 

On the other terminal, start the `ping` node
```
ros2 run ping ping
```
This node sends out a message on the `ping` topic every 500ms. 
Whenever it receives a message on the `pong` topic, it alculates the time difference. 
It prints the time difference to console, and also publishes it to the `/round_trip` topic as a `builtin_interfaces::msg::Duration` msg. 

One a single computer, the ping should be about 1 ms. 


