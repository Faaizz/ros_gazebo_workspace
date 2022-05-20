# ROS 2 Basics

`docs.ros.org` Tutorials.

- [ROS 2 Basics](#ros-2-basics)
  - [Basics](#basics)
    - [General](#general)
    - [Nodes](#nodes)
    - [Pub/Sub](#pubsub)
    - [Services](#services)
    - [Parameters](#parameters)
    - [Actions](#actions)
  - [Usage](#usage)
    - [Using `rqt_console`](#using-rqt_console)
    - [ROS 2 Launch](#ros-2-launch)
    - [Recording & Playing Back Data](#recording--playing-back-data)
  - [Workspaces](#workspaces)


## Basics

### General
```shell
# Type Introspection: Get type specifications
ros2 interface show <interface_type>
```

### Nodes
```shell
# Show node info: Subscribers, Publishers, Services, Actions
ros2 node info <node_name>
# Run node (& specify logger level)
ros2 run <node_pkg> <node_name> ---ros-args --log-level WARN
```

### Pub/Sub
```shell
# List topics
ros2 topic list
# Publish to topic
ros2 topic pub /topic <msg_type> <msg>
# Listen on topic (Subscribe to topic)
ros2 topic echo <topic_name>
# Check topic publishing frequency
ros2 topic hz <topic_name>
```

### Services
```shell
# List services
ros2 service list
# View service type
ros2 service type <srv_name>
# Invoke a service
ros2 service call <service_name> <service_type> <arguments>
```

### Parameters
```shell
# List params
ros2 param list
# Get param type and current value
ros2 param get <node_name> <param_name>
# Set parameter value
ros2 param set <node_name> <param_name> <param_value>
# Save parameters to file
ros2 param dump <node_name>
# Load parameters from file
ros2 param load <node_name> <param_file>
```

### Actions
```shell
# List Actions with types
ros2 action list -t
# Send action goal (with feedback)
ros2 action send_goal --feedback <action_name> <action_type> <values>
```


## Usage

### Using `rqt_console`
GUI tool for introspecting log messages in ROS 2.
```shell
# Launch rqt_console
ros2 run rqt_console rqt_console0
```

### ROS 2 Launch
```shell
# Run launch file
ros2 launch <pkg_name> <launch_file>

```

### Recording & Playing Back Data
```shell
# Record data published to a topic
ros2 bag record <topic_name> [<other_topic_names>, <other_topic_names>, ...]
# Playback recorded data
ros2 bag play <bag_name>
```


## Workspaces
```shell
# Resolve package dependencies
rosdep install -i --from-path <src_path> --rosdistro galactic -y
# Override OS
export ROS_OS_OVERRIDE="ubuntu"
```
