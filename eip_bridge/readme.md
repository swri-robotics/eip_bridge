## Description:
This package contains a bridge between ROS msgs and the Ethernet/IP (EIP) communication protocol.
The bridge accepts a YAML config file specifying the ROS topics to convert to a list of PLC tags
and PLC tags to convert to ROS messages.  In both cases, the EIP
datastructure is created based off of ROS msg types and is generated at the beginning of the run.

## Usage:
1. Install Required Dependencies (see below):
  1. `rosdep install eip_bridge`  (installs all package.xml dependencies)
  1. `apt-get install python-trollius`
  1. `pip install pycomm`
1. Create YAML file listing ROS topics to map to EIP tags
  - You must completely specify both the topic name and expected type.
  - Topics are uni-directional, and must be listed under "Publishers" or "Subscribers" headings:
      - Publishers  : EIP -> ROS
      - Subscribers : ROS -> EIP
  - EIP data structure will be created automatically from topic message structure
1. Run the launch file, specifying the path to your config file
  - `roslaunch eip_bridge eip_bridge.launch config_file:='/path/to/config.yaml'`

## Example:
``
roslaunch eip_bridge eip_bridge.launch config_file:=`rospack find eip_bridge`/example/eip_config.yaml
``

## Notes:
  - Supported message types: base PLC data types and fixed-length arrays of thos data types
  - Not currently supported: UDT's, array of UTD

## Requirements:

1. PyComm python library 1.0.8 https://github.com/ruscito/pycomm
2. PyYAML parser PyYAML-3.11 http://pyyaml.org/wiki/PyYAMLDocumentation
3. ROS bridge http://wiki.ros.org/rosbridge_library
