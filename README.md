# eip_bridge

This repository contains a bridge between ROS messages and the Ethernet/IP (EIP) communication protocol.
The bridge accepts a YAML config file specifying the ROS topics to convert to a list of PLC tags
and PLC tags to convert to ROS messages.  In both cases, the EIP data structure is created based
off of ROS message types and is generated at the beginning of the run.

## Requirements:

1. [PyComm python library 1.0.8](https://github.com/ruscito/pycomm)
2. [PyYAML parser PyYAML-3.11](http://pyyaml.org/wiki/PyYAMLDocumentation)
3. [ROS bridge](http://wiki.ros.org/rosbridge_library)

## Installation

1. Install Required Dependencies:
  1. `rosdep install eip_bridge`
  1. `apt-get install python-trollius`
  1. `pip install pycomm`

## Notes:

- Supported message types: base PLC data types and fixed-length arrays of those data types
- Not currently supported:
  - User-defined type (UDT)
  - User-defined type array

## Usage:

1. Create YAML file listing ROS topics to map to EIP tags
    - You must completely specify both the topic name and expected type.
    - Topics are uni-directional, and must be listed under "Publishers" or "Subscribers" headings:
      - Publishers  : EIP -> ROS
      - Subscribers : ROS -> EIP
    - EIP data structure will be created automatically from topic message structure

1. Add the path to `pycomm` to the `PYTHONPATH` environment variable

    ``` bash
    export PYTHONPATH=<path_to_pycomm_install>
    ```

1. Run the launch file, specifying the path to your config file

    ``` bash
    roslaunch eip_bridge eip_bridge.launch config_file:='/path/to/config.yaml'
    ```

## Example:

``` bash
export PYTHONPATH=<path_to_pycomm_install>
source <path_to_workspace>/devel/setup.bash
roslaunch eip_bridge eip_bridge.launch config_file:=`rospack find eip_bridge`/example/eip_config.yaml
```
