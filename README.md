# EIP Bridge
[![license - Apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)

This repository contains a bridge between a ROS system and PLC using the Ethernet/IP (EIP) communication protocol.

## Dependencies:
1. [PyComm 1.0.8](https://github.com/ruscito/pycomm)
    - `pip install pycomm==1.0.8`

## Notes:
- Supported message types:
  - Core PLC data types (e.g. integers, floats, booleans, strings, etc.)
  - Fixed-length arrays of core PLC data types
- Not currently supported:
  - User-defined type (UDT)
  - User-defined type array
- Currently only tested on Allen Bradley PLCs

## Configuration
The EIP bridge accepts a YAML config file specifying communication between the ROS system and PLC. This includes:
  - `Subscribers`: PLC tags which should receive data from ROS topics (i.e. ROS -> PLC)
  - `Publishers`: PLC tags which should publish their values as ROS messages (i.e. PLC -> ROS)
  - `Services`: on-demand access via ROS service for getting/setting data of a specified type on arbitrary tags

The format of the YAML configuration file is shown below:

``` yaml
Services:
  - name: <Service topic name>
  - type: <Service message type>
Publishers:
  - name: <ROS topic on which to publish data>
    tag: <PLC tag from which to publish data>
    type: <PLC data type>
    length: <Array length, 0 for single values>
    sim_value: <Arbitrary value to publish for simulation>
Subscribers:
  - name: <ROS topic on which to subscribe>
    tag:  <Tag into which to write data from the ROS system>
    type: <PLC data type>
    length: <Array length (0 for single values)>
    sim_value: <Arbitrary initial value - used only by simulated PLC>
```
See the [example configuration file](eip_bridge/example/eip_config.yaml) for reference

## Usage:
1. Create YAML file listing ROS topics to map to EIP tags
1. **(Optional)** Add the path to `pycomm` to the `PYTHONPATH` environment variable

    ``` bash
    export PYTHONPATH=<path_to_pycomm_install>
    ```
    > Note: this should not be necessary if `pycomm` was installed using `pip`
1. Run the launch file, specifying the path to your config file

    ``` bash
    roslaunch eip_bridge eip_bridge.launch config_file:='/path/to/config.yaml' sim:=<true/false>
    ```
