#!/usr/bin/env python

from eip_bridge.eip_bridge import EIPBridge
from eip_bridge.plc_simulator import PLCSimulator
import rospy


def main():
    rospy.init_node('eip_bridge_node', anonymous=True)

    # Get the parameters
    try:
        config = rospy.get_param('~config')
    except:
        rospy.logfatal('Failed to get "config" parameter')
        exit(2)

    pub_rate = rospy.get_param('~pub_rate', 10)

    # Create the PLC
    plc = PLCSimulator()

    # Add tags to the simulated PLC
    plc.parse_config(config)

    # Create the bridge
    bridge = EIPBridge(plc, "0.0.0.0", config, pub_rate)

    # Run the bridge
    bridge.run()
    rospy.spin()

    bridge.plc.disconnect_plc()


if __name__ == '__main__':
    main()
