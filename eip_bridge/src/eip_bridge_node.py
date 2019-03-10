#!/usr/bin/env python

from eip_bridge import EIPBridge
from eip_msgs.srv import SetModeRequest
from eip_functions import EIPFunctions
import rospy
from time import sleep


if __name__ == '__main__':
    rospy.init_node('eip_bridge_node', anonymous = True)

    # Get the parameters
    try:
        ip = rospy.get_param('~ip')
    except:
        rospy.logfatal('Failed to get "ip" parameter')
        exit(1)

    try:
        config = rospy.get_param('~config')
    except:
        rospy.logfatal('Failed to get "config" parameter')
        exit(2)

    pub_rate = rospy.get_param('~pub_rate', 10)

    # Create the PLC
    plc = EIPFunctions()

    # Create the bridge
    bridge = EIPBridge(plc, ip, config, pub_rate)

    # Run the bridge
    bridge.run()
    rospy.spin()

    bridge.plc.disconnect_plc()

