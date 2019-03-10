#!/usr/bin/env python
# WARING: pip install pycomm may not install the latest pycomm; better to install from the github repo

import rospy
import signal

from time import sleep

from eip_publisher import Publisher
from eip_subscriber import Subscriber
from eip_service import Service
from eip_msgs.srv import SetMode
from eip_msgs.srv import SetModeRequest
from eip_msgs.srv import SetModeResponse

from pycomm.ab_comm.clx import DataError
from pycomm.cip.cip_base import CommError

class EIPBridge:
    """The main EIP Bridge communication object"""
    def __init__(self, plc, ip, config, pub_rate):
        self.plc = plc
        self.ip = ip
        self.config = config
        self.pub_rate = pub_rate

        # initialize empty topic and service lists
        self.subscribers = []
        self.publishers = []
        self.services = []
    
        # parse config file
        if not self.parse_config(self.config, self.plc):
            rospy.logfatal("Error initializing EIP Bridge")
            exit(1)

        # abort if no publishers, subscribers and services are specified
        if not self.subscribers and not self.publishers and not self.services:
            rospy.logfatal("No topics and services specified. Cowardly aborting with nothing to do.")
            exit(1)

        self.mode = SetModeRequest.RUN
        self.inform_mode()

        self.log_throttle = 60

        # Create a service server to set the mode of the PLC and bridge
        self.mode_service = "set_bridge_mode"
        self.mode_server = rospy.Service(self.mode_service, SetMode, self.set_mode)

    def parse_config(self, config, plc):
        # Try to load the publishers
        rospy.loginfo("Attempting to load from 'Publishers' namespace")
        try:
            for cfg in config['Publishers']:
                try:
                    pub = Publisher(cfg['name'], cfg['type'], cfg['tag'], cfg['length'], self.plc)
                    self.publishers.append(pub)

                except KeyError as e:
                    rospy.logwarn("No '{0}' entry defined in config file".format(e.message))

        except Exception as e:
            rospy.logwarn("No publishers specified!")

        # Try to load the subscribers
        rospy.loginfo("Attempting to load from 'Subscribers' namespace")
        try:
            for cfg in config['Subscribers']:
                try:
                    sub = Subscriber(cfg['name'], cfg['type'], cfg['tag'], cfg['length'], self.plc)
                    self.subscribers.append(sub)

                except KeyError as e:
                    rospy.logwarn("No '{0}' entry defined in config file".format(e.message))

        except Exception as e:
            rospy.logwarn("No subscribers specified!")

        # Try to load the services
        rospy.loginfo("Attempting to load from 'Services' namespace")
        try:
            for cfg in config['Services']:
                try:
                    srv = Service(cfg['name'], cfg['type'], self.plc)
                    self.services.append(srv)

                except KeyError as e:
                    rospy.logwarn("No '{0}' entry defined in config file".format(e.message))

        except Exception as e:
            rospy.logwarn("No services specified!")

        # Return
        return True


    def inform_mode(self):
        if(self.mode == SetModeRequest.RUN):
            rospy.loginfo("EIP Bridge is in RUN mode")
        elif(self.mode == SetModeRequest.DEBUG):
            rospy.loginfo("EIP Bridge is in DEBUG mode")
        elif(self.mode == SetModeRequest.SLEEP):
            rospy.loginfo("EIP Bridge is in SLEEP mode")
        return

    def run(self):
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.pub_rate), self.update)

    def set_mode(self, req):

        if req.mode == self.mode:
           return SetModeResponse(True, "Bridge is already in desired state")

        # Check whether the timer updates should happen
        if not req.mode == SetModeRequest.RUN:
            # Stop the update callbacks
            self.timer.shutdown()

            # Disconnect from the PLC
            self.plc.disconnect_plc()

            # Record the mode internally
            self.mode = req.mode
            self.inform_mode()

            return SetModeResponse(True, "Stopped timer")
        else:
            if not self.plc.connect_plc(self.ip):
                self.inform_mode()
                return SetModeResponse(False, "Failed to connect to PLC")
            else:
                self.mode = req.mode
                self.inform_mode()

                # Restart the callbacks
                self.run()
                return SetModeResponse(True, "Started timer")

    def update(self, event):
        if not self.mode == SetModeRequest.RUN:
            rospy.logwarn_throttle(self.log_throttle, "Use the '{0}' service to change mode to RUN ({1})".format(self.mode_service, SetModeRequest.RUN))
            return

        if not self.plc.is_connected():
            try:
                rospy.loginfo("Connecting to PLC at {}".format(self.ip))
                self.plc.connect_plc(self.ip)
                rospy.loginfo("Connected to PLC at {}".format(self.ip))

            except CommError as e:
                rospy.logerr_throttle(self.log_throttle, "Failed to connect to PLC at {0}: {1}. Retry in 10 sec".format(self.ip, e.message))
                sleep(10)
                return

            except Exception as e:
                rospy.logerr_throttle(self.log_throttle, "Error while connecting to PLC at {0}: {1}".format(self.ip, e.message))
                return

        else:
            try:
                for pub in self.publishers:
                    pub_topic = pub.topic_name
                    pub.publish()

            except DataError as e:
                rospy.logwarn("Data error while reading data for topic {2} from PLC at {0}: {1}".
                             format(self.ip, e.message, pub_topic))

            except CommError as e:
                rospy.logerr("Lost connection to PLC at {0}: {1}".format(self.ip, e.message))

            except Exception as e:
                rospy.logerr("Exception while publishing topic {0}: {1}".format(pub_topic, e.message))
