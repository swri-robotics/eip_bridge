import rospy
from .eip_functions import convert_topic_type, topic_to_tag
from std_msgs.msg import *


class Subscriber:
    """
        Subscriber read messages from ROS and update PLC
    """
    def __init__(self, topic_name, topic_base_type, tag_name, length, plc):
        """Construct a Subscriber object_instance """

        self.topic_name = topic_name
        self.topic_type = topic_base_type
        self.length = length
        self.plc = plc

        try:
            self.plc_tag = tag_name or topic_to_tag(topic_name)
        except Exception as e:
            rospy.logerr("Error: setting subscriber on topic {0}: {1}".format(topic_name, e))
            raise

        try:
            self.plc_tag_type = convert_topic_type(topic_base_type)
        except Exception as e:
            rospy.logerr("Error: setting subscriber on topic type {0}: {1}".format(topic_base_type, e))
            raise

        rospy.Subscriber(topic_name, eval(topic_base_type), self.callback, queue_size=1)

    def callback(self,  msg):
        """Update EIP data when a new ROS message is received"""
        result = None
        try:
            if self.length == 0:
                if self.plc_tag_type == 'STRING':
                    result = self.plc.write_tag_string(self.plc_tag, msg.data)
#                elif self.plc_tag_type == 'BOOL':
#                    result = self.plc.write_tag(self.plc_tag, int(msg.data), self.plc_tag_type)
                else:
                    result = self.plc.write_tag(self.plc_tag, msg.data, self.plc_tag_type)
            else:
                result = self.plc.write_tag_array(self.plc_tag, list(msg.data), self.plc_tag_type)
        except Exception as e:
            result = '{}'.format(e)
        finally:
            return result
