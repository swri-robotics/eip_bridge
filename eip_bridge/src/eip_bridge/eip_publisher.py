import rospy
from .eip_functions import convert_topic_type, topic_to_tag
from std_msgs.msg import *


class Publisher:
    """
        Publisher forwards messages from PLC to ROS
    """
    def __init__(self, topic_name, topic_base_type, tag_name, length, plc):
        """Construct a Publisher object_instance """

        self.topic_name = topic_name
        self.topic_type = topic_base_type
        self.length = length
        self.plc = plc
        self.pub = rospy.Publisher(self.topic_name, eval(self.topic_type), queue_size=1)

        try:
            self.plc_tag = tag_name or topic_to_tag(topic_name)
        except Exception as e:
            rospy.logerr("Error: setting publisher on topic {0}: {1}".format(topic_name, e))
            raise

        try:
            self.plc_tag_type = convert_topic_type(topic_base_type)
        except Exception as e:
            rospy.logerr("Error: setting publisher on topic type {0}: {1}".format(topic_base_type, e))
            raise

    def publish(self):
        """Publish ROS msg with PLC data"""

        if self.length > 0:
            tag_value = self.plc.read_tag_array(self.plc_tag, self.length)

            # if this is an array, tag_value will be a list of tuples (index, data)
            # use the map function below to separate the data from the index if required
            tag_value = map(lambda x: x[1], tag_value)

        elif self.plc_tag_type == 'STRING':
            tag_value = self.plc.read_tag_string(self.plc_tag)
        elif self.plc_tag_type == 'BOOL':
            tag_value = bool(self.plc.read_tag(self.plc_tag))
        else:
            tag_value = self.plc.read_tag(self.plc_tag)

        # save of the message type
        msg_type = eval(self.topic_type)

        # make sure to convert/cast the data to be published to the correct type
        # using keyword 'data' to set the data; relying on the data type's
        # constructor using args will not always work (as in the case of *MultiArray's)
        msg = msg_type(data=tag_value)

        self.pub.publish(msg)
