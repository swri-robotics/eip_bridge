import re
from threading import Lock
from pycomm.ab_comm.clx import Driver as ClxDriver


def topic_to_tag(topic_name):
    # remove the leading topic id and replace /'s with .'s
    # and topic name becomes tag name
    plc_tag = re.sub('eip_bridge/', '', topic_name)
    plc_tag = re.sub('/', '.', plc_tag)

    return plc_tag


def convert_topic_type(topic_type):
    # TODO: Add support for unsigned data types
    # AB PLC's do not have support for unsigned data types,
    # however with a little work some accommodation might be made
    # by mapping into the size data type. If pycomm then still returns a
    # negative value for the tag, it then must be ignored as in invalid.

    # TODO: allow support for uint64 by mapping to array LINT[2]

    if topic_type.find('Bool') >= 0:
        plc_tag_type = 'BOOL'
    elif topic_type.find('Int8') >= 0:
        plc_tag_type = 'SINT'
    # elif base_type == 'Int16' or base_type == 'UInt8':
    elif topic_type.find('Int16') >= 0:
        plc_tag_type = 'INT'
    # elif base_type == 'Int32' or base_type == 'UInt16':
    elif topic_type.find('Int32') >= 0:
        plc_tag_type = 'DINT'
    # elif base_type == 'Int64' or base_type == 'UInt32':
    elif topic_type.find('Int64') >= 0:
        plc_tag_type = 'LINT'
    elif topic_type.find('Float32') >= 0:
        plc_tag_type = 'REAL'
    elif topic_type.find('String') >= 0:
        plc_tag_type = 'STRING'
    else:
        raise ValueError("Not supported")

    return plc_tag_type


class EIPFunctions:
    def __init__(self):
        # initialize a thread safe queue
        # load queue with a single plc obj
        self.plc = ClxDriver()
        self.mutex = Lock()

    def is_connected(self):
        with self.mutex:
            return self.plc.is_connected()

    def connect_plc(self, ip):
        with self.mutex:
            self.plc.open(ip)
            return self.plc.is_connected()

    def disconnect_plc(self):
        with self.mutex:
            if self.plc.is_connected():
                self.plc.close()

    def read_tag(self, tag_name):
        with self.mutex:
            return self.plc.read_tag(tag_name)[0]

    def write_tag(self, tag_name, tag_value, tag_type):
        """
        :param tag_name: tag name, or an array of tuple containing (tag name, value, data type)
        :param tag_value: the value to write or none if tag is an array of tuple or a tuple
        :param tag_type: the type of the tag to write or none if tag is an array of tuple or a tuple
        :return: None is returned in case of error otherwise the tag list is returned
        The type accepted are:
            - BOOL
            - SINT
            - INT'
            - DINT
            - REAL
            - LINT
            - BYTE
            - WORD
            - DWORD
            - LWORD
        """
        with self.mutex:
            return self.plc.write_tag(tag_name, tag_value, tag_type)

    def read_tag_string(self, tag_name):
        with self.mutex:
            return self.plc.read_string(tag_name)

    def write_tag_string(self, tag_name, tag_value):
        with self.mutex:
            return self.plc.write_string(tag_name, tag_value)

    def read_tag_array(self, tag_name, count):
        with self.mutex:
            return self.plc.read_array(tag_name, count)

    def write_tag_array(self, tag_name, tag_array, tag_type):
        with self.mutex:
            self.plc.write_array(tag_name, tag_array, tag_type)

        # always return true; pycomm is not telling us if we succeed or not
        return True
