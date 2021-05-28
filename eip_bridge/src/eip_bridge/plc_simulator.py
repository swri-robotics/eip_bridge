import rospy


class Tag:
    def __init__(self, name, value=None):
        self.name = name
        self.value = value

    def get_name(self):
        return self.name

    def get_value(self):
        return self.value

    def set_value(self, value):
        self.value = value


class PLCSimulator:
    def __init__(self):
        self.tags = {}
        self.active = False

    def is_connected(self):
        return self.active

    def connect_plc(self, ip):
        self.active = True
        return self.active

    def disconnect_plc(self):
        self.active = False

    def add_tag(self, tag_name, length, value=None):
        tag = Tag(tag_name, value)
        if length > 0:
            tag.set_value(zip(range(length), [value for value in range(0, length)]))
        self.tags[tag_name] = tag

    def parse_config(self, config):
        # Try to load tags from the publishers namespace
        for ns in config:
            # Try to load the entries under each namespace
            try:
                entry = config[ns]

                rospy.loginfo("Attempting to load tags from '{0}' namespace".format(ns))
                for cfg in entry:
                    try:
                        tag_name = cfg['tag']
                        length = cfg['length']

                        try:
                            value = cfg['sim_value']
                        except KeyError as value_ke:
                            rospy.logwarn("'{0}' entry does not exist for '{1}'; setting '{0}' to 'None'"
                                          .format(value_ke, tag_name))
                            value = None

                        self.add_tag(tag_name, length, value)

                    except KeyError as ke:
                        rospy.logwarn("'{0}' entry does not exist".format(ke))
                    except Exception as ex:
                        rospy.logwarn("Exception: {0}".format(ex))

            except Exception:
                rospy.logwarn("No tags defined in '{0}' namespace".format(ns))

    def write_to_tag(self, tag_name, value):
        if self.is_connected():
            try:
                tag = self.tags[tag_name]
                tag.setValue(value)
                return tag_name

            except KeyError as e:
                raise Exception('Tag "{}" does not exist'.format(e))

        else:
            raise Exception('PLC is not currently connected')

    def read_from_tag(self, tag_name):
        if self.is_connected():
            try:
                tag = self.tags[tag_name]
                return tag.get_value()

            except KeyError as e:
                raise Exception('Tag "{}" does not exist'.format(e))

        else:
            raise Exception('PLC is not currently connected')

    def read_tag_array(self, tag_name, length):
        return self.read_from_tag(tag_name)

    def read_tag(self, tag_name):
        return self.read_from_tag(tag_name)

    def read_tag_string(self, tag_name):
        return self.read_from_tag(tag_name)

    def write_tag_array(self, tag_name, value_list, tag_type):
        # Value is a list, but needs to be stored as a tuple
        value_tuple = zip(range(len(value_list)), value_list)
        return self.write_to_tag(tag_name, value_tuple)

    def write_tag(self, tag_name, value, tag_type):
        return self.write_to_tag(tag_name, value)

    def write_tag_string(self, tag_name, value):
        return self.write_to_tag(tag_name, value)
