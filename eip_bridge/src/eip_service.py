import rospy
import re
from eip_functions import EIPFunctions
from eip_msgs.srv import *
from std_msgs.msg import *


class Service:
    def __init__(self, service_name, service_type, plc):
        base_name = re.sub('eip_bridge/', '', service_name)
        self.plc = plc
        self.service_type = service_type
        self.request_type = eval(service_type + 'Request')
        self.response_type = eval(service_type + 'Response')

        # note that care must be taken given making service names
        rospy.Service(service_name, eval(service_type), getattr(self, 'handle_'+base_name))

    def read_tag(self, req):
        tag_value = None
        error = 'OK'
        success = True

        try:
            if self.plc.is_connected():
                tag_value = self.plc.read_tag(req.tag_name)

            if tag_value is None:
                raise Exception('Tag value is empty')

        except Exception as e:
            error = e.message
            success = False

        if tag_value is None:
            tag_value = 0
            success = False

        return self.response_type(tag_value, success, error)

    def read_tag_string(self, req):
        tag_value = None
        error = 'OK'
        success = True

        try:
            if self.plc.is_connected():
                tag_value = self.plc.read_tag_string(req.tag_name)

            if tag_value is None:
                raise Exception('Tag value is empty')

        except Exception as e:
            error = e.message
            success = False

        if tag_value is None:
            tag_value = 0
            success = False

        return self.response_type(tag_value, success, error)

    def read_tag_array(self, req):
        tag_value = None
        error = 'OK'
        success = True

        try:
            if self.plc.is_connected():
                tag_value = map(lambda x: x[1], self.plc.read_tag_array(req.tag_name, req.count))

            if tag_value is None:
                raise Exception('Tag value is empty')

            if len(tag_value) == 0:
                raise Exception('"{}" tag has length of 0'.format(req.tag_name))

        except Exception as e:
            error = e.message
            success = False

        return self.response_type(tag_value, success, error)

    def write_tag(self, req, typ):
        tag_list = None
        error = 'OK'
        success = True

        try:
            if self.plc.is_connected():
                tag_list = self.plc.write_tag(req.tag_name, req.tag_value, typ)

            if tag_list is None:
                raise Exception('Tag list is empty')
            elif not tag_list:
                raise Exception('Failed to write to tag "{}"'.format(req.tag_name))
            else:
                tag_list = req.tag_name

        except Exception as e:
            tag_list = ''
            error = e.message
            success = False

        return self.response_type(tag_list, success, error)

    def write_tag_string(self, req):
        tag_list = None
        error = 'OK'
        success = True

        try:
            if self.plc.is_connected():
                tag_list = self.plc.write_tag_string(req.tag_name, req.tag_value)

            if tag_list is None:
                raise Exception('Tag list is empty')
            elif not tag_list:
                raise Exception('Failed to write to tag "{}"'.format(req.tag_name))
            else:
                tag_list = req.tag_name

        except Exception as e:
            tag_list = ''
            error = e.message
            success = False

        return self.response_type(tag_list, success, error)

    def write_tag_array(self, req, typ):
        error = 'OK'
        success = True

        try:
            if self.plc.is_connected():
                self.plc.write_tag_array(req.tag_name, list(req.tag_value), typ)

        except Exception as e:
            error = e.message
            success = False

        return self.response_type(success, error)

    def handle_read_tag_int8(self, req):
        return self.read_tag(req)

    def handle_read_tag_int16(self, req):
        return self.read_tag(req)

    def handle_read_tag_int32(self, req):
        return self.read_tag(req)

    def handle_read_tag_int64(self, req):
        return self.read_tag(req)

    def handle_read_tag_string(self, req):
        return self.read_tag_string(req)

    def handle_read_tag_array_int8(self, req):
        return self.read_tag_array(req)

    def handle_read_tag_array_int16(self, req):
        return self.read_tag_array(req)

    def handle_read_tag_array_int32(self, req):
        return self.read_tag_array(req)

    def handle_read_tag_array_int64(self, req):
        return self.read_tag_array(req)

    def handle_read_tag_bool(self, req):
        return self.read_tag(req)

    def handle_write_tag_int8(self, req):
        return self.write_tag(req, 'SINT')

    def handle_write_tag_int16(self, req):
        return self.write_tag(req, 'INT')

    def handle_write_tag_int32(self, req):
        return self.write_tag(req, 'DINT')

    def handle_write_tag_int64(self, req):
        return self.write_tag(req, 'LINT')

    def handle_write_tag_string(self, req):
        return self.write_tag_string(req, 'STRING')

    def handle_write_tag_array_int8(self, req):
        return self.write_tag_array(req, 'SINT')

    def handle_write_tag_array_int16(self, req):
        return self.write_tag_array(req, 'INT')

    def handle_write_tag_array_int32(self, req):
        return self.write_tag_array(req, 'DINT')

    def handle_write_tag_array_int64(self, req):
        return self.write_tag_array(req, 'LINT')

    def handle_write_tag_bool(self, req):
        return self.write_tag(req, 'BOOL')

