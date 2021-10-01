import time
import utils
from threading import Thread

# https://mavlink.io/en/services/parameter.html
# Note that the docs say ArduPilot implements an incompatible version of this protocol.
# 1 - Ardupilot might have some some rounding errors?
# 2 - A PARAM_VALUE is not emitted after the parameter update is received.

LOGGER = utils.create_logger(log_name="PARAMS")

class Param(object):
    def __init__(self, message):
        assert message.name == "PARAM_VALUE"
        self.id = message.param_id
        self.value = message.param_value
        self.type = message.param_type
        self.count = message.param_count
        self.index = message.param_index
        self.timestamp = message._timestamp
        self.blocking = False
        self.timeout = 1
        self.raw_message = message

    def __str__(self):
        return "ID_{}_VALUE_{}_COUNT_{}_INDEX_{}".format(self.id, self.value, self.count, self.index)


class Params(dict):
    """
    Subclasses dict; storing each param as a key/value pair
     - also implements a message_hook broker in the param messages
    """
    def __init__(self, copter, *args, **kwargs):
        super(dict, self).__init__(*args, **kwargs)
        self.mav = copter.mav
        self.target_system = copter.target_system
        self.target_component = copter.target_component
        self.param_draining_thread = Thread(target=self.drain_params, daemon=False)
        self.initialization_complete = False # should remain false until we receive all params

    def param_message_hook(self, message):
        # receives the message from message_router and updates the param value in our dict
        assert message.get_type() == "PARAM_VALUE"
        param = Param(message)
        self[param.id] = param
        if param.index == (param.count - 1):
            self.initialization_complete = True

    def get_all_params(self):
        # only sends the param_request_list message, param_message_hook will handle the responses
        self.mav.mav.param_request_list_send(self.target_system, self.target_component)
        self.start_param_draining_thread()
        self.wait_for_initialization_complete()

    def wait_for_initialization_complete(self):
        while not self.initialization_complete:
            self.mav.recv_match(type='PARAM_VALUE')

    def start_param_draining_thread(self):
        if not self.param_draining_thread.is_alive():
            self.param_draining_thread = Thread(target=self.drain_params, daemon=False)
            self.param_draining_thread.start()

    def drain_params(self):
        no_response_count = 0
        while no_response_count <= 5:
            response = self.mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            if response:
                no_response_count = 0
                continue
                # no need to do anything with the message here
                #  reading the message automatically sends it to the message_hooks so param_message_hook will handle it
            else:
                if self.initialization_complete:
                    break
                no_response_count += 1

    def receive_message(self, message):
        # updates the param value in our dict
        param = Param(message)
        self[param.id] = param
        if param.index == (param.count - 1):
            self.initialization_complete = True

    def is_initialization_complete(self) -> bool:
        return self.initialization_complete

    def read_param(self, param_id) -> Param:
        """
        Reads the param value from the vehicle for a given param_id and returns that Param
        :rtype: Param
        """
        old_timestamp = self[param_id].timestamp
        self.mav.mav.param_request_read_send(self.target_system,
                                                        self.target_component,
                                                        param_id.encode(),
                                                        -1)
        current_timestamp = self[param_id].timestamp
        loop_time = 0
        while old_timestamp == current_timestamp:
            # copter.drain_mav is running in another thread which will eventually route the param_value message
            #  to param_message_hook which will in turn update our dict. So we just wait for the param to be updated
            time.sleep(.5)
            loop_time += .5
            current_timestamp = self[param_id].timestamp
            if loop_time / .5 >= 10:
                # if this goes over ~5 seconds then something went wrong
                # TODO need exception here
                break
        return self[param_id]

    def set_param(self, param_id, param_value) -> Param:
        """
        Sends a message to the vehicle to set <param_id> to <param_value> and returns the up-to-date Param
        :param param_id: String, the param_id key for the Param
        :param param_value: Float or Int, the value to set the param to
        :return: Param, the most up-to-date Param
        """
        param = self[param_id]
        old_timestamp = param.timestamp
        param.value = param_value
        self.mav.mav.param_set_send(self.target_system,
                                               self.target_component,
                                               param.id.encode(),
                                               param.value,
                                               param.type)
        current_timestamp = self[param_id].timestamp
        loop_time = 0
        while old_timestamp == current_timestamp:
            # copter.drain_mav is running in another thread which will eventually route the param_value message
            #  to param_message_hook which will in turn update our dict. So we just wait for the param to be updated
            time.sleep(.5)
            loop_time += .5
            current_timestamp = self[param_id].timestamp
            if loop_time / .5 >= 10:
                # if this goes over ~5 seconds then something went wrong
                # TODO need exception here
                break
        updated_param = self.read_param(param_id)
        if updated_param.value != param_value:
            # TODO need exception here
            print(f"PARAM UPDATE FAILED! Desired Value: {param_value}\t Actual Value: {updated_param.value}")
            LOGGER.error(f"PARAM UPDATE FAILED! Desired Value: {param_value}\t Actual Value: {updated_param.value}")
        return updated_param

