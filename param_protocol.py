import time
import queue
import utils
from threading import Thread

# https://mavlink.io/en/services/parameter.html
# Note that the docs say ArduPilot implements an incompatible version of this protocol.
# 1 - Ardupilot might have some some rounding errors
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
        self.blocking = False
        self.timeout = 1
        self.raw_message = message

    def __str__(self):
        return "ID_{}_VALUE_{}_COUNT_{}_INDEX_{}".format(self.id, self.value, self.count, self.index)
        #return "PARAM_VALUE__{}__{}".format(str(self.id), str(self.value))

class Params(dict):
    """
    Subclasses dict; storing each param as a key/value pair
     - also implements a message_hook, a queue, and a thread to broker in the param messages
    """
    def __init__(self, mav_connection=None, target_system=0, target_component=0, *args, **kwargs):
        super(dict, self).__init__(*args, **kwargs)
        self.mav_connection = mav_connection
        self.target_system = target_system
        self.target_component = target_component
        self.message_queue = queue.Queue()
        self.initialization_complete = False # should remain false until we receive all params
        self.thread = Thread(target=self.drain_param_messages_from_mav, daemon=False)
        self.debug_logging_enabled = False
        #self.thread = Thread(target=self._drain_message_queue, daemon=False)

    def start_message_draining_thread(self):
        # start a separate thread to drain all of the param messages
        if not self.thread.is_alive():
            self.thread.start()

    def param_message_hook(self, message):
        assert message.get_type() == "PARAM_VALUE"
        #self.receive_message(message)
        self.message_queue.put(message)
        if self.debug_logging_enabled:
            LOGGER.debug(f"Received PARAM_VALUE message: {message}")

    def drain_param_messages_from_mav(self):
        no_response_count = 0
        while no_response_count <= 5:
            response = self.mav_connection.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            if response:
                no_response_count = 0
                continue
                # no need to put this message on the queue, param_message_hook will already do that.
                # self.message_queue.put(response)
            else:
                if self.initialization_complete:
                    break
                no_response_count += 1
        self.drain_message_queue()

    def drain_message_queue(self):
        # loops through the message queue and updates each parameter
        while not self.message_queue.empty():
            message = self.message_queue.get(block=False, timeout=1)
            self.receive_message(message)
            self.message_queue.task_done()

    def get_all_params(self):
        # only sends the param_request_list message, param_message_hook will handle the responses
        self.mav_connection.mav.param_request_list_send(self.target_system, self.target_component)
        self.start_message_draining_thread()

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
        self.mav_connection.mav.param_request_read_send(self.target_system,
                                                        self.target_component,
                                                        param_id.encode(),
                                                        -1)
        # we start the message_draining_thread and wait for it to finish
        #  once the thread finishes our params dicts should have the up-to-date value so just return the relevant param
        self.start_message_draining_thread()
        while self.thread.is_alive():
            time.sleep(.1)
        return self[param_id]

    def set_param(self, param_id, param_value) -> Param:
        """
        Sends a message to the vehicle to set <param_id> to <param_value> and returns the up-to-date Param
        :param param_id: String, the param_id key for the Param
        :param param_value: Float or Int, the value to set the param to
        :return: Param, the most up-to-date Param
        """
        param = self[param_id]
        param.value = param_value
        self.mav_connection.mav.param_set_send(self.target_system,
                                               self.target_component,
                                               param.id.encode(),
                                               param.value,
                                               param.type)
        # start the drain_param_messages_from_mav thread to get the response(s)
        self.start_message_draining_thread()
        updated_param = self.read_param(param_id)
        if updated_param.value != param_value:
            print(f"PARAM UPDATE FAILED! Desired Value: {param_value}\t Actual Value: {updated_param.value}")
            LOGGER.error(f"PARAM UPDATE FAILED! Desired Value: {param_value}\t Actual Value: {updated_param.value}")
        return updated_param


# def disable_radio_failsafe(self):
#     """Update a couple params to allow arming without an RC connection.
#         https://ardupilot.org/copter/docs/common-gcs-only-operation.html#copter"""
#     print("Disabling RC failsafes.")
#     # disable the throttle failsafe param
#     #self.set_param("FS_THR_ENABLE", 0)
#     self.params.set_param("FS_THR_ENABLE", 0)
#     # disable the 'RC Channels' bit in the ARMING_CHECK param
#     # Note: 65470 is a bitmask that evaluates to: [False, True, True, True, True, True, False, True, True, True, True, True, True, True, True, True]
#     #self.set_param("ARMING_CHECK", 65470)
#     self.params.set_param("ARMING_CHECK", 65470)