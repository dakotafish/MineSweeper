import logging
import os
import time
import queue
import signal
#
from pymavlink import mavutil
from threading import Thread
#
import utils
from param_protocol import Params
from heartbeat_protocol import HeartBeat


os.environ['MAVLINK20'] = '1'
mavutil.set_dialect("ardupilotmega")

LOGGER = utils.create_logger("copter")


class Copter:

    def __init__(self, connection_string="udpin:0.0.0.0:14550"):
        self.connection_string = connection_string
        self.target_system = 1
        self.target_component = 1
        self.mav_connection = None
        self.armed = False
        self.MAV_RESULT_CONSTANTS = mavutil.mavlink.enums["MAV_RESULT"]

        self.set_signal_handler()
        self.connect()

        self.message_listeners = {
            "HEARTBEAT": [],
            "PARAM_VALUE": [],
            "COMMAND_ACK": dict(),
        }

        self.message_stats = dict()
        self.message_router_thread = None
        self.initialize_message_router()
        self.params = None
        self.initialize_params()
        self.heartbeat = None
        self.initialize_heartbeat()

    def initialize_message_router(self):
        self.message_router_thread = Thread(target=self.drain_mav, daemon=True)
        self.message_router_thread.start()
        self.mav_connection.message_hooks.append(self.message_router)

    def drain_mav(self):
        """
        This appears to do nothing with the messages but every time this reads in a message
          the MAV also routes that message to our message_hooks (ie message_router)
        """
        draining = True
        while draining:
            # an endless loop that keeps reading in messages
            message = self.mav_connection.recv_match(blocking=False, timeout=1)
            #time.sleep(.01)
            #break
            #if message:
                #print(message)
                #continue
                #self.message_router(message=message)
            #else:
                #continue
                #draining = False

    def message_router(self, mav_connection=None, message=None):
        if not message:
            return None
        message_type = message.get_type()
        #print(message_type)
        # if message_type in self.message_stats:
        #     m_stat = self.message_stats[message_type]
        #     m_stat['count'] += 1
        #     #m_stat['messages'].append(message)
        # else:
        #     self.message_stats[message_type] = {"count": 1}#, "messages": [message]}
        if message_type in self.message_listeners:
            message_listeners = self.message_listeners[message_type]
            if type(message_listeners) == list:
                for message_listener in message_listeners:
                    message_listener(message)
            elif type(message_listeners) == dict:
                for message_listener in message_listeners.values():
                    message_listener(message)

    def initialize_params(self):
        self.params = Params(mav_connection=self.mav_connection,
                             target_system=self.target_system,
                             target_component=self.target_component)
        self.message_listeners["PARAM_VALUE"].append(self.params.param_message_hook)
        self.params.get_all_params()

    def initialize_heartbeat(self):
        self.heartbeat = HeartBeat(mav_connection=self.mav_connection,
                                   target_system=self.target_system,
                                   target_component=self.target_component)
        #self.mav_connection.message_hooks.append(self.heartbeat.heartbeat_message_hook)
        self.message_listeners["HEARTBEAT"].append(self.heartbeat.heartbeat_message_hook)
        while not self.heartbeat.heartbeat:
            # waiting..
            time.sleep(.1)
        print("Heartbeat Initialized!")

    def connect(self):
        if self.mav_connection is None:
            self.mav_connection = mavutil.mavlink_connection(self.connection_string)
        self.start_timeout_timer()
        m = self.mav_connection.wait_heartbeat(blocking=True, timeout=30)
        self.stop_timeout_timer()
        print("Heartbeat? \n" + str(m))

    def add_logging_message_hook(self):
        self.mav_connection.message_hooks.append(self.log_message_hook)

    def log_message_hook(self, *args):
        msg = args[1]
        if msg.get_type() != "PARAM_VALUE":
            LOGGER.debug("HOOKED, YEEHAW!\t" + str(msg))

    def run_prearm_checks(self):
        # https://mavlink.io/en/services/arm_authorization.html
        # https://mavlink.io/en/messages/common.html#MAV_CMD_RUN_PREARM_CHECKS
        ack_message = self.send_long_command(command=401)
        sys_status = None
        self.start_timeout_timer(30)
        while not sys_status:
            sys_status = self.mav_connection.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
            time.sleep(.5)
        self.stop_timeout_timer()
        for k, v in sys_status.__dict__.items():
            print(k, v)

    def check_system_status(self):
        m = None
        self.start_timeout_timer()
        while not m:
            m = self.mav_connection.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
        self.stop_timeout_timer()
        for k, v in m.__dict__.items():
            print(k, v)

    def arm_vehicle(self, force=False):
        # https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
        force_arm = 0
        if force:
            force_arm = 21196
        ack_message = self.send_long_command(command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=1, p2=force_arm)
        if ack_message.result == 0:
            self.armed = True
        print(str(ack_message))

    def alt_arm_vehicle(self):
        m = self.mav_connection.arducopter_arm()
        print(m)

    def set_flight_mode(self, mode="STABILIZE"):
        LOGGER.debug("Setting flight mode: {}".format(mode))
        mode_id = self.mav_connection.mode_mapping()[mode]
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        ack_message = self.send_long_command(command=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                             p1=base_mode,
                                             p2=mode_id)
        return ack_message

    def send_long_command(self, target_system=None, target_component=None, command=None, confirmation=0,
                          p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
        target_system = target_system or self.target_system
        target_component = target_component or self.target_component
        self.start_timeout_timer(30)
        while True:
            self.mav_connection.mav.command_long_send(target_system,
                                                      target_component,
                                                      command,
                                                      confirmation,
                                                      p1, p2, p3, p4, p5, p6, p7)
            time.sleep(.5)
            ack_message = self.mav_connection.recv_match(type="COMMAND_ACK")
            if ack_message:
                if ack_message.command == command:
                    self.stop_timeout_timer()
                    if ack_message.result != 0:
                        # see mav_result options: for k, v in mavutil.mavlink.enums["MAV_RESULT"].items(): print(v.name)
                        result = self.MAV_RESULT_CONSTANTS[ack_message.result]
                        print("Result of command was not MAV_RESULT_ACCEPTED (aka 0).\nack_message: {}"\
                              "\n\tResult name: {} \n\tResult description: {}".format(ack_message, result.name, result.description))
                    else:
                        print("Received successful ack_message: {}".format(ack_message))
                    return ack_message
            else:
                # mavlink command protocol says we should increment confirmation if the command drops
                #  https://mavlink.io/en/services/command.html
                confirmation += 1

    def set_message_interval(self, message_id, frequency_hz):
        """https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL
            Set the inverval between messages for a particular MAVLink message ID.
              Set frequency_hz == -1 to disable the data stream
              Set frequency_hz == 0 to request the default stream rate
            Note: this replaces REQUEST_DATA_STREAM"""
        frequency_hz = 1e6 / frequency_hz
        ack_message = self.send_long_command(command=mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                             p1=message_id, p2=frequency_hz)
        return ack_message

    def disable_radio_failsafe(self):
        """Update a couple params to allow arming without an RC connection.
            https://ardupilot.org/copter/docs/common-gcs-only-operation.html#copter"""
        print("Disabling RC failsafes.")
        # disable the throttle failsafe param
        #self.set_param("FS_THR_ENABLE", 0)
        self.params.set_param("FS_THR_ENABLE", 0)
        # disable the 'RC Channels' bit in the ARMING_CHECK param
        # Note: 65470 is a bitmask that evaluates to: [False, True, True, True, True, True, False, True, True, True, True, True, True, True, True, True]
        #self.set_param("ARMING_CHECK", 65470)
        self.params.set_param("ARMING_CHECK", 65470)

    def simple_takeoff(self, target_altitude):
        if self.armed:
            target_altitude = float(target_altitude)
            ack_message = self.send_long_command(command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=target_altitude)
        else:
            print("Copter must be armed before it can take off.")

    def start_timeout_timer(self, timeout=5):
        # requests that a SIGALRM signal is sent after the timeout period (in seconds)
        signal.alarm(timeout)

    def stop_timeout_timer(self):
        # passing 0 to signal.alarm() disables it
        signal.alarm(0)

    def alarm_signal_handler(self, signum, frame):
        # raises a timeout exception when we receive the SIGALRM signal
        LOGGER.debug("Signal handler called with signal: " + str(signum))
        raise TimeoutError()

    def set_signal_handler(self):
        # sets self.alarm_signal_handler as the method to handle SIGALRM signals
        signal.signal(signal.SIGALRM, self.alarm_signal_handler)


class Task(object):
    """This class wraps method calls so that they can be called from within a queue."""
    def __init__(self, action, timeout=5, *args, **kwargs):
        self.action = action
        self.args = args
        self.kwargs = kwargs
        self.timeout = timeout
        self.task_completed = None

    def start_timeout_timer(self):
        signal.alarm(self.timeout)

    def stop_timeout_timer(self):
        # passing 0 to signal.alarm() disables it
        signal.alarm(0)

    def run(self):
        #timer = threading.Timer(self.timeout, raise_exception).start()
        self.start_timeout_timer()
        try:
            self.action(self.args, self.kwargs)
            self.stop_timeout_timer()
            self.task_completed = True
        except TimeoutError:
            self.task_completed = False
        return self.task_completed


class Error(Exception):
    """Base class for Copter exceptions."""
    pass

class TimeoutError(Error):
    """Exception raised when a timeout occurs."""
    def __init__(self, message="A Timeout Error occurred: "):
        LOGGER.error(message)


def big_print(text):
    print("######")
    print("############### {}  ###############".format(text))
    print("######")

def get_mav_cmd(key=None, name=None, name_contains=None):
    command = None
    possible_commands = []
    if key:
        command = mavutil.mavlink.enums["MAV_CMD"][key.upper()]
    elif name or name_contains:
        for k, v in mavutil.mavlink.enums["MAV_CMD"].items():
            if name:
                if v.name == name.upper():
                    command = v
        if not command:
            searching_for = name_contains or name
            for k, v in mavutil.mavlink.enums["MAV_CMD"].items():
                if searching_for.upper() in v.name:
                    possible_commands.append(v)
    if command:
        print("command.name \t {}".format(command.name))
        print("command.dcription \t {}".format(command.description))
        print("command.param \t {}".format(str(command.param)))
    else:
        for command in possible_commands:
            print("command.name \t {}".format(command.name))
            print("command.dcription \t {}".format(command.description))
            print("command.param \t {}".format(str(command.param)))




if __name__ == "__main__":
    #copter = Copter("udpin:192.168.1.17:14550")
    copter = Copter()
    big_print("Connecting to Copter")
    copter.connect()
    #copter.initialize_message_router()
    #copter.initialize_params()
    #copter.post_initialization()
    #time.sleep(10)
    # copter.add_logging_message_hook()
    # big_print("Setting Flight Mode")
    # copter.set_flight_mode("GUIDED")
    # big_print("Getting all Params")
    # #copter.get_all_params()
    # # example_param_to_set = "VTX_ENABLE"
    # # #big_print("Setting a parameter: {}".format(example_param_to_set))
    # # #copter.set_param(example_param_to_set, 1)
    # # big_print("Setting some message interval")
    # # copter.set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 3)
    # copter.disable_radio_failsafe()
    # #big_print("Checking system status.")
    # #copter.run_prearm_checks()
    # big_print("Arming Vehicle")
    # copter.arm_vehicle(force=False)
    # #copter.alt_arm_vehicle()
    # big_print("Taking off!")
    # copter.simple_takeoff(10)

