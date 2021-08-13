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
from command_protocol import CommandQueue, CommandLong, CommandInt

import math

os.environ['MAVLINK20'] = '1'
mavutil.set_dialect("ardupilotmega")

LOGGER = utils.create_logger("COPTER")
MAV_RESULTS = mavutil.mavlink.enums["MAV_RESULT"]
MAV_FRAMES = {v.name: k for k, v in mavutil.mavlink.enums['MAV_FRAME'].items()}


class Copter:

    def __init__(self, connection_string="udpin:0.0.0.0:14550"):
        self.connection_string = connection_string
        self.target_system = 1
        self.target_component = 1
        self.mav_connection = None
        self.armed = False

        self.set_signal_handler()
        self.connect()

        self.log_all_messages = False
        if self.log_all_messages:
            self.mav_connection.message_hooks.append(self.logging_message_hook)

        self.message_listeners = {
            "HEARTBEAT": [],
            "PARAM_VALUE": [],
            "COMMAND_ACK": dict(),
        }
        self.message_stats = dict()
        self.message_router_thread = None
        self.initialize_message_router()
        self.heartbeat = None
        self.initialize_heartbeat()
        self.params = None
        self.initialize_params()
        self.command_queue = None
        self.initialize_command_queue()
        time.sleep(10)

    def initialize_message_router(self):
        self.message_router_thread = Thread(target=self.drain_mav, daemon=True)
        self.message_router_thread.start()
        self.mav_connection.message_hooks.append(self.message_router)

    def drain_mav(self):
        """
        This appears to do nothing with the messages but every time this reads in a message
          the MAV also routes that message to our message_hooks (ie message_router)
        """
        self.draining = True
        while self.draining:
            # an endless loop that keeps reading in messages
            # each time we read a message it is also sent to each of the mav_connection.message_hooks
            # Note: The only real message_hook that we install (as of now) is self.message_router()
            message = self.mav_connection.recv_match(blocking=False, timeout=1)
            time.sleep(.01)

    def message_router(self, mav_connection=None, message=None):
        if not message:
            return None
        self.add_to_message_stats(message)
        message_type = message.get_type()
        if message_type in self.message_listeners:
            message_listeners = self.message_listeners[message_type]
            if type(message_listeners) == list:
                for message_listener in message_listeners:
                    message_listener(message)
            elif type(message_listeners) == dict:
                for message_listener in message_listeners.values():
                    message_listener(message)

    def add_to_message_stats(self, message):
        message_type = message.get_type()
        if message_type in self.message_stats:
            self.message_stats[message_type] += 1
        else:
            self.message_stats[message_type] = 1

    def initialize_heartbeat(self):
        self.heartbeat = HeartBeat(mav_connection=self.mav_connection,
                                   target_system=self.target_system,
                                   target_component=self.target_component)
        self.message_listeners["HEARTBEAT"].append(self.heartbeat.heartbeat_message_hook)
        while not self.heartbeat.heartbeat:
            # waiting..
            time.sleep(.5)
        print("Heartbeat Initialized!")

    def initialize_params(self):
        self.params = Params(mav_connection=self.mav_connection,
                             target_system=self.target_system,
                             target_component=self.target_component)
        self.message_listeners["PARAM_VALUE"].append(self.params.param_message_hook)
        self.params.get_all_params()

    def initialize_command_queue(self):
        self.command_queue = CommandQueue(self)

    def add_message_hook(self, some_function):
        self.mav_connection.message_hooks.append(some_function)

    def logging_message_hook(self, mav_connection, message):
        print(message)
        LOGGER.debug("MESSAGE_HOOK_LOGGER: {}".format(str(message)))

    def location_logging_message_hook(self, mav_connection, message):
        types_to_log = ["GPS_RAW_INT", "GLOBAL_POSITION_INT",] #"VFR_HUD"]
        if message.get_type() in types_to_log:
            raw_lat = message.lat
            lat = raw_lat * 1.0e-7
            rad_lat = math.radians(raw_lat) * 1.0e-7

            raw_lon = message.lon
            lon = raw_lon * 1.0e-7
            rad_lon = math.radians(raw_lon) * 1.0e-7

            raw_alt = message.alt
            alt = raw_alt * 0.001
            log_string = "RAW_MESSAGE: {}".format(message)
            log_string += "\n\tRaw_Lat: {} \t Lat: {} \t Rad_Lat: {}".format(raw_lat, lat, rad_lat)
            log_string += "\n\tRaw_Lon: {} \t Lon: {} \t Rad_Lon: {}".format(raw_lon, lon, rad_lon)
            log_string += "\n\tRaw_Alt: {} \t Alt: {}".format(raw_alt, alt)
            #print(log_string)
            LOGGER.debug(log_string)

    ### State Commands ###

    def connect(self):
        if self.mav_connection is None:
            self.mav_connection = mavutil.mavlink_connection(self.connection_string)
        self.start_timeout_timer(30)
        print("Waiting for Heartbeat...")
        m = self.mav_connection.wait_heartbeat(blocking=True, timeout=30)
        self.stop_timeout_timer()
        print(str(m))

    def arm_vehicle(self, force=False, send=True):
        # https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
        force_arm = 0
        if force:
            force_arm = 21196
        arm_command = CommandLong(mav_connection=self.mav_connection,
                                 command_id=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                 p1=1,
                                 p2=force_arm)
        self.command_queue.put(arm_command)
        self.armed = True
        if send:
            self.command_queue.send_commands()

    def set_flight_mode(self, mode="STABALIZE", send=True):
        LOGGER.debug("Setting flight mode: {}".format(mode))
        mode_id = self.mav_connection.mode_mapping()[mode]
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        set_flight_mode_command = CommandLong(mav_connection=self.mav_connection,
                                             command_id=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                             p1=base_mode,
                                             p2=mode_id)
        self.command_queue.put(set_flight_mode_command)
        if send:
            self.command_queue.send_commands()

    def set_message_interval(self, message_id, frequency_hz, send=True):
        """https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL
            Set the inverval between messages for a particular MAVLink message ID.
              Set frequency_hz == -1 to disable the data stream
              Set frequency_hz == 0 to request the default stream rate
            Note: this replaces REQUEST_DATA_STREAM"""
        frequency_hz = 1e6 / frequency_hz
        set_message_interval_command = CommandLong(mav_connection=self.mav_connection,
                                                  command_id=mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                                  p1=message_id,
                                                  p2=frequency_hz)
        self.command_queue.put(set_message_interval_command)
        if send:
            self.command_queue.send_commands()

    def disable_radio_failsafe(self):
        """Update a couple params to allow arming without an RC connection.
            https://ardupilot.org/copter/docs/common-gcs-only-operation.html#copter"""
        print("Disabling RC failsafes.")
        # disable the throttle failsafe param
        self.params.set_param("FS_THR_ENABLE", 0)
        # disable the 'RC Channels' bit in the ARMING_CHECK param
        # Note: 65470 is a bitmask that evaluates to: [False, True, True, True, True, True, False, True, True, True, True, True, True, True, True, True]
        self.params.set_param("ARMING_CHECK", 65470)

    def set_home_position(self, send=True):
        set_home_position_command = CommandLong(mav_connection=self.mav_connection,
                                               command_id=mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                                               p1=1)
        self.command_queue.put(set_home_position_command)
        if send:
            self.command_queue.send_commands()

    ### NAV Commands ###

    def simple_takeoff(self, target_altitude, send=True):
        if self.armed:
            target_altitude = float(target_altitude)
            simple_takeoff_command = CommandLong(mav_connection=self.mav_connection,
                                                command_id=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                                p7=target_altitude)
            self.command_queue.put(simple_takeoff_command)
            if send:
                self.command_queue.send_commands()
        else:
            print("Copter must be armed before it can take off.")

    def simple_land(self, send=True):
        simple_land_command = CommandLong(mav_connection=self.mav_connection,
                                         command_id=mavutil.mavlink.MAV_CMD_NAV_LAND)
        self.command_queue.put(simple_land_command)
        if send:
            self.command_queue.send_commands()

    def simple_waypoint(self,
                        hold_time=1,
                        accept_radius=1,
                        pass_radius=0,
                        yaw=None,
                        lat=None,
                        lon=None,
                        alt=None,
                        send=True):
        assert lat and lon and alt
        #lat = lat * 1E7
        #lon = lon * 1E7
        simple_waypoint_command = CommandInt(mav_connection=self.mav_connection,
                                             command_id=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                             frame=MAV_FRAMES['MAV_FRAME_GLOBAL_RELATIVE_ALT'],
                                             p1=hold_time,
                                             p2=accept_radius,
                                             p3=pass_radius,
                                             p4=0,
                                             p5=lat,
                                             p6=lon,
                                             p7=alt)
        self.command_queue.put(simple_waypoint_command)
        if send:
            self.command_queue.send_commands()

    ### Timeout Signals and Handlers ###

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

class Location(object):
    def __init__(self, lat, lon, alt, init_type, alt_relative=True):
        if init_type == "RADIANS":
            self.radians_lat = lat
            self.radians_lon = lon
            self.radians_alt = alt
        elif init_type == "DEGREES":
            self.degrees_lat = lat
            self.degrees_lon = lon
            self.degrees_alt = alt
        elif init_type == "ENCODED":
            self.encoded_lat = lat
            self.encoded_lon = lon
            self.encoded_alt = alt

        # alt needs to be * 1E4 at some point.. https://mavlink.io/en/services/mission.html

    def finish_init_with_encoded_params(self):
        self.degrees_lat = self.encoded_lat * 1.0e-7
        self.radians_lat = math.radians(self.degrees_lat)
        self.degrees_lon = self.encoded_lon * 1.0e-7
        self.radians_lon = math.radians(self.degrees_lon)
        self.degrees_alt = self.encoded_alt * 1.0e-7
        self.radians_alt = self.degrees_alt



class Error(Exception):
    """Base class for Copter exceptions."""
    pass

class TimeoutError(Error):
    """Exception raised when a timeout occurs."""
    def __init__(self, message="A Timeout Error occurred: "):
        LOGGER.error(message)



if __name__ == "__main__":
    #copter = Copter("udpin:192.168.1.17:14550")
    utils.big_print("Connecting to Copter")
    copter = Copter()
    copter.add_message_hook(copter.location_logging_message_hook)
    utils.big_print("Disabling Radio Failsafe")
    copter.disable_radio_failsafe()
    utils.big_print("Adding Set Flight Mode Command to Command Queue")
    copter.set_flight_mode("GUIDED", send=False)
    utils.big_print("Adding Set Home Position Command to Command Queue")
    #copter.set_home_position(send=False)
    utils.big_print("Adding Arm Vehicle Command to Command Queue")
    copter.arm_vehicle(force=False, send=False)
    utils.big_print("Adding Takeoff Command to Command Queue and Sending all commands.")
    copter.simple_takeoff(target_altitude=20, send=False)
    copter.simple_waypoint(lat=376193729, lon=-1223766375, alt=30, send=True)