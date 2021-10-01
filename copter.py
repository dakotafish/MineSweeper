import logging
import geopy
import os
import time
import queue
import signal
#
from geopy.distance import distance as geopy_distance
from pymavlink import mavutil
from threading import Thread
#
import utils
from param_protocol import Params
from heartbeat_protocol import HeartBeat
from command_protocol import CommandInt, CommandLong, MissionItemInt
from mission_protocol import Navigation

# os.environ['MAVLINK20'] = '1'
# mavutil.set_dialect("ardupilotmega")

LOGGER = utils.create_logger("COPTER")
MAV_CMDS = mavutil.mavlink.enums["MAV_CMD"]
MAV_RESULTS = mavutil.mavlink.enums["MAV_RESULT"]
MAV_FRAMES = {v.name: k for k, v in mavutil.mavlink.enums['MAV_FRAME'].items()}
MAV_MISSION_TYPES = {v.name: k for k, v in mavutil.mavlink.enums['MAV_MISSION_TYPE'].items()}
MAV_MISSION_RESULTS = mavutil.mavlink.enums["MAV_MISSION_RESULT"]


class Copter:

    def __init__(self, connection_string="udpin:0.0.0.0:14550"):
        self.connection_string = connection_string
        self.target_system = 1
        self.target_component = 1
        self.mav = mavutil.mavlink_connection(self.connection_string) #, autoreconnect=True)
        self.mav.wait_heartbeat(blocking=True, timeout=30)
        # 1 message router
        self.message_router = MessageRouter(copter=self)
        self.mav.message_hooks.append(self.message_router.router)
        self.message_router.start()
        # 2 outbound message queue
        self.outbound_q = OutboundMessageQueue(copter=self)
        # 3 heartbeat
        self.heartbeat = HeartBeat(copter=self)
        self.message_router.message_listeners["HEARTBEAT"].append(self.heartbeat.heartbeat_message_hook)
        # 4 params
        self.params = Params(copter=self)
        self.message_router.message_listeners["PARAM_VALUE"].append(self.params.param_message_hook)
        self.params.get_all_params()
        # 5 nav
        self.nav = Navigation(copter=self)
        self.message_router.message_listeners["MISSION_REQUEST_INT"].append(self.nav.mission_request_int_message_hook)
        self.message_router.message_listeners["MISSION_REQUEST"].append(self.nav.mission_request_int_message_hook)
        self.message_router.message_listeners["MISSION_ACK"].append(self.nav.mission_upload_ack_message_hook)
        #self.message_router.message_listeners["MISSION_ITEM_REACHED"].append(self.nav.mission_progress_message_hook)
        #self.message_router.message_listeners["MISSION_CURRENT"].append(self.nav.mission_progress_message_hook)


    def arm(self, force=False, send=True):
        # https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
        force_arm = 0
        if force:
            force_arm = 21196
        arm_command = CommandLong(mav=self.mav, command_id=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                  p1=1, p2=force_arm)
        self.outbound_q.put(arm_command)
        if send:
            self.outbound_q.send_all()

    def run_prearm_checks(self, send=True):
        prearm_command = CommandLong(self.mav, 401)
        self.outbound_q.put(prearm_command)
        if send:
            self.outbound_q.send_all()


    def set_flight_mode(self, mode="STABALIZE", send=True):
        mode_id = self.mav.mode_mapping()[mode]
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        set_flight_mode_command = CommandLong(self.mav,
                                              mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                              p1=base_mode, p2=mode_id)
        self.outbound_q.put(set_flight_mode_command)
        if send:
            self.outbound_q.send_all()

    def set_message_interval(self, message_id, frequency_hz, send=True):
        """https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL
            Set the inverval between messages for a particular MAVLink message ID.
              Set frequency_hz == -1 to disable the data stream
              Set frequency_hz == 0 to request the default stream rate
            Note: this replaces REQUEST_DATA_STREAM"""
        frequency_hz = 1e6 / frequency_hz
        set_message_interval_command = CommandLong(mav=self.mav,
                                                   command_id=mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                                   p1=message_id, p2=frequency_hz)
        self.outbound_q.put(set_message_interval_command)
        if send:
            self.outbound_q.send_all()

    def request_data_stream(self, stream_id, rate=1, start_stop=1):
        self.mav.mav.request_data_stream_send(self.target_system,
                                              self.target_component,
                                              stream_id,
                                              rate,
                                              start_stop)


    def disable_radio_failsafe(self):
        """Update a couple params to allow arming without an RC connection.
            https://ardupilot.org/copter/docs/common-gcs-only-operation.html#copter"""
        # disable the throttle failsafe param
        self.params.set_param("FS_THR_ENABLE", 0)
        # disable the 'RC Channels' bit in the ARMING_CHECK param
        # Note: 65470 is a bitmask that evaluates to:
        #   [False, True, True, True, True, True, False, True, True, True, True, True, True, True, True, True]
        self.params.set_param("ARMING_CHECK", 65470)

    def set_home_position(self, send=True):
        set_home_position_command = CommandLong(mav=self.mav, command_id=mavutil.mavlink.MAV_CMD_DO_SET_HOME, p1=1)
        self.outbound_q.put(set_home_position_command)
        if send:
            self.outbound_q.send_all()

    def simple_takeoff(self, target_altitude, send=True):
        target_altitude = float(target_altitude)
        simple_takeoff_command = CommandLong(mav=self.mav, command_id=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                             p7=target_altitude)
        self.outbound_q.put(simple_takeoff_command)
        if send:
            self.outbound_q.send_all()

    def simple_land(self, send=True):
        simple_land_command = CommandLong(mav=self.mav, command_id=mavutil.mavlink.MAV_CMD_NAV_LAND)
        self.outbound_q.put(simple_land_command)
        if send:
            self.outbound_q.send_all()

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
        simple_waypoint_command = CommandInt(mav=self.mav,
                                             command_id=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                             frame=MAV_FRAMES['MAV_FRAME_GLOBAL_RELATIVE_ALT'],
                                             p1=hold_time,
                                             p2=accept_radius,
                                             p3=pass_radius,
                                             p4=0,
                                             p5=lat,
                                             p6=lon,
                                             p7=alt)
        self.outbound_q.put(simple_waypoint_command)
        if send:
            self.outbound_q.send_all()


class MessageRouter(object):

    def __init__(self, copter):
        self.copter = copter
        self.mav = copter.mav
        self.message_stats = dict()
        self.draining = None
        self.pause_draining = None
        self.mav_drain_thread = None
        self.message_listeners = {
            "HEARTBEAT": [],
            "PARAM_VALUE": [],
            "COMMAND_ACK": dict(),
            "MISSION_ACK": [],
            "MISSION_REQUEST_INT": [],
            "MISSION_REQUEST": [],
            "MISSION_ITEM_REACHED": dict(),
            "MISSION_CURRENT": dict(),
        }
        self.ad_hoc_log = []
        self.log_all_messages = True

    def message_listeners_contains(self, message_type, key=None):
        # Return True if message_type [and optionally key] are in the message_listeners.
        if message_type not in self.message_listeners:
            return False
        if key:
            if key not in self.message_listeners[message_type]:
                return False
        return True

    def add_to_message_listeners_list(self, message_type, message_hook):
        if message_type in self.message_listeners:
            self.message_listeners.append(message_hook)
        else:
            self.message_listeners[message_type] = [message_hook]

    def add_to_message_listeners_dict(self, message_type, key, message_hook):
        if message_type in self.message_listeners:
            self.message_listeners[message_type][key] = message_hook
        else:
            self.message_listeners[message_type] = {key: message_hook}

    def remove_from_message_listeners_dict(self, message_type, key):
        if message_type in self.message_listeners:
            if key in self.message_listeners[message_type]:
                del self.message_listeners[message_type][key]

    def mav_drain(self):
        """
        message_router and mav_drain (and start/stop/pause/resume_mav_drain) are tightly coupled.
        When mav_drain reads in a message the message is also sent to mav.message_hooks which sends it to message_router
        In python threading I/O and waiting are tasks that can be run in parallel (I think.. havent timed/tested it yet)
        mav_drain basically only reads in messages and waits so we run it in its own thread (mav_drain_thread)
        """
        self.draining = True
        self.pause_draining = False
        while self.draining:
            if not self.pause_draining:
                self.mav.recv_msg()
            time.sleep(.01)

    def router(self, mav=None, message=None):
        """
        Every time the mav reads in a message it sends the message to each of the functions in mav.message_hooks
        Instead of sending every message to multiple message_hooks (functions) this will be the only message hook
         and it will route the messages accordingly.
        """
        if not message:
            return None
        self.add_to_message_stats(message)
        message_type = message.get_type()

        if message_type in self.ad_hoc_log:
            print(message)

        if self.log_all_messages:
            if "MISSION" in message.get_type():
                LOGGER.debug(message)

        if message_type in self.message_listeners:
            message_listeners = self.message_listeners[message_type]
            if type(message_listeners) == dict:
                message_listeners = message_listeners.values()
            for message_listener in message_listeners:
                # send the message to each of the relevant message_listener functions
                message_listener(message)

    def start(self):
        if self.mav_drain_thread and self.mav_drain_thread.is_alive():
            # thread is already running
            return True
        else:
            # if the thread isn't defined or if it was been stopped then just re-define and start it
            self.mav_drain_thread = Thread(target=self.mav_drain, daemon=True)
            self.mav_drain_thread.start()

    def stop(self):
        self.draining = False

    def pause(self):
        self.pause_draining = True

    def resume(self):
        self.pause_draining = False

    def add_to_message_stats(self, message):
        message_type = message.get_type()
        if message_type in self.message_stats:
            self.message_stats[message_type] += 1
        else:
            self.message_stats[message_type] = 1


class OutboundMessageQueue(object):
    """
    Queue to hold and send all outbound messages.
    """
    def __init__(self, copter):
        self.copter = copter
        self.queue = queue.Queue()

    def send_all(self):
        while not self.queue.empty():
            command = self.queue.get(block=False, timeout=1)
            # add the commands message_hook to copter.message_router.message_listeners
            self.copter.message_router.message_listeners["COMMAND_ACK"][command.uuid] = command.command_message_hook
            command.send()
            while not command.command_complete:
                time.sleep(.01)
            # once the command completes remove it from the message_listeners
            del self.copter.message_router.message_listeners["COMMAND_ACK"][command.uuid]
            self.queue.task_done()

    def send_next(self):
        if not self.queue.empty():
            command = self.queue.get(block=False, timeout=1)
            # add the commands message_hook to copter.message_listeners
            self.copter.message_router.message_listeners["COMMAND_ACK"][command.uuid] = command.command_message_hook
            command.send()
            while not command.command_complete:
                time.sleep(.01)
            # once the command completes remove it from the message_listeners
            del self.copter.message_router.message_listeners["COMMAND_ACK"][command.uuid]
            self.queue.task_done()

    def put(self, command):
        self.queue.put(command)





# class Location(object):
#     def __init__(self, lat, lon, alt, init_type, alt_relative=True):
#         if init_type == "RADIANS":
#             self.radians_lat = lat
#             self.radians_lon = lon
#             self.radians_alt = alt
#         elif init_type == "DEGREES":
#             self.degrees_lat = lat
#             self.degrees_lon = lon
#             self.degrees_alt = alt
#         elif init_type == "ENCODED":
#             self.encoded_lat = lat
#             self.encoded_lon = lon
#             self.encoded_alt = alt
#
#         # alt needs to be * 1E4 at some point.. https://mavlink.io/en/services/mission.html
#
#     def finish_init_with_encoded_params(self):
#         self.degrees_lat = self.encoded_lat * 1.0e-7
#         self.radians_lat = math.radians(self.degrees_lat)
#         self.degrees_lon = self.encoded_lon * 1.0e-7
#         self.radians_lon = math.radians(self.degrees_lon)
#         self.degrees_alt = self.encoded_alt * 1.0e-7
#         self.radians_alt = self.degrees_alt


    # def location_logging_message_hook(self, mav_connection, message):
    #     types_to_log = ["GPS_RAW_INT", "GLOBAL_POSITION_INT",] #"VFR_HUD"]
    #     if message.get_type() in types_to_log:
    #         raw_lat = message.lat
    #         lat = raw_lat * 1.0e-7
    #         rad_lat = math.radians(raw_lat) * 1.0e-7
    #
    #         raw_lon = message.lon
    #         lon = raw_lon * 1.0e-7
    #         rad_lon = math.radians(raw_lon) * 1.0e-7
    #
    #         raw_alt = message.alt
    #         alt = raw_alt * 0.001
    #         log_string = "RAW_MESSAGE: {}".format(message)
    #         log_string += "\n\tRaw_Lat: {} \t Lat: {} \t Rad_Lat: {}".format(raw_lat, lat, rad_lat)
    #         log_string += "\n\tRaw_Lon: {} \t Lon: {} \t Rad_Lon: {}".format(raw_lon, lon, rad_lon)
    #         log_string += "\n\tRaw_Alt: {} \t Alt: {}".format(raw_alt, alt)
    #         #print(log_string)
    #         LOGGER.debug(log_string)


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
    # #copter.add_message_hook(copter.location_logging_message_hook)
    # utils.big_print("Requesting data stream of vehicle position.")
    # copter.request_data_stream(mavutil.mavlink.MAV_DATA_STREAM_POSITION)
    # utils.big_print("Disabling Radio Failsafe")
    # copter.disable_radio_failsafe()
    # utils.big_print("Adding Set Flight Mode Command to Command Queue")
    # copter.set_flight_mode("GUIDED", send=True)
    # utils.big_print("Adding Set Home Position Command to Command Queue")
    # #copter.set_home_position(send=False)
    # utils.big_print("Adding Arm Vehicle Command to Command Queue")
    # copter.arm(force=False, send=True)
    # utils.big_print("Adding Takeoff Command to Command Queue and Sending all commands.")
    # copter.simple_takeoff(target_altitude=20, send=True)
    # copter.set_flight_mode("AUTO", send=True)
    # #copter.simple_waypoint(lat=376193729, lon=-1223766375, alt=30, send=True)
    # # copter.mav.mav.request_data_stream_send(copter.target_system, copter.target_component, mavutil.mavlink.MAV_DATA_STREAM_POSITION, 1,1)