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
import uuid
#from param_protocol import Params
#from heartbeat_protocol import HeartBeat
#from command_protocol import CommandQueue, CommandLong, CommandInt

import math

os.environ['MAVLINK20'] = '1'
mavutil.set_dialect("ardupilotmega")

LOGGER = utils.create_logger("COPTER")
MAV_CMDS = mavutil.mavlink.enums["MAV_CMD"]
MAV_RESULTS = mavutil.mavlink.enums["MAV_RESULT"]
MAV_FRAMES = {v.name: k for k, v in mavutil.mavlink.enums['MAV_FRAME'].items()}


class Copter:

    def __init__(self, connection_string="udpin:0.0.0.0:14550"):
        self.connection_string = connection_string
        self.target_system = 1
        self.target_component = 1
        self.mav = mavutil.mavlink_connection(self.connection_string, autoreconnect=True)
        self.mav.wait_heartbeat(timeout=30)
        # message router
        self.message_router = MessageRouter(copter=self)
        self.mav.message_hooks.append(self.message_router.router)
        self.message_router.start()
        # outbound message queue
        self.outbound_q = OutboundMessageQueue(copter=self)
        # heartbeat
        self.heartbeat = HeartBeat(copter=self)
        self.message_router.message_listeners["HEARTBEAT"].append(self.heartbeat.heartbeat_message_hook)
        # params
        self.params = Params(copter=self)
        self.message_router.message_listeners["PARAM_VALUE"].append(self.params.param_message_hook)
        self.params.get_all_params()

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

    def set_flight_mode(self, mode="STABALIZE", send=True):
        mode_id = self.mav.mode_mapping()[mode]
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        set_flight_mode_command = CommandLong(mav=self.mav, command_id=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
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
        }

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
                self.mav.recv_match(blocking=False)
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
            # if the thread isn't defined or if it was been stopped then just re-initialize and start it
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
            command.send_command()
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
            command.send_command()
            while not command.command_complete:
                time.sleep(.01)
            # once the command completes remove it from the message_listeners
            del self.copter.message_router.message_listeners["COMMAND_ACK"][command.uuid]
            self.queue.task_done()

    def put(self, command):
        self.queue.put(command)


class HeartBeatMessage(object):
    """
    A wrapper for heartbeat messages.
    """

    def __init__(self, message):
        assert message.name == "HEARTBEAT"
        self.type = message.type
        self.autopilot = message.autopilot
        self.base_mode = message.base_mode
        self.custom_mode = message.custom_mode
        self.system_status = message.system_status
        self.mavlink_version = message.mavlink_version
        self.raw_message = message

    def __str__(self):
        return str(self.raw_message)


class HeartBeat(object):
    """
    HeartBeat manages the heartbeat connection with the vehicle.
    message_router will route HEARTBEAT messages to the heartbeat_message_hook.
    -- We send a HEARTBEAT and set a timer for 1 second.
    -- If we receive a heartbeat back before the timer goes off then we reset the timer and send another heartbeat.
    -- If the timer goes off before we receive a heartbeat back then we increment a counter and send another heartbeat.
    -- If the counter goes over 6 missed heartbeats then we've lost contact and should throw an exception.
    MAVLink docs: https://mavlink.io/en/services/heartbeat.html
    """
    def __init__(self, copter, *args, **kwargs):
        self.mav = copter.mav
        self.target_system = copter.target_system
        self.target_component = copter.target_component
        self.frequency = 1
        self.missed_heartbeats = 0
        self.heartbeat = None
        signal.signal(signal.SIGALRM, self.alarm_signal_handler)

    def heartbeat_message_hook(self, message):
        # if we receive a HEARTBEAT then stop our timer, reset the missed heartbeat count, and send a fresh heartbeat
        assert message.get_type() == "HEARTBEAT"
        self.stop_timer()
        self.missed_heartbeats = 0
        self.send_heartbeat()
        self.heartbeat = HeartBeatMessage(message)
        # you'll need this later.. bit_map = utils.bit_mapper(message.base_mode, debug=True)

    def send_heartbeat(self):
        self.start_timer(self.frequency)
        if self.heartbeat:
            # this might be better:
            self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                                   mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                                   self.heartbeat.base_mode,
                                                   self.heartbeat.custom_mode,
                                                   self.heartbeat.system_status)
        else:
            self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                                   mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                                   0, 0, 0)

    def start_timer(self, timeout):
        # requests that a SIGALRM signal is sent after the timeout period (in seconds)
        signal.alarm(timeout)

    def stop_timer(self):
        # passing 0 to signal.alarm() disables it
        signal.alarm(0)

    def alarm_signal_handler(self, signum, frame):
        # if our timer goes off then we missed a heartbeat, increment the count, send a new heartbeat
        #  and throw an exception if we miss six heartbeats in a row
        self.missed_heartbeats += 1
        if self.missed_heartbeats >= 6:
            LOGGER.error("HEARTBEAT: MISSED 6 IN A ROW.")
            raise Exception
        self.send_heartbeat()


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


class MavCommand(object):
    """
    Base MavCommand class - a wrapper for MAV_CMD's: https://mavlink.io/en/messages/common.html#mav_commands
    Should be subclassed by either:
        CommandInt() - where params 5 & 6 are floats
        or
        CommandLong() - where all params are ints
    """

    # https://mavlink.io/en/messages/common.html#mav_commands
    def __init__(self, mav, command_id, frame=None, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0,
                 target_system=0, target_component=0):
        self.uuid = uuid.uuid4().hex
        self.mav = mav
        self.frame = frame
        self.current = 0  # always zero
        self.autocontinue = 0  # always zero
        self.confirmation = 0
        self.target_system = target_system
        self.target_component = target_component
        if command_id in MAV_CMDS:
            self.command_id = command_id
            command = MAV_CMDS[command_id]
            self.command_name = command.name
            self.command_description = command.description
            assert len(command.param) == 7
            param_descriptions = command.param
            self.p1 = float(0 or p1)
            self.p1_description = param_descriptions[1]
            self.p2 = float(0 or p2)
            self.p2_description = param_descriptions[2]
            self.p3 = float(0 or p3)
            self.p3_description = param_descriptions[3]
            self.p4 = float(0 or p4)
            self.p4_description = param_descriptions[4]
            self.p5 = 0 or p5
            self.p5_description = param_descriptions[5]
            self.p6 = 0 or p6
            self.p6_description = param_descriptions[6]
            self.p7 = float(0 or p7)
            self.p7_description = param_descriptions[7]
        else:
            print("No MAV_CMD found for command_id {}.".format(command_id))

        #self.thread = Thread(target=self.send_command_worker, daemon=False)
        self.command_complete = False
        self.is_command_complete = lambda: self.command_complete
        self.command_send_method = None
        self.command_send_args = None
        self.set_command_send_method()

    def set_command_send_method(self):
        raise NotImplementedError

    def send_command(self):
        self.command_send_method(*self.command_send_args())

    def command_message_hook(self, ack_message):
        assert ack_message.get_type() == "COMMAND_ACK"
        if ack_message and ack_message.command == self.command_id:
            mav_result = MAV_RESULTS[ack_message.result]
            print(mav_result.name, mav_result.description)
            if mav_result.name == "MAV_RESULT_ACCEPTED":
                print("Command Completed: {}".format(str(ack_message)))
                self.command_complete = True
                return ack_message
            elif mav_result.name == "MAV_RESULT_TEMPORARILY_REJECTED":
                # most likely just need to wait for some other command to finish
                time.sleep(1)
                self.send_command()
            elif mav_result.name == "MAV_RESULT_DENIED":
                # Command is invalid (is supported but has invalid parameters).
                # Retrying same command and parameters will not work.
                self.command_complete = True
                # TODO need better exceptions here
                raise Exception
            elif mav_result.name == "MAV_RESULT_FAILED":
                self.command_complete = True
                # TODO need better exceptions here
                raise Exception
            elif mav_result.name == "MAV_RESULT_IN_PROGRESS":
                # Per the MavLink documentation we just need to increase the timeout and wait
                # We don't need to re-send the command and if we do we'll just get another progress message
                print("Command in Progress. Currently at {}% complete.".format(ack_message.progress))
                time.sleep(1)
        else:
            # if the command drops for some reason then we just increment confirmation and re-send it
            self.confirmation += 1
            self.send_command()


class CommandInt(MavCommand):
    """
    https://mavlink.io/en/messages/common.html#COMMAND_INT
    """
    def __init__(self, mav, command_id, frame, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0,
                 target_system=0, target_component=0):
        super(MavCommand, self).__init__(mav,
                                         command_id,
                                         frame=frame,
                                         p1=p1, p2=p2, p3=p3, p4=p4, p5=p5, p6=p6, p7=p7,
                                         target_system=target_system,
                                         target_component=target_component)
        self.p5 = int(self.p5)
        self.p6 = int(self.p6)

    def set_command_send_method(self):
        self.command_send_method = self.mav.mav.command_int_send
        self.command_send_args = lambda: (self.target_system, self.target_component, self.frame, self.command_id,
                                          self.current, self.autocontinue, self.p1, self.p2, self.p3, self.p4, self.p5,
                                          self.p6, self.p7)


class CommandLong(MavCommand):
    """
    https://mavlink.io/en/messages/common.html#COMMAND_INT
    """
    def __init__(self, mav, command_id, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0, target_system=0, target_component=0):
        super(MavCommand, self).__init__(mav,
                                         command_id,
                                         p1=p1, p2=p2, p3=p3, p4=p4, p5=p5, p6=p6, p7=p7,
                                         target_system=target_system,
                                         target_component=target_component)
        self.p5 = float(self.p5)
        self.p6 = float(self.p6)

    def set_command_send_method(self):
        self.command_send_method = self.mav.mav.command_int_send
        self.command_send_args = lambda: (self.target_system, self.target_component, self.command_id, self.confirmation,
                                          self.p1, self.p2, self.p3, self.p4, self.p5, self.p6, self.p7)


class MissionItemInt(MavCommand):
    """
    https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT
    """
    pass



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
    #copter.add_message_hook(copter.location_logging_message_hook)
    utils.big_print("Disabling Radio Failsafe")
    copter.disable_radio_failsafe()
    utils.big_print("Adding Set Flight Mode Command to Command Queue")
    copter.set_flight_mode("GUIDED", send=False)
    utils.big_print("Adding Set Home Position Command to Command Queue")
    #copter.set_home_position(send=False)
    utils.big_print("Adding Arm Vehicle Command to Command Queue")
    copter.arm(force=False, send=False)
    utils.big_print("Adding Takeoff Command to Command Queue and Sending all commands.")
    copter.simple_takeoff(target_altitude=20, send=False)
    copter.simple_waypoint(lat=376193729, lon=-1223766375, alt=30, send=True)