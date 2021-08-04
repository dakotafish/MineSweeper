import signal
import utils
from pymavlink import mavutil

# https://mavlink.io/en/services/heartbeat.html

LOGGER = utils.create_logger(log_name="HEARTBEAT")

class HeartBeatMessage(object):

    def __init__(self, message):
        assert message.name == "HEARTBEAT"
        self.type = message.type
        self.autopilot = message.autopilot
        self.base_mode = message.base_mode
        self.custom_mode = message.custom_mode
        self.system_status = message.system_status
        self.mavlink_version = message.mavlink_version
        self.raw_message = message

class HeartBeat(object):
    """
    message_router will route HEARTBEAT messages to the heartbeat_message_hook.
    -- We send a HEARTBEAT and set a timer for 1 second.
    -- If we receive a heartbeat back before the timer goes off then we reset the timer and send another heartbeat.
    -- If the timer goes off before we receive a heartbeat back then we increment a counter and send another heartbeat.
    -- If the counter goes over 6 missed heartbeats then we've lost contact and should throw an exception.
    MAVLink docs: https://mavlink.io/en/services/heartbeat.html
    """
    def __init__(self, mav_connection=None, target_system=0, target_component=0, *args, **kwargs):
        self.mav_connection = mav_connection
        self.target_system = target_system
        self.target_component = target_component
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
            self.mav_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                                   mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                                   self.heartbeat.base_mode,
                                                   self.heartbeat.custom_mode,
                                                   self.heartbeat.system_status)
        else:
            self.mav_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
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


