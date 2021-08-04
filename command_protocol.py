import signal
import time
import utils
from pymavlink import mavutil
from threading import Thread

# Mavlink command protocol: https://mavlink.io/en/services/command.html

MAV_CMDS = mavutil.mavlink.enums["MAV_CMD"]
MAV_RESULTS = mavutil.mavlink.enums["MAV_RESULT"]

class MavCommand(object):
    """
    Base MavCommand class - a wrapper for MAV_CMD's: https://mavlink.io/en/messages/common.html#mav_commands
    Should be subclassed by either:
        CommandInt() - where params 5 & 6 are floats
        or
        CommandLong() - where all params are ints
    """
    # https://mavlink.io/en/messages/common.html#mav_commands
    def __init__(self, mav_connection, command_id, **kwargs):
        self.mav_connection = mav_connection
        if command_id in MAV_CMDS:
            self.command_id = command_id
            command = MAV_CMDS[command_id]
            self.command_name = command.name
            self.command_description = command.description
            assert len(command.param) == 7
            param_descriptions = command.param
            self.p1 = float(0 or kwargs['p1'])
            self.p1_description = param_descriptions[1]
            self.p2 = float(0 or kwargs['p2'])
            self.p2_description = param_descriptions[2]
            self.p3 = float(0 or kwargs['p3'])
            self.p3_description = param_descriptions[3]
            self.p4 = float(0 or kwargs['p4'])
            self.p4_description = param_descriptions[4]
            self.p5 = 0 or kwargs['p5']
            self.p5_description = param_descriptions[5]
            self.p6 = 0 or kwargs['p6']
            self.p6_description = param_descriptions[6]
            self.p7 = float(0 or kwargs['p7'])
            self.p7_description = param_descriptions[7]
        else:
            print("No MAV_CMD found for command_id {}.".format(command_id))
        self.thread = Thread(target=self.send_command_worker, daemon=False)
        self.command_complete = False

    def send_command(self):
        # Certain commands can take awhile to return so run this in a new thread to do the waiting & reading
        if not self.thread.is_alive():
            self.thread.start()

    def send_command_worker(self):
        raise NotImplementedError



class CommandInt(MavCommand):
    """
    https://mavlink.io/en/messages/common.html#COMMAND_INT
    """
    def __init__(self,
                 mav_connection,
                 command_id,
                 frame,
                 p1=0,
                 p2=0,
                 p3=0,
                 p4=0,
                 p5=0,
                 p6=0,
                 p7=0,
                 target_system=0,
                 target_component=0):
        super(MavCommand, self).__init__(mav_connection, command_id, p1=p1, p2=p2, p3=p3, p4=p4, p5=p5, p6=p6, p7=p7)
        self.frame = frame
        self.target_system = target_system
        self.target_component = target_component
        self.current = 0 # always zero
        self.autocontinue = 0 # always zer0
        self.p5 = int(self.p5)
        self.p6 = int(self.p6)

    def send_command_worker(self):
        confirmation = 0
        timeout=1
        while True:
            self.mav_connection.mav.command_int_send(self.target_system,
                                                     self.target_component,
                                                     self.frame,
                                                     self.command_id,
                                                     self.current,
                                                     self.autocontinue,
                                                     self.p1, self.p2, self.p3, self.p4, self.p5, self.p6, self.p7)
            ack_message = self.mav_connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=timeout)
            if ack_message and ack_message.command == self.command_id:
                mav_result = MAV_RESULTS[ack_message.result]
                print(mav_result.name, mav_result.description)
                if mav_result.name == "MAV_RESULT_ACCEPTED":
                    self.command_complete = True
                    return ack_message
                elif mav_result.name == "MAV_RESULT_TEMPORARILY_REJECTED":
                    # most likely just need to wait for some other command to finish
                    time.sleep(1)
                elif mav_result.name == "MAV_RESULT_DENIED":
                    # Command is invalid (is supported but has invalid parameters).
                    # Retrying same command and parameters will not work.
                    self.command_complete = True
                    raise Exception
                elif mav_result.name == "MAV_RESULT_FAILED":
                    self.command_complete = True
                    raise Exception
                elif mav_result.name == "MAV_RESULT_IN_PROGRESS":
                    # Per the MavLink documentation we just need to increase the timeout and wait
                    # We don't need to re-send the command but if we do we'll just get another progress message so it
                    # doesn't hurt to just let this loop run until the status changes to MAV_RESULT_ACCEPTED
                    print("Command in Progress. Current at {}% complete.".format(ack_message.progress))
                    time.sleep(3)
            else:
                # if the command drops for some reason then we just increment confirmation and re-send it
                confirmation += 1


# def send_long_command(self, target_system=None, target_component=None, command=None, confirmation=0,
#                       p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
#     target_system = target_system or self.target_system
#     target_component = target_component or self.target_component
#     self.start_timeout_timer(30)
#     while True:
#         self.mav_connection.mav.command_long_send(target_system,
#                                                   target_component,
#                                                   command,
#                                                   confirmation,
#                                                   p1, p2, p3, p4, p5, p6, p7)
#         time.sleep(.5)
#         ack_message = self.mav_connection.recv_match(type="COMMAND_ACK")
#         if ack_message:
#             if ack_message.command == command:
#                 self.stop_timeout_timer()
#                 if ack_message.result != 0:
#                     # see mav_result options: for k, v in mavutil.mavlink.enums["MAV_RESULT"].items(): print(v.name)
#                     result = self.MAV_RESULT_CONSTANTS[ack_message.result]
#                     print("Result of command was not MAV_RESULT_ACCEPTED (aka 0).\nack_message: {}" \
#                           "\n\tResult name: {} \n\tResult description: {}".format(ack_message, result.name,
#                                                                                   result.description))
#                 else:
#                     print("Received successful ack_message: {}".format(ack_message))
#                 return ack_message
#         else:
#             # mavlink command protocol says we should increment confirmation if the command drops
#             #  https://mavlink.io/en/services/command.html
#             confirmation += 1
#
#     def set_message_interval(self, message_id, frequency_hz):
#         """https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL
#             Set the inverval between messages for a particular MAVLink message ID.
#               Set frequency_hz == -1 to disable the data stream
#               Set frequency_hz == 0 to request the default stream rate
#             Note: this replaces REQUEST_DATA_STREAM"""
#         frequency_hz = 1e6 / frequency_hz
#         ack_message = self.send_long_command(command=mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
#                                              p1=message_id, p2=frequency_hz)
#         return ack_message
#
#     def set_flight_mode(self, mode="STABILIZE"):
#         LOGGER.debug("Setting flight mode: {}".format(mode))
#         mode_id = self.mav_connection.mode_mapping()[mode]
#         base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
#         ack_message = self.send_long_command(command=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
#                                              p1=base_mode,
#                                              p2=mode_id)
#         return ack_message
#
#     def arm_vehicle(self, force=False):
#         # https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
#         force_arm = 0
#         if force:
#             force_arm = 21196
#         ack_message = self.send_long_command(command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=1, p2=force_arm)
#         if ack_message.result == 0:
#             self.armed = True
#         print(str(ack_message))