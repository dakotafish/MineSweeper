import signal
import time
import queue
import utils
import uuid
from pymavlink import mavutil
from threading import Thread

# Mavlink command protocol: https://mavlink.io/en/services/command.html

MAV_CMDS = mavutil.mavlink.enums["MAV_CMD"]
MAV_RESULTS = mavutil.mavlink.enums["MAV_RESULT"]

class CommandQueue(object):
    def __init__(self, copter):
        self.copter = copter
        self.queue = queue.Queue()
        self.thread = Thread(target=self.drain_command_queue, daemon=False)

    def drain_command_queue(self):
        while not self.queue.empty():
            command = self.queue.get(block=False, timeout=1)
            # add the commands message_hook to copter.message_listeners
            self.copter.message_listeners["COMMAND_ACK"][command.uuid] = command.command_message_hook
            command.send_command()
            while not command.command_complete:
                time.sleep(.01)
            # once the command completes we remove it from the message_listeners
            del self.copter.message_listeners["COMMAND_ACK"][command.uuid]
            self.queue.task_done()

    def put(self, command):
        self.queue.put(command)

    def send_commands(self):
        if not self.thread.is_alive():
            self.thread = Thread(target=self.drain_command_queue, daemon=False)
            self.thread.start()


class MavCommand(object):
    """
    Base MavCommand class - a wrapper for MAV_CMD's: https://mavlink.io/en/messages/common.html#mav_commands
    Should be subclassed by either:
        CommandInt() - where params 5 & 6 are floats
        or
        CommandLong() - where all params are ints
    """

    # https://mavlink.io/en/messages/common.html#mav_commands
    def __init__(self,
                 mav_connection,
                 command_id,
                 frame=None,
                 p1=0,
                 p2=0,
                 p3=0,
                 p4=0,
                 p5=0,
                 p6=0,
                 p7=0,
                 target_system=0,
                 target_component=0):
        self.uuid = uuid.uuid4().hex
        self.mav_connection = mav_connection
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
                raise Exception
            elif mav_result.name == "MAV_RESULT_FAILED":
                self.command_complete = True
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
        super(MavCommand, self).__init__(mav_connection,
                                         command_id,
                                         frame=frame,
                                         p1=p1, p2=p2, p3=p3, p4=p4, p5=p5, p6=p6, p7=p7,
                                         target_system=target_system,
                                         target_component=target_component)
        self.p5 = int(self.p5)
        self.p6 = int(self.p6)

    def set_command_send_method(self):
        self.command_send_method = self.mav_connection.mav.command_int_send
        self.command_send_args = lambda: (self.target_system,
                                          self.target_component,
                                          self.frame,
                                          self.command_id,
                                          self.current,
                                          self.autocontinue,
                                          self.p1, self.p2, self.p3, self.p4, self.p5, self.p6, self.p7)


class CommandLong(MavCommand):
    """
    https://mavlink.io/en/messages/common.html#COMMAND_INT
    """
    def __init__(self,
                 mav_connection,
                 command_id,
                 p1=0,
                 p2=0,
                 p3=0,
                 p4=0,
                 p5=0,
                 p6=0,
                 p7=0,
                 target_system=0,
                 target_component=0):
        super(MavCommand, self).__init__(mav_connection,
                                         command_id,
                                         p1=p1, p2=p2, p3=p3, p4=p4, p5=p5, p6=p6, p7=p7,
                                         target_system=target_system,
                                         target_component=target_component)
        self.p5 = float(self.p5)
        self.p6 = float(self.p6)

    def set_command_send_method(self):
        self.command_send_method = self.mav_connection.mav.command_int_send
        self.command_send_args = lambda: (self.target_system,
                                          self.target_component,
                                          self.command_id,
                                          self.confirmation,
                                          self.p1, self.p2, self.p3, self.p4, self.p5, self.p6, self.p7)

class MissionItemInt(MavCommand):
    """
    https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT
    """
    pass