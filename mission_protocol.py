import geopy
import math
import uuid
from geopy.distance import distance as geopy_distance
from pymavlink import mavutil
from command_protocol import MissionItemInt

MAV_FRAMES = {v.name: k for k, v in mavutil.mavlink.enums['MAV_FRAME'].items()}
MAV_MISSION_TYPES = {v.name: k for k, v in mavutil.mavlink.enums['MAV_MISSION_TYPE'].items()}
MAV_MISSION_RESULTS = mavutil.mavlink.enums["MAV_MISSION_RESULT"]

import logging
import utils
LOGGER = utils.create_logger("MISSION_")

class Navigation(object):
    def __init__(self, copter):
        self.copter = copter
        self.mav = copter.mav
        self.target_system = copter.target_system
        self.target_component = copter.target_component
        self.mission = Mission(copter)
        self.mission_ready = False
        self.current_mission_item = None
        self.change_to_mission_item = None
        #self.mission.create_square_survey_mission()
        self.start_mission_immediately = False
        self.mission_progress_message_types = dict(MISSION_ITEM_REACHED=uuid.uuid4().hex,
                                                   MISSION_CURRENT=uuid.uuid4().hex)

    def start_mission(self):
        assert self.mission_ready
        self.add_mission_progress_hooks_to_copter()
        self.copter.set_flight_mode("AUTO")

    def add_mission_progress_hooks_to_copter(self):
        for message_type, message_key in self.mission_progress_message_types.items():
            if not self.copter.message_router.message_listeners_contains(message_type=message_type, key=message_key):
                self.copter.message_router.add_to_message_listeners_dict(message_type=message_type,key=message_key,
                                                                         message_hook=self.mission_progress_message_hook)

    def clear_all_waypoints(self):
        self.mav.waypoint_clear_all_send()

    def send_empty_mission(self):
        # just a different way to clear the mission. basically the same as clear_all_waypoints()
        self.mav.mav.mission_count_send(target_system=self.target_system,
                                        target_component=self.target_component,
                                        count=0,
                                        mission_type=self.mission.type)

    def upload_mission(self):
        # this message initiates a mission upload: https://mavlink.io/en/messages/common.html#MISSION_COUNT
        #  after sending this message we should receive self.mission.count # of MISSION_REQUEST_INT messages
        self.mav.mav.mission_count_send(target_system=self.target_system,
                                        target_component=self.target_component,
                                        count=self.mission.count())
                                        #mission_type=self.mission.type)

    def mission_request_int_message_hook(self, message):
        # after send_mission_count() the vehicle will request each item in the mission
        assert message.get_type() in ["MISSION_REQUEST_INT", "MISSION_REQUEST"]
        LOGGER.debug("MissionItemMessageHook- Received: {}".format(message))
        print(message)
        #assert message.mission_type == self.mission.type
        # TODO - I don't understand why ardupilot sends back a MISSION_REQUEST but it makes me think I'm doing something wrong..
        mission_item = self.mission.get(message.seq)
        if mission_item.was_sent:
            # do nothing
            LOGGER.debug("Command was already sent.. doing nothing.")
            return
        if message.seq > 0:
            previous_mission_item = self.mission.get(message.seq - 1)
            if previous_mission_item.was_sent:
                mission_item.send()
                LOGGER.debug("MissionItemMessageHook- Responded with: {}".format(mission_item.__dict__.items()))
            else:
                LOGGER.debug("Mission Item was out of order, sent back to message router.")
                self.send_back_to_message_router(message)
        else:
            mission_item.send()
            LOGGER.debug("MissionItemMessageHook- Responded with: {}".format(mission_item.__dict__.items()))


    def send_back_to_message_router(self, message):
        self.copter.message_router.router(self.mav, message)

    def mission_upload_ack_message_hook(self, message):
        # should receive a MISSION_ACK message back after mission upload is complete
        assert message.get_type() == "MISSION_ACK"
        print(message)
        upload_result = MAV_MISSION_RESULTS.get(message.type)
        if upload_result.name == "MAV_MISSION_ACCEPTED":
            self.mission_ready = True
            if self.start_mission_immediately:
                self.start_mission()
            return None
        else:
            # TODO Need a better exception here
            pass
            raise Exception("Mission upload failed. Result was: {}, {}".format(upload_result.name,
                                                                              upload_result.description))

    def set_current_mission_item(self, seq):
        # set the current mission item to seq (seq is the index of a mission item)
        assert seq in self.mission
        self.change_to_mission_item = seq
        self.mav.mav.mission_set_current_send(target_system=self.target_system, target_component=self.target_component,
                                              seq=seq)

    def mission_progress_message_hook(self, message):
        assert message.get_type() in ["MISSION_ITEM_REACHED", "MISSION_CURRENT"]
        print(message)
        if message.get_type() == "MISSION_ITEM_REACHED":
            print("Mission Item #{} has been reached!".format(message.seq))
            self.current_mission_item = message.seq
        elif message.get_type() == "MISSION_CURRENT":
            if self.change_to_mission_item == message.seq:
                print("Mission Item was changed from #{} to #{} successfully!".format(self.current_mission_item,
                                                                                      message.seq))
                self.change_to_mission_item = None
            elif self.current_mission_item != message.seq:
                self.current_mission_item = message.seq
                print("Moved on to Mission Item #{}.".format(message.seq))




class Mission(dict):

    def __init__(self, copter, *args, **kwargs):
        super(dict, self).__init__(*args, **kwargs)
        self.copter = copter
        self.mav = copter.mav
        self.type = kwargs.get("type") or MAV_MISSION_TYPES["MAV_MISSION_TYPE_MISSION"]
        self.count = lambda: len(self.keys())


    def create_square_survey_mission(self, distance=10):
        #self.copter.set_home_position()
        current_position = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        bearing = current_position.hdg * .01
        start_lat, start_lon, start_rel_alt = current_position.lat, current_position.lon, current_position.relative_alt
        start_location = Location(start_lat, start_lon, start_rel_alt, bearing)
        origin = geopy.Point(start_location.degrees_lat, start_location.degrees_lon,
                             altitude=start_location.encoded_alt)
        self[0] = MissionItemInt.from_location(self.mav,
                                               mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                               start_location,
                                               0,
                                               MAV_FRAMES['MAV_FRAME_GLOBAL_RELATIVE_ALT'],
                                               current=True,
                                               autocontinue=True)

        for i in range(1, (distance * 2 + 1)):
            seq = i
            destination = geopy_distance(meters=distance).destination(origin, bearing)
            print("Destination: {}".format(destination))
            seq_location = Location.from_destination(destination)
            self[seq] = MissionItemInt.from_location(self.mav,
                                                     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                                     seq_location,
                                                     seq,
                                                     MAV_FRAMES['MAV_FRAME_GLOBAL_RELATIVE_ALT'],
                                                     current=False,
                                                     autocontinue=True)
            print("Mission Item: {}".format(self[seq]))


class Location(object):

    def __init__(self, lat, lon, alt, bearing=0):
        self.encoded_lat = lat
        self.degrees_lat = self.encoded_lat * 1.0e-7
        self.radians_lat = math.radians(self.degrees_lat)
        self.encoded_lon = lon
        self.degrees_lon = self.encoded_lon * 1.0e-7
        self.radians_lon = math.radians(self.degrees_lon)
        self.encoded_alt = alt
        self.alt = self.encoded_alt * 1E4
        if bearing:
            self.encoded_bearing = bearing
            self.bearing = self.encoded_bearing * .01

    @classmethod
    def from_destination(cls, destination):
        lat = destination.latitude * 1.0e-7
        lon = destination.longitude * 1.0e-7
        alt = destination.altitude
        return cls(lat, lon, alt)


        # wp_int = mavutil.mavlink.MAVLink_mission_item_int_message(wp.target_system,
        #                                                           wp.target_component,
        #                                                           wp.seq,
        #                                                           wp.frame,
        #                                                           wp.command,
        #                                                           wp.current,
        #                                                           wp.autocontinue,
        #                                                           wp.param1,
        #                                                           wp.param2,
        #                                                           wp.param3,
        #                                                           wp.param4,
        #                                                           int(wp.x * 1.0e7),
        #                                                           int(wp.y * 1.0e7),
        #                                                           wp.z)