from copter import Copter
from utils import big_print
import utils
from pymavlink import mavutil
import os

# os.environ['MAVLINK20'] = '1'
# mavutil.set_dialect("ardupilotmega")

if __name__ == "__main__":
    #copter = Copter("udpin:192.168.1.17:14550")
    utils.big_print("Connecting to Copter")
    copter = Copter()
    input("Press enter to run prearm checks.")
    copter.run_prearm_checks()
    input("Press enter to request data stream for copters position.")
    copter.request_data_stream(mavutil.mavlink.MAV_DATA_STREAM_POSITION)
    input("Press enter to disable radio failsafe.")
    copter.disable_radio_failsafe()
    input("Press enter to clear any old missions from the vehicle.")
    copter.nav.clear_all_waypoints()
    input("Press enter to create a mission.")
    print("existing mission:{}".format(copter.nav.mission))
    copter.nav.mission.create_square_survey_mission()
    print("New Mission: {}".format(str(copter.nav.mission)))
    for i in copter.nav.mission.values():
        print("Mission item: {}".format(str(i.__dict__)))
    print("...")
    input("Press enter to upload the new mission.")
    copter.nav.add_mission_progress_hooks_to_copter()
    copter.nav.upload_mission()
    input("Press enter to run the mission.")
    copter.set_flight_mode("GUIDED", send=True)
    copter.arm(force=False, send=True)
    copter.simple_takeoff(target_altitude=20, send=True)
    input("Press enter to add message hooks and change to AUTO mode.")
    copter.nav.start_mission()
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