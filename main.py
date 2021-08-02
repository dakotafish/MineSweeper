from copter import Copter
from utils import big_print

if __name__ == "__main__":
    #copter = Copter("udpin:192.168.1.17:14550")
    copter = Copter()
    big_print("Connecting to Copter")
    copter.connect()
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