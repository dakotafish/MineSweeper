import time
import logging


def create_logger(log_name):
    log_name = "logs/{}_{}_.log".format(log_name, str(time.strftime("%Y%m%d-%H%M%S")))
    logging.basicConfig(filename=log_name, format='%(levelname)s:%(message)s', level=logging.DEBUG)
    return logging.getLogger(__name__)

def big_print(text):
    print("######")
    print("############### {}  ###############".format(text))
    print("######")

def bit_mapper(map, debug=False):
    # checks each byte and evaluates it to true/false
    bit_mask = []
    one_byte = 0b1
    for n in range(map.bit_length()):
        flag = bool(map & (one_byte << n))
        bit_mask.append(flag)
    if debug:
        print("Bitmask:")
        for i in bit_mask: print(i)
    return bit_mask

# def bit_mapper(map, enums=None, print_it=False):
#     # checks each byte and evaluates it to true/false
#     #  maps those true/false flags to a separate enums list if one is given
#     # side note,, don't use 32, use the actual length here m.onboard_control_sensors_present.bit_length()
#     bit_mask = []
#     one_byte = 0b1
#     for n in range(map.bit_length()):
#         flag = bool(map & (one_byte << n))
#         bit_mask.append(flag)
#     if enums:
#         for i in range(map.bit_length()):
#             flag = bit_mask[i]
#             enums[i].append(flag)
#         bit_mask = enums
#     if print_it:
#         print(bit_mask)
#         for i in bit_mask: print(i)
#         # for i in bit_mask:
#         #     print("Flag Name: {1}\t\t Flag Value: {2} \n\t Flag Description: {3}".format(i[0], i[2], i[1]))
#     return bit_mask