from board import SCL, SDA
import busio
import time

import board
import digitalio
from digitalio import DigitalInOut, Direction

# This example also relies on the Adafruit motor library available here:
# https://github.com/adafruit/Adafruit_CircuitPython_Motor
from adafruit_motor import servo

# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685


def set_servo_pos(idx, pos):
    servo_pos[idx] = pos


def servo_set_pwm(idx, pos):
    if idx in servo_key_dict:
        # print(servo_key_dict[idx])
        servo_obj_dict[servo_key_dict[idx]].angle = pos
        time.sleep(0.001)
    set_servo_pos(idx, pos)


def commit_servo(servo_pos: list):
    idx = 0
    for pos in servo_pos:
        servo_set_pwm(idx, pos)
        idx += 1


def servo_reset(val):
    set_servo_pos(SERVO_FW_1ST_JOINT_L_IDX, SERVO_FW_1ST_JOINT_L_CLOSE_POS)
    set_servo_pos(SERVO_FW_2ND_JOINT_L_IDX, SERVO_FW_2ND_JOINT_L_CLOSE_POS)
    set_servo_pos(SERVO_FW_1ST_JOINT_R_IDX, SERVO_FW_1ST_JOINT_R_CLOSE_POS)
    set_servo_pos(SERVO_FW_2ND_JOINT_R_IDX, SERVO_FW_2ND_JOINT_R_CLOSE_POS)
    set_servo_pos(SERVO_BW_1ST_JOINT_L_IDX, SERVO_BW_1ST_JOINT_L_CLOSE_POS)
    set_servo_pos(SERVO_BW_2ND_JOINT_L_IDX, SERVO_BW_2ND_JOINT_L_CLOSE_POS)
    set_servo_pos(SERVO_BW_1ST_JOINT_R_IDX, SERVO_BW_1ST_JOINT_R_CLOSE_POS)
    set_servo_pos(SERVO_BW_2ND_JOINT_R_IDX, SERVO_BW_2ND_JOINT_R_CLOSE_POS)
    commit_servo(servo_pos)
    print(servo_pos)
    return val


def init_servo():
    j = 0
    for idx in servo_idx_list:
        servo_vec[idx] = servo_vec_list[j]
        j = j + 1

    j = 0
    for idx in servo_idx_list:
        servo_obj_dict[servo_key_list[j]] = servo.Servo(
            pca.channels[idx], actuation_range=270, min_pulse=600, max_pulse=2400
        )
        j = j + 1

    print(servo_obj_dict[servo_key_list[0]])


def servo_motion(val):
    set_servo_pos(2, 383)
    set_servo_pos(0, 266)
    set_servo_pos(6, 379)
    set_servo_pos(4, 379)
    commit_servo(servo_pos)  # must commit, here!!
    webiopi.sleep(0.5)

    set_servo_pos(8, 378)
    set_servo_pos(10, 258)
    set_servo_pos(14, 261)
    set_servo_pos(12, 376)

    commit_servo(servo_pos)
    return val


def adjust_2nd_leg(dst, bk):
    while 1:
        set_servo_pos(SERVO_FW_2ND_JOINT_L_IDX, SERVO_FW_2ND_JOINT_L_RESET_POS)
        set_servo_pos(SERVO_FW_2ND_JOINT_R_IDX, SERVO_FW_2ND_JOINT_R_RESET_POS)
        set_servo_pos(SERVO_BW_2ND_JOINT_L_IDX, SERVO_BW_2ND_JOINT_L_RESET_POS)
        set_servo_pos(SERVO_BW_2ND_JOINT_R_IDX, SERVO_BW_2ND_JOINT_R_RESET_POS)
        commit_servo(servo_pos)
        time.sleep(2)

        for i in range(0, dst):
            a = SERVO_FW_2ND_JOINT_L_RESET_POS - (
                i * 10 + SERVO_FW_2ND_JOINT_L_OPEN_DELTA_POS
            )
            b = SERVO_FW_2ND_JOINT_R_RESET_POS + (
                i * 10 + SERVO_FW_2ND_JOINT_R_OPEN_DELTA_POS
            )
            c = SERVO_BW_2ND_JOINT_L_RESET_POS + (
                i * 10 + SERVO_BW_2ND_JOINT_L_OPEN_DELTA_POS
            )
            d = SERVO_BW_2ND_JOINT_R_RESET_POS - (
                i * 10 + SERVO_BW_2ND_JOINT_R_OPEN_DELTA_POS
            )
            set_servo_pos(SERVO_FW_2ND_JOINT_L_IDX, a)
            set_servo_pos(SERVO_FW_2ND_JOINT_R_IDX, b)
            set_servo_pos(SERVO_BW_2ND_JOINT_L_IDX, c)
            set_servo_pos(SERVO_BW_2ND_JOINT_R_IDX, d)
            commit_servo(servo_pos)
            # time.sleep(0.01)

        if bk:
            break

        time.sleep(1)
        aa = a
        bb = b
        cc = c
        dd = d
        for i in range(0, dst):
            a = aa + i * 10
            b = bb - i * 10
            c = cc - i * 10
            d = dd + i * 10
            set_servo_pos(SERVO_FW_2ND_JOINT_L_IDX, a)
            set_servo_pos(SERVO_FW_2ND_JOINT_R_IDX, b)
            set_servo_pos(SERVO_BW_2ND_JOINT_L_IDX, c)
            set_servo_pos(SERVO_BW_2ND_JOINT_R_IDX, d)
            commit_servo(servo_pos)
            # time.sleep(0.01)


def adjust_1st_leg(dst, bk):
    while 1:
        set_servo_pos(SERVO_FW_1ST_JOINT_L_IDX, SERVO_FW_1ST_JOINT_L_RESET_POS)
        set_servo_pos(SERVO_FW_1ST_JOINT_R_IDX, SERVO_FW_1ST_JOINT_R_RESET_POS)
        set_servo_pos(SERVO_BW_1ST_JOINT_L_IDX, SERVO_BW_1ST_JOINT_L_RESET_POS)
        set_servo_pos(SERVO_BW_1ST_JOINT_R_IDX, SERVO_BW_1ST_JOINT_R_RESET_POS)
        commit_servo(servo_pos)
        time.sleep(2)

        for i in range(0, dst):
            a = SERVO_FW_1ST_JOINT_L_RESET_POS - (
                i * 10 + SERVO_FW_1ST_JOINT_L_OPEN_DELTA_POS
            )
            b = SERVO_FW_1ST_JOINT_R_RESET_POS + (
                i * 10 + SERVO_FW_1ST_JOINT_R_OPEN_DELTA_POS
            )
            c = SERVO_BW_1ST_JOINT_L_RESET_POS + (
                i * 10 + SERVO_BW_1ST_JOINT_L_OPEN_DELTA_POS
            )
            d = SERVO_BW_1ST_JOINT_R_RESET_POS - (
                i * 10 + SERVO_BW_1ST_JOINT_R_OPEN_DELTA_POS
            )
            set_servo_pos(SERVO_FW_1ST_JOINT_L_IDX, a)
            set_servo_pos(SERVO_FW_1ST_JOINT_R_IDX, b)
            set_servo_pos(SERVO_BW_1ST_JOINT_L_IDX, c)
            set_servo_pos(SERVO_BW_1ST_JOINT_R_IDX, d)
            commit_servo(servo_pos)
            # time.sleep(0.01)

        if bk:
            break

        time.sleep(1)
        aa = a
        bb = b
        cc = c
        dd = d
        for i in range(0, dst):
            a = aa + i * 10
            b = bb - i * 10
            c = cc - i * 10
            d = dd + i * 10
            set_servo_pos(SERVO_FW_1ST_JOINT_L_IDX, a)
            set_servo_pos(SERVO_FW_1ST_JOINT_R_IDX, b)
            set_servo_pos(SERVO_BW_1ST_JOINT_L_IDX, c)
            set_servo_pos(SERVO_BW_1ST_JOINT_R_IDX, d)
            commit_servo(servo_pos)
            # time.sleep(0.01)


def robot_main():
    # servo_motion_slow_stand_up()
    # servo_motion(0)

    time.sleep(1)
    # servo_motion_slow_sit_down()
    init_servo()
    servo_reset(0)
    time.sleep(1)

    # 	adjust_1st_leg(4,False)
    adjust_2nd_leg(7, False)

    print("end of loop")


print("hoge")

SERVO_FW_1ST_JOINT_L_IDX = 2
SERVO_FW_2ND_JOINT_L_IDX = 14
SERVO_FW_1ST_JOINT_R_IDX = 0
SERVO_FW_2ND_JOINT_R_IDX = 8
SERVO_BW_1ST_JOINT_L_IDX = 6
SERVO_BW_2ND_JOINT_L_IDX = 12
SERVO_BW_1ST_JOINT_R_IDX = 4
SERVO_BW_2ND_JOINT_R_IDX = 10

SERVO_FW_1ST_JOINT_L_RESET_POS = 242
SERVO_FW_2ND_JOINT_L_RESET_POS = 238
SERVO_FW_1ST_JOINT_R_RESET_POS = 60
SERVO_FW_2ND_JOINT_R_RESET_POS = 46
SERVO_BW_1ST_JOINT_L_RESET_POS = 49
SERVO_BW_2ND_JOINT_L_RESET_POS = 47
SERVO_BW_1ST_JOINT_R_RESET_POS = 237
SERVO_BW_2ND_JOINT_R_RESET_POS = 244

SERVO_FW_1ST_JOINT_L_OPEN_DELTA_POS = 17
SERVO_FW_2ND_JOINT_L_OPEN_DELTA_POS = 0
SERVO_FW_1ST_JOINT_R_OPEN_DELTA_POS = 12
SERVO_FW_2ND_JOINT_R_OPEN_DELTA_POS = 0
SERVO_BW_1ST_JOINT_L_OPEN_DELTA_POS = 18
SERVO_BW_2ND_JOINT_L_OPEN_DELTA_POS = 0
SERVO_BW_1ST_JOINT_R_OPEN_DELTA_POS = 13
SERVO_BW_2ND_JOINT_R_OPEN_DELTA_POS = 0

SERVO_FW_1ST_JOINT_L_MIN_POS = SERVO_FW_1ST_JOINT_L_RESET_POS
SERVO_FW_2ND_JOINT_L_MIN_POS = 0
SERVO_FW_1ST_JOINT_R_MIN_POS = 0
SERVO_FW_2ND_JOINT_R_MIN_POS = SERVO_FW_2ND_JOINT_R_RESET_POS
SERVO_BW_1ST_JOINT_L_MIN_POS = 0
SERVO_BW_2ND_JOINT_L_MIN_POS = SERVO_BW_2ND_JOINT_L_RESET_POS
SERVO_BW_1ST_JOINT_R_MIN_POS = SERVO_BW_1ST_JOINT_R_RESET_POS
SERVO_BW_2ND_JOINT_R_MIN_POS = 0

SERVO_FW_1ST_JOINT_L_MAX_POS = 0
SERVO_FW_2ND_JOINT_L_MAX_POS = SERVO_FW_2ND_JOINT_L_RESET_POS
SERVO_FW_1ST_JOINT_R_MAX_POS = SERVO_FW_1ST_JOINT_R_RESET_POS
SERVO_FW_2ND_JOINT_R_MAX_POS = 0
SERVO_BW_1ST_JOINT_L_MAX_POS = SERVO_BW_1ST_JOINT_L_RESET_POS
SERVO_BW_2ND_JOINT_L_MAX_POS = 0
SERVO_BW_1ST_JOINT_R_MAX_POS = 0
SERVO_BW_2ND_JOINT_R_MAX_POS = SERVO_BW_2ND_JOINT_R_RESET_POS

SERVO_FW_1ST_JOINT_L_OPEN_POS = SERVO_FW_1ST_JOINT_L_MAX_POS
SERVO_FW_2ND_JOINT_L_OPEN_POS = SERVO_FW_2ND_JOINT_L_MIN_POS
SERVO_FW_1ST_JOINT_R_OPEN_POS = SERVO_FW_1ST_JOINT_R_MIN_POS
SERVO_FW_2ND_JOINT_R_OPEN_POS = SERVO_FW_2ND_JOINT_R_MAX_POS
SERVO_BW_1ST_JOINT_L_OPEN_POS = SERVO_BW_1ST_JOINT_L_MIN_POS
SERVO_BW_2ND_JOINT_L_OPEN_POS = SERVO_BW_2ND_JOINT_L_MAX_POS
SERVO_BW_1ST_JOINT_R_OPEN_POS = SERVO_BW_1ST_JOINT_R_MAX_POS
SERVO_BW_2ND_JOINT_R_OPEN_POS = SERVO_BW_2ND_JOINT_R_MIN_POS

SERVO_FW_1ST_JOINT_L_CLOSE_POS = SERVO_FW_1ST_JOINT_L_RESET_POS
SERVO_FW_2ND_JOINT_L_CLOSE_POS = SERVO_FW_2ND_JOINT_L_RESET_POS
SERVO_FW_1ST_JOINT_R_CLOSE_POS = SERVO_FW_1ST_JOINT_R_RESET_POS
SERVO_FW_2ND_JOINT_R_CLOSE_POS = SERVO_FW_2ND_JOINT_R_RESET_POS
SERVO_BW_1ST_JOINT_L_CLOSE_POS = SERVO_BW_1ST_JOINT_L_RESET_POS
SERVO_BW_2ND_JOINT_L_CLOSE_POS = SERVO_BW_2ND_JOINT_L_RESET_POS
SERVO_BW_1ST_JOINT_R_CLOSE_POS = SERVO_BW_1ST_JOINT_R_RESET_POS
SERVO_BW_2ND_JOINT_R_CLOSE_POS = SERVO_BW_2ND_JOINT_R_RESET_POS

##		   0 1 2 3 4 5 6 7 8 9 A B C D E F
servo_pos = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
##			   0 1 2 3 4 5 6 7 8 9 A B C D E F
servo_pos_min = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
##			   0 1 2 3 4 5 6 7 8 9 A B C D E F
servo_pos_max = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
##			   0 1 2 3 4 5 6 7 8 9 A B C D E F
servo_vec = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

servo_obj_dict = {}

servo_key_list = [
    "fw_1st_l",
    "fw_2nd_l",
    "fw_1st_r",
    "fw_2nd_r",
    "bw_1st_l",
    "bw_2nd_l",
    "bw_1st_r",
    "bw_2nd_r",
]
servo_idx_list = [
    SERVO_FW_1ST_JOINT_L_IDX,
    SERVO_FW_2ND_JOINT_L_IDX,
    SERVO_FW_1ST_JOINT_R_IDX,
    SERVO_FW_2ND_JOINT_R_IDX,
    SERVO_BW_1ST_JOINT_L_IDX,
    SERVO_BW_2ND_JOINT_L_IDX,
    SERVO_BW_1ST_JOINT_R_IDX,
    SERVO_BW_2ND_JOINT_R_IDX,
]
servo_vec_list = [1, -1, -1, 1, -1, 1, 1, -1]

servo_idx_dict = dict(zip(servo_key_list, servo_idx_list))
servo_key_dict = dict(zip(servo_idx_list, servo_key_list))
# servo_motion_dict_list = []
print(len(servo_pos))
print(servo_pos)

i2c = busio.I2C(SCL, SDA)
# Create a simple PCA9685 class instance./0000000
pca = None
pca = PCA9685(i2c)
pca.frequency = 50

robot_main()
