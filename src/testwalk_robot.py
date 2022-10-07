import webiopi

import Adafruit_PCA9685
import time
from getch import getch
import RPi.GPIO as GPIO
import os
import pickle
import time

ROOT_PATH = "/home/pi/RetroPie/pi_shares/lego_robot_arm-clone/src/python/work/"

MOTOR_PWM_FREQ = 1000  # [Hz]
MOTOR_FW_L_AIN1 = 27
MOTOR_FW_L_AIN2 = 22
MOTOR_FW_R_BIN1 = 10
MOTOR_FW_R_BIN2 = 9

MOTOR_BW_ENABLE = 13
MOTOR_BW_L_AIN1 = 5
MOTOR_BW_L_AIN2 = 6
MOTOR_BW_R_BIN1 = 19
MOTOR_BW_R_BIN2 = 26

SERVO_FW_1ST_JOINT_L_IDX = 2
SERVO_FW_2ND_JOINT_L_IDX = 8
SERVO_FW_1ST_JOINT_R_IDX = 0
SERVO_FW_2ND_JOINT_R_IDX = 10
SERVO_BW_1ST_JOINT_L_IDX = 6
SERVO_BW_2ND_JOINT_L_IDX = 14
SERVO_BW_1ST_JOINT_R_IDX = 4
SERVO_BW_2ND_JOINT_R_IDX = 12
SERVO_MISSILE_IDX = 15

SERVO_FW_1ST_JOINT_L_RESET_POS = 320
SERVO_FW_2ND_JOINT_L_RESET_POS = 446
SERVO_FW_1ST_JOINT_R_RESET_POS = 328
SERVO_FW_2ND_JOINT_R_RESET_POS = 187
SERVO_BW_1ST_JOINT_L_RESET_POS = 445
SERVO_BW_2ND_JOINT_L_RESET_POS = 189
SERVO_BW_1ST_JOINT_R_RESET_POS = 316
SERVO_BW_2ND_JOINT_R_RESET_POS = 445

SERVO_FW_1ST_JOINT_L_MIN_POS = SERVO_FW_1ST_JOINT_L_RESET_POS
SERVO_FW_2ND_JOINT_L_MIN_POS = 187
SERVO_FW_1ST_JOINT_R_MIN_POS = 205
SERVO_FW_2ND_JOINT_R_MIN_POS = SERVO_FW_2ND_JOINT_R_RESET_POS
SERVO_BW_1ST_JOINT_L_MIN_POS = 313
SERVO_BW_2ND_JOINT_L_MIN_POS = SERVO_BW_2ND_JOINT_L_RESET_POS
SERVO_BW_1ST_JOINT_R_MIN_POS = SERVO_BW_1ST_JOINT_R_RESET_POS
SERVO_BW_2ND_JOINT_R_MIN_POS = 187
SERVO_MISSILE_MIN_POS = 150

SERVO_FW_1ST_JOINT_L_MAX_POS = 446
SERVO_FW_2ND_JOINT_L_MAX_POS = SERVO_FW_2ND_JOINT_L_RESET_POS
SERVO_FW_1ST_JOINT_R_MAX_POS = SERVO_FW_1ST_JOINT_R_RESET_POS
SERVO_FW_2ND_JOINT_R_MAX_POS = 449
SERVO_BW_1ST_JOINT_L_MAX_POS = SERVO_BW_1ST_JOINT_L_RESET_POS
SERVO_BW_2ND_JOINT_L_MAX_POS = 446
SERVO_BW_1ST_JOINT_R_MAX_POS = 442
SERVO_BW_2ND_JOINT_R_MAX_POS = SERVO_BW_2ND_JOINT_R_RESET_POS
SERVO_MISSILE_MAX_POS = 440

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

# 		   0 1 2 3 4 5 6 7 8 9 A B C D E F
servo_pos = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
# 			   0 1 2 3 4 5 6 7 8 9 A B C D E F
servo_pos_min = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
# 			   0 1 2 3 4 5 6 7 8 9 A B C D E F
servo_pos_max = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
# 			   0 1 2 3 4 5 6 7 8 9 A B C D E F
servo_vec = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
# 			   0 1 2 3 4 5 6 7 8 9 A B C D E F
servo_range = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

servo_wait = [
    0.01,
    0.01,
    0.05,
    0.05,
    0.05,
    0.05,
    0.05,
    0.05,
    0.05,
    0.05,
    0.05,
    0.05,
    0.05,
    0.05,
    0.05,
    0.05,
]

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
servo_range_list = [90.0, 180.0, 90.0, 180.0, 90.0, 180.0, 90.0, 180.0]

servo_idx_dict = dict(zip(servo_key_list, servo_idx_list))
servo_motion_dict_list = []

print(len(servo_pos))
print(servo_pos)

pwm = None
pwm = Adafruit_PCA9685.PCA9685()


def setup():
    init_motor()


def loop():
    webiopi.sleep(0.5)


def set_servo_pos(idx, pos):
    servo_pos[idx] = pos


def servo_set_pwm(idx, pos):
    pwm.set_pwm(idx, 0, pos)
    set_servo_pos(idx, pos)


def commit_servo(servo_pos: list):
    idx = 0
    for pos in servo_pos:
        servo_set_pwm(idx, pos)
        idx += 1


@webiopi.macro
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
    return val


def init_servo():
    pwm.set_pwm_freq(50)
    servo_pos_min[SERVO_FW_1ST_JOINT_L_IDX] = SERVO_FW_1ST_JOINT_L_MIN_POS
    servo_pos_min[SERVO_FW_2ND_JOINT_L_IDX] = SERVO_FW_2ND_JOINT_L_MIN_POS
    servo_pos_min[SERVO_FW_1ST_JOINT_R_IDX] = SERVO_FW_1ST_JOINT_R_MIN_POS
    servo_pos_min[SERVO_FW_2ND_JOINT_R_IDX] = SERVO_FW_2ND_JOINT_R_MIN_POS
    servo_pos_min[SERVO_BW_1ST_JOINT_L_IDX] = SERVO_BW_1ST_JOINT_L_MIN_POS
    servo_pos_min[SERVO_BW_2ND_JOINT_L_IDX] = SERVO_BW_2ND_JOINT_L_MIN_POS
    servo_pos_min[SERVO_BW_1ST_JOINT_R_IDX] = SERVO_BW_1ST_JOINT_R_MIN_POS
    servo_pos_min[SERVO_BW_2ND_JOINT_R_IDX] = SERVO_BW_2ND_JOINT_R_MIN_POS
    servo_pos_min[SERVO_MISSILE_IDX] = SERVO_MISSILE_MIN_POS

    servo_pos_max[SERVO_FW_1ST_JOINT_L_IDX] = SERVO_FW_1ST_JOINT_L_MAX_POS
    servo_pos_max[SERVO_FW_2ND_JOINT_L_IDX] = SERVO_FW_2ND_JOINT_L_MAX_POS
    servo_pos_max[SERVO_FW_1ST_JOINT_R_IDX] = SERVO_FW_1ST_JOINT_R_MAX_POS
    servo_pos_max[SERVO_FW_2ND_JOINT_R_IDX] = SERVO_FW_2ND_JOINT_R_MAX_POS
    servo_pos_max[SERVO_BW_1ST_JOINT_L_IDX] = SERVO_BW_1ST_JOINT_L_MAX_POS
    servo_pos_max[SERVO_BW_2ND_JOINT_L_IDX] = SERVO_BW_2ND_JOINT_L_MAX_POS
    servo_pos_max[SERVO_BW_1ST_JOINT_R_IDX] = SERVO_BW_1ST_JOINT_R_MAX_POS
    servo_pos_max[SERVO_BW_2ND_JOINT_R_IDX] = SERVO_BW_2ND_JOINT_R_MAX_POS
    servo_pos_max[SERVO_MISSILE_IDX] = SERVO_MISSILE_MAX_POS

    j = 0
    for idx in servo_idx_list:
        servo_vec[idx] = servo_vec_list[j]
        j = j + 1
    servo_vec[SERVO_MISSILE_IDX] = 1

    j = 0
    for idx in servo_idx_list:
        servo_range[idx] = servo_range_list[j]
        j = j + 1
    servo_range[SERVO_MISSILE_IDX] = 180


def init_motor():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MOTOR_FW_L_AIN1, GPIO.OUT)
    GPIO.setup(MOTOR_FW_L_AIN2, GPIO.OUT)
    GPIO.setup(MOTOR_FW_R_BIN1, GPIO.OUT)
    GPIO.setup(MOTOR_FW_R_BIN2, GPIO.OUT)
    GPIO.setup(MOTOR_BW_L_AIN1, GPIO.OUT)
    GPIO.setup(MOTOR_BW_L_AIN2, GPIO.OUT)
    GPIO.setup(MOTOR_BW_R_BIN1, GPIO.OUT)
    GPIO.setup(MOTOR_BW_R_BIN2, GPIO.OUT)
    GPIO.setup(MOTOR_BW_ENABLE, GPIO.OUT)

    GPIO.output(MOTOR_FW_L_AIN1, 0)
    GPIO.output(MOTOR_FW_L_AIN2, 0)
    GPIO.output(MOTOR_FW_R_BIN1, 0)
    GPIO.output(MOTOR_FW_R_BIN2, 0)

    GPIO.output(MOTOR_BW_L_AIN1, 0)
    GPIO.output(MOTOR_BW_L_AIN2, 0)
    GPIO.output(MOTOR_BW_R_BIN1, 0)
    GPIO.output(MOTOR_BW_R_BIN2, 0)
    GPIO.output(MOTOR_BW_ENABLE, 1)

    init_servo()


def fw_r_motor(mode, move_time=0.1):
    if mode is "ccw":
        GPIO.output(MOTOR_FW_R_BIN1, GPIO.LOW)
        GPIO.output(MOTOR_FW_R_BIN2, GPIO.HIGH)
    elif mode is "cw":
        GPIO.output(MOTOR_FW_R_BIN1, GPIO.HIGH)
        GPIO.output(MOTOR_FW_R_BIN2, GPIO.LOW)
    else:
        GPIO.output(MOTOR_FW_R_BIN1, 0)
        GPIO.output(MOTOR_FW_R_BIN2, 0)
    time.sleep(move_time)
    GPIO.output(MOTOR_FW_R_BIN1, 0)
    GPIO.output(MOTOR_FW_R_BIN2, 0)


def fw_l_motor(mode, move_time=0.1):
    if mode is "ccw":
        GPIO.output(MOTOR_FW_L_AIN1, GPIO.LOW)
        GPIO.output(MOTOR_FW_L_AIN2, GPIO.HIGH)
    elif mode is "cw":
        GPIO.output(MOTOR_FW_L_AIN1, GPIO.HIGH)
        GPIO.output(MOTOR_FW_L_AIN2, GPIO.LOW)
    else:
        GPIO.output(MOTOR_FW_L_AIN1, GPIO.LOW)
        GPIO.output(MOTOR_FW_L_AIN2, GPIO.LOW)
    time.sleep(move_time)
    GPIO.output(MOTOR_FW_L_AIN1, GPIO.LOW)
    GPIO.output(MOTOR_FW_L_AIN2, GPIO.LOW)


def fw_l_motor_get_pwm(mode):
    if mode is "ccw":
        return GPIO.PWM(MOTOR_FW_L_AIN2, MOTOR_PWM_FREQ)
    elif mode is "cw":
        return GPIO.PWM(MOTOR_FW_L_AIN1, MOTOR_PWM_FREQ)
    else:
        return None


def fw_r_motor_get_pwm(mode):
    if mode is "ccw":
        return GPIO.PWM(MOTOR_FW_R_BIN2, MOTOR_PWM_FREQ)
    elif mode is "cw":
        return GPIO.PWM(MOTOR_FW_R_BIN1, MOTOR_PWM_FREQ)
    else:
        return None


def bw_l_motor_get_pwm(mode):
    if mode is "ccw":
        return GPIO.PWM(MOTOR_BW_L_AIN2, MOTOR_PWM_FREQ)
    elif mode is "cw":
        return GPIO.PWM(MOTOR_BW_L_AIN1, MOTOR_PWM_FREQ)
    else:
        return None


def bw_r_motor_get_pwm(mode):
    if mode is "ccw":
        return GPIO.PWM(MOTOR_BW_R_BIN2, MOTOR_PWM_FREQ)
    elif mode is "cw":
        return GPIO.PWM(MOTOR_BW_R_BIN1, MOTOR_PWM_FREQ)
    else:
        return None


def fw_l_motor_pwm(mode, duty, move_time=0.1):
    pig = fw_l_motor_get_pwm(mode)

    pig.start(duty)
    pig.ChangeDutyCycle(duty)
    time.sleep(move_time)
    pig.stop()


def fw_r_motor_pwm(mode, duty, move_time=0.1):
    pig = fw_r_motor_get_pwm(mode)

    pig.start(duty)
    pig.ChangeDutyCycle(duty)
    time.sleep(move_time)
    pig.stop()


def bw_r_motor(mode, move_time=0.1):
    if mode is "ccw":
        GPIO.output(MOTOR_BW_R_BIN1, GPIO.LOW)
        GPIO.output(MOTOR_BW_R_BIN2, GPIO.HIGH)
    elif mode is "cw":
        GPIO.output(MOTOR_BW_R_BIN1, GPIO.HIGH)
        GPIO.output(MOTOR_BW_R_BIN2, GPIO.LOW)
    else:
        GPIO.output(MOTOR_BW_R_BIN1, 0)
        GPIO.output(MOTOR_BW_R_BIN2, 0)
    time.sleep(move_time)
    GPIO.output(MOTOR_BW_R_BIN1, 0)
    GPIO.output(MOTOR_BW_R_BIN2, 0)


def bw_l_motor(mode, move_time=0.1):
    if mode is "ccw":
        GPIO.output(MOTOR_BW_L_AIN1, GPIO.LOW)
        GPIO.output(MOTOR_BW_L_AIN2, GPIO.HIGH)
    elif mode is "cw":
        GPIO.output(MOTOR_BW_L_AIN1, GPIO.HIGH)
        GPIO.output(MOTOR_BW_L_AIN2, GPIO.LOW)
    else:
        GPIO.output(MOTOR_BW_L_AIN1, GPIO.LOW)
        GPIO.output(MOTOR_BW_L_AIN2, GPIO.LOW)
    time.sleep(move_time)
    GPIO.output(MOTOR_BW_L_AIN1, GPIO.LOW)
    GPIO.output(MOTOR_BW_L_AIN2, GPIO.LOW)


def bw_l_motor_pwm(mode, duty, move_time=0.1):
    pig = bw_l_motor_get_pwm(mode)

    pig.start(duty)
    pig.ChangeDutyCycle(duty)
    time.sleep(move_time)
    pig.stop()


def bw_r_motor_pwm(mode, duty, move_time=0.1):
    pig = bw_r_motor_get_pwm(mode)

    pig.start(duty)
    pig.ChangeDutyCycle(duty)
    time.sleep(move_time)
    pig.stop()


def motor_demo1():
    loop = 1

    counter = 0
    while counter < loop:
        bw_l_motor_pwm("cw", 2.5 * 10, 1.0)
        bw_l_motor_pwm("cw", 7.25 * 10, 1.0)
        bw_l_motor_pwm("cw", 10.0 * 10, 1.0)
        bw_l_motor_pwm("cw", 7.25 * 10, 1.0)
        counter = counter + 1

    counter = 0
    while counter < loop:
        bw_r_motor_pwm("cw", 2.5 * 10, 1.0)
        bw_r_motor_pwm("cw", 7.25 * 10, 1.0)
        bw_r_motor_pwm("cw", 10.0 * 10, 1.0)
        bw_r_motor_pwm("cw", 7.25 * 10, 1.0)
        counter = counter + 1

    counter = 0
    while counter < loop:
        bw_l_motor_pwm("ccw", 2.5 * 10, 1.0)
        bw_l_motor_pwm("ccw", 7.25 * 10, 1.0)
        bw_l_motor_pwm("ccw", 10.0 * 10, 1.0)
        bw_l_motor_pwm("ccw", 7.25 * 10, 1.0)
        counter = counter + 1

    counter = 0
    while counter < loop:
        bw_r_motor_pwm("ccw", 2.5 * 10, 1.0)
        bw_r_motor_pwm("ccw", 7.25 * 10, 1.0)
        bw_r_motor_pwm("ccw", 10.0 * 10, 1.0)
        bw_r_motor_pwm("ccw", 7.25 * 10, 1.0)
        counter = counter + 1


def servo_transform_to_wheel():
    servo_idx_l = [
        SERVO_FW_1ST_JOINT_L_IDX,
        SERVO_FW_1ST_JOINT_R_IDX,
        SERVO_BW_1ST_JOINT_L_IDX,
        SERVO_BW_1ST_JOINT_R_IDX,
        SERVO_FW_2ND_JOINT_L_IDX,
        SERVO_FW_2ND_JOINT_R_IDX,
        SERVO_BW_2ND_JOINT_L_IDX,
        SERVO_BW_2ND_JOINT_R_IDX,
    ]
    servo_mov_pos_1st_l = [90, 90, 90, 90, 45, 45, 45, 45]
    servo_move_to_pos_list(servo_idx_l, servo_mov_pos_1st_l, servo_mov_pos_1st_l, 0)


def servo_transform_to_wheel_for_role():
    servo_idx_l = [
        SERVO_FW_1ST_JOINT_L_IDX,
        SERVO_FW_1ST_JOINT_R_IDX,
        SERVO_BW_1ST_JOINT_L_IDX,
        SERVO_BW_1ST_JOINT_R_IDX,
        SERVO_FW_2ND_JOINT_L_IDX,
        SERVO_FW_2ND_JOINT_R_IDX,
        SERVO_BW_2ND_JOINT_L_IDX,
        SERVO_BW_2ND_JOINT_R_IDX,
    ]
    servo_mov_pos_1st_l = [45, 45, 45, 45, 45, 45, 45, 45]
    servo_move_to_pos_list(servo_idx_l, servo_mov_pos_1st_l, servo_mov_pos_1st_l, 0)


def motor_move_with_speed(pig_list, speed, span_time):
    for pigv in pig_list:
        pigv.start(speed)
        pigv.ChangeDutyCycle(speed)

    time.sleep(span_time)

    for pigv in pig_list:
        pigv.stop()


def motor_move_foward(speed):
    servo_transform_to_wheel()

    pig = [0, 0, 0, 0]

    pig[0] = bw_l_motor_get_pwm("ccw")
    pig[1] = bw_r_motor_get_pwm("cw")
    pig[2] = fw_l_motor_get_pwm("ccw")
    pig[3] = fw_r_motor_get_pwm("cw")

    motor_move_with_speed(pig, speed, 1.0)


def motor_move_backward(speed):
    servo_transform_to_wheel()

    pig = [0, 0, 0, 0]

    pig[0] = bw_l_motor_get_pwm("cw")
    pig[1] = bw_r_motor_get_pwm("ccw")
    pig[2] = fw_l_motor_get_pwm("cw")
    pig[3] = fw_r_motor_get_pwm("ccw")

    motor_move_with_speed(pig, speed, 1.0)


def motor_role_left(speed):
    servo_transform_to_wheel_for_role()

    pig = [0, 0, 0, 0]

    pig[0] = bw_l_motor_get_pwm("cw")
    pig[1] = bw_r_motor_get_pwm("cw")
    pig[2] = fw_l_motor_get_pwm("cw")
    pig[3] = fw_r_motor_get_pwm("cw")

    motor_move_with_speed(pig, speed, 1.0)


def motor_role_right(speed):
    servo_transform_to_wheel_for_role()

    pig = [0, 0, 0, 0]

    pig[0] = bw_l_motor_get_pwm("ccw")
    pig[1] = bw_r_motor_get_pwm("ccw")
    pig[2] = fw_l_motor_get_pwm("ccw")
    pig[3] = fw_r_motor_get_pwm("ccw")

    motor_move_with_speed(pig, speed, 1.0)


@webiopi.macro
def servo_test(val):
    vlist = val.split("_")
    # 				  idx           pos
    set_servo_pos(int(vlist[0]), int(vlist[1]))
    commit_servo(servo_pos)
    return val


def servo_parse_data(val):
    valist = val.split(":")
    dct = {}
    for vv in valist:
        kk = vv.split("=")
        dct[kk[0]] = kk[1]
    return dct


@webiopi.macro
def servo_test_save(val):
    dct = servo_parse_data(val)
    path = dct["path"]
    with open(ROOT_PATH + path, mode="wb") as f:
        pickle.dump(dct, f)

    return dct


@webiopi.macro
def servo_test_load(val):
    dct = servo_parse_data(val)
    path = dct["path"]
    data = {}
    if not os.path.exists(ROOT_PATH + path):
        return "Error: not exist file!!"

    with open(ROOT_PATH + path, mode="rb") as fi:
        data = pickle.load(fi)
    # print(data)
    val = ""
    for kk in servo_key_list:
        val = val + kk + "=" + data[kk] + ":"

    val = val + "path=" + data["path"]
    if "wait" in data:
        val = val + ":wait=" + data["wait"]

    return val


@webiopi.macro
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


@webiopi.macro
def servo_motion_slow_stand_up():
    servo_idx_2nd_l = [
        SERVO_FW_2ND_JOINT_L_IDX,
        SERVO_FW_2ND_JOINT_R_IDX,
        SERVO_BW_2ND_JOINT_L_IDX,
        SERVO_BW_2ND_JOINT_R_IDX,
    ]
    servo_idx_1st_l = [
        SERVO_FW_1ST_JOINT_L_IDX,
        SERVO_FW_1ST_JOINT_R_IDX,
        SERVO_BW_1ST_JOINT_L_IDX,
        SERVO_BW_1ST_JOINT_R_IDX,
    ]

    servo_ini_pos_2nd_l = [0, 0, 0, 0]
    # 	j=0
    # 	for idx in servo_idx_2nd_l:
    # 		servo_ini_pos_2nd_l[j] = servo_convert_servo_deg(idx, servo_pos[idx])
    # 		j=j+1

    servo_mov_pos_1_2nd_l = [30, 30, 30, 30]
    servo_mov_pos_2_2nd_l = [40, 40, 40, 40]
    servo_mov_pos_a_2nd_l = [47, 48, 50, 49]

    servo_ini_pos_1st_l = [0, 0, 0, 0]
    servo_mov_pos_1_1st_l = [45, 45, 45, 45]

    servo_move_to_pos_list(
        servo_idx_2nd_l, servo_ini_pos_2nd_l, servo_mov_pos_1_2nd_l, 6
    )
    for idx in servo_idx_2nd_l:
        print(servo_pos[idx])
    servo_move_to_pos_list(
        servo_idx_1st_l, servo_ini_pos_1st_l, servo_mov_pos_1_1st_l, 2
    )
    servo_move_to_pos_list(
        servo_idx_2nd_l, servo_mov_pos_1_2nd_l, servo_mov_pos_2_2nd_l, 3
    )
    servo_move_to_pos_list(
        servo_idx_2nd_l, servo_mov_pos_a_2nd_l, servo_mov_pos_a_2nd_l, 3
    )

    set_servo_pos(8, 378)
    set_servo_pos(10, 258)
    set_servo_pos(14, 261)
    set_servo_pos(12, 376)

    set_servo_pos(2, 383)
    set_servo_pos(0, 266)
    set_servo_pos(6, 379)
    set_servo_pos(4, 379)

    commit_servo(servo_pos)


@webiopi.macro
def servo_motion_slow_sit_down():
    servo_idx_2nd_l = [
        SERVO_FW_2ND_JOINT_L_IDX,
        SERVO_FW_2ND_JOINT_R_IDX,
        SERVO_BW_2ND_JOINT_L_IDX,
        SERVO_BW_2ND_JOINT_R_IDX,
    ]
    servo_idx_1st_l = [
        SERVO_FW_1ST_JOINT_L_IDX,
        SERVO_FW_1ST_JOINT_R_IDX,
        SERVO_BW_1ST_JOINT_L_IDX,
        SERVO_BW_1ST_JOINT_R_IDX,
    ]

    servo_ini_pos_2nd_l = [0, 0, 0, 0]
    servo_mov_pos_2nd_l = [30, 30, 30, 30]
    servo_mov_pos_a_2nd_l = [47, 48, 50, 49]

    servo_ini_pos_1st_l = [0, 0, 0, 0]
    servo_mov_pos_a_1st_l = [5, 5, 5, 5]
    servo_mov_pos_1st_l = [45, 45, 45, 45]

    servo_move_to_pos_list(
        servo_idx_2nd_l, servo_mov_pos_a_2nd_l, servo_mov_pos_2nd_l, 6
    )
    servo_move_to_pos_list(
        servo_idx_1st_l, servo_mov_pos_1st_l, servo_mov_pos_a_1st_l, 10
    )
    servo_move_to_pos_list(
        servo_idx_1st_l, servo_mov_pos_a_1st_l, servo_ini_pos_1st_l, 1
    )
    servo_move_to_pos_list(servo_idx_2nd_l, servo_mov_pos_2nd_l, servo_ini_pos_2nd_l, 6)

    servo_reset(0)


def servo_quick_stand_up():
    servo_idx_2nd_l = [
        SERVO_FW_2ND_JOINT_L_IDX,
        SERVO_FW_2ND_JOINT_R_IDX,
        SERVO_BW_2ND_JOINT_L_IDX,
        SERVO_BW_2ND_JOINT_R_IDX,
    ]
    servo_idx_1st_l = [
        SERVO_FW_1ST_JOINT_L_IDX,
        SERVO_FW_1ST_JOINT_R_IDX,
        SERVO_BW_1ST_JOINT_L_IDX,
        SERVO_BW_1ST_JOINT_R_IDX,
    ]

    servo_ini_pos_2nd_l = [20, 20, 20, 20]
    servo_ini_pos_2nd_a_l = [45, 45, 45, 45]

    servo_mov_pos_1st_l = [45, 45, 45, 45]

    servo_move_to_pos_list(servo_idx_1st_l, servo_mov_pos_1st_l, servo_mov_pos_1st_l, 0)
    time.sleep(0.1)

    j = 0
    for idx in servo_idx_2nd_l:
        servo_ini_pos_2nd_l[j] = servo_get_current_deg(idx)
        j = j + 1

    servo_move_to_pos_list(servo_idx_2nd_l, servo_ini_pos_2nd_l, servo_ini_pos_2nd_l, 0)
    time.sleep(0.5)
    servo_move_to_pos_list(
        servo_idx_2nd_l, servo_ini_pos_2nd_a_l, servo_ini_pos_2nd_a_l, 0
    )


def servo_quick_sit_down():
    servo_idx_2nd_l = [
        SERVO_FW_2ND_JOINT_L_IDX,
        SERVO_FW_2ND_JOINT_R_IDX,
        SERVO_BW_2ND_JOINT_L_IDX,
        SERVO_BW_2ND_JOINT_R_IDX,
    ]
    servo_idx_1st_l = [
        SERVO_FW_1ST_JOINT_L_IDX,
        SERVO_FW_1ST_JOINT_R_IDX,
        SERVO_BW_1ST_JOINT_L_IDX,
        SERVO_BW_1ST_JOINT_R_IDX,
    ]

    servo_ini_pos_2nd_l = [0, 0, 0, 0]

    servo_ini_pos_1st_l = [0, 0, 0, 0]
    servo_mov_pos_1st_l = [45, 45, 45, 45]

    servo_move_to_pos_list(servo_idx_1st_l, servo_mov_pos_1st_l, servo_mov_pos_1st_l, 0)
    servo_move_to_pos_list(servo_idx_2nd_l, servo_ini_pos_2nd_l, servo_ini_pos_2nd_l, 0)
    time.sleep(0.1)
    servo_move_to_pos_list(servo_idx_1st_l, servo_ini_pos_1st_l, servo_ini_pos_1st_l, 0)

    servo_reset(0)


@webiopi.macro
def servo_motion_open(val):
    servo_motion(0)

    set_servo_pos(SERVO_FW_1ST_JOINT_L_IDX, SERVO_FW_1ST_JOINT_L_OPEN_POS)
    set_servo_pos(SERVO_FW_1ST_JOINT_R_IDX, SERVO_FW_1ST_JOINT_R_OPEN_POS)
    set_servo_pos(SERVO_BW_1ST_JOINT_L_IDX, SERVO_BW_1ST_JOINT_L_OPEN_POS)
    set_servo_pos(SERVO_BW_1ST_JOINT_R_IDX, SERVO_BW_1ST_JOINT_R_OPEN_POS)

    commit_servo(servo_pos)
    return val


@webiopi.macro
def servo_set_pos(val):
    vlist = val.split("_")
    # 				  idx           pos
    set_servo_pos(int(vlist[0]), int(vlist[1]))
    return val


@webiopi.macro
def servo_set_pos_list(val):
    dct = servo_parse_data(val)
    servo_motion_dict_list.append(dct)
    # 	for kk in dct.keys():
    # 		set_servo_pos(servo_idx_dict[kk],int(dct[kk]))

    return dct


@webiopi.macro
def servo_clear_pos_list(val):
    servo_motion_dict_list.clear()

    return "remove all of list"


def servo_exec_pos_dict(dct):
    nn = 1
    if "exec_num" in dct:
        nn = int(dct["exec_num"])
    i = 0
    wt = 0
    while i < nn:
        for dict in dct:
            for kk in dict.keys():
                if kk in servo_idx_dict:
                    set_servo_pos(servo_idx_dict[kk], int(dict[kk]))
            commit_servo(servo_pos)  # must commit, here!!
            webiopi.sleep(0.1)
            if "wait" in dict:
                wt = int(dict["wait"])
            webiopi.sleep(wt / 1000)
        i = i + 1

    return dct


@webiopi.macro
def servo_exec_pos_list(val):
    dct = servo_parse_data(val)
    print(dct)
    print(servo_motion_dict_list)

    dct = servo_exec_pos_dict(servo_motion_dict_list)
    servo_motion_dict_list.clear()


@webiopi.macro
def servo_run(val):
    commit_servo(servo_pos)
    return val


def servo_demo1():
    # 187 - 449
    reg = SERVO_FW_2ND_JOINT_R_CLOSE_POS
    servo_set_pwm(SERVO_FW_2ND_JOINT_R_IDX, reg)
    while reg < SERVO_FW_2ND_JOINT_R_OPEN_POS:
        servo_set_pwm(SERVO_FW_2ND_JOINT_R_IDX, reg)
        time.sleep(0.03)
        reg = reg + 4
    while reg > SERVO_FW_2ND_JOINT_R_CLOSE_POS:
        servo_set_pwm(SERVO_FW_2ND_JOINT_R_IDX, reg)
        time.sleep(0.03)
        reg = reg - 4

    reg = SERVO_BW_2ND_JOINT_L_CLOSE_POS
    servo_set_pwm(SERVO_BW_2ND_JOINT_L_IDX, reg)
    while reg < SERVO_BW_2ND_JOINT_L_OPEN_POS:
        servo_set_pwm(SERVO_BW_2ND_JOINT_L_IDX, reg)
        time.sleep(0.03)
        reg = reg + 4
    while reg > SERVO_BW_2ND_JOINT_L_CLOSE_POS:
        servo_set_pwm(SERVO_BW_2ND_JOINT_L_IDX, reg)
        time.sleep(0.03)
        reg = reg - 4


def servo_demo2():
    dct = servo_test_load("xxx=xxx:path=walk000")
    # print(dct)
    servo_set_pos_list(dct)

    dct = servo_test_load("xxx=xxx:path=walk001")
    servo_set_pos_list(dct)
    dct = servo_test_load("xxx=xxx:path=walk002")
    servo_set_pos_list(dct)
    dct = servo_test_load("xxx=xxx:path=walk003")
    servo_set_pos_list(dct)
    dct = servo_test_load("xxx=xxx:path=walk004")
    servo_set_pos_list(dct)
    dct = servo_test_load("xxx=xxx:path=walk005")
    servo_set_pos_list(dct)

    for dc in servo_motion_dict_list:
        for kk in dc.keys():
            if kk in servo_idx_dict:
                print(
                    kk
                    + ":"
                    + str(servo_convert_servo_deg(servo_idx_dict[kk], int(dc[kk])))
                )

    servo_exec_pos_list("xxx=xxx:exec_num=10")


def servo_convert_servo_val(idx, value):
    if servo_vec[idx] > 0:
        return int(
            servo_pos_min[idx]
            + (value * (servo_pos_max[idx] - servo_pos_min[idx]) / servo_range[idx])
        )
    else:
        return int(
            servo_pos_max[idx]
            - (value * (servo_pos_max[idx] - servo_pos_min[idx]) / servo_range[idx])
        )


def servo_convert_servo_deg(idx, value):
    if servo_vec[idx] > 0:
        return int(
            (value - servo_pos_min[idx])
            * servo_range[idx]
            / (servo_pos_max[idx] - servo_pos_min[idx])
        )
    else:
        return int(
            (servo_pos_max[idx] - value)
            * servo_range[idx]
            / (servo_pos_max[idx] - servo_pos_min[idx])
        )


def servo_move_to_pos(idx, init_pos, move_pos, delta_pos):
    regv = init_pos
    if init_pos < move_pos:
        while regv <= move_pos:
            servo_set_pwm(idx, servo_convert_servo_val(idx, regv))
            time.sleep(0.03)
            regv = regv + delta_pos
    elif init_pos > move_pos:
        while regv >= move_pos:
            servo_set_pwm(idx, servo_convert_servo_val(idx, regv))
            time.sleep(0.03)
            regv = regv - delta_pos


def servo_move_to_pos_list(idx_list, init_pos_list, move_pos_list, delta_pos):
    reg = list(init_pos_list)
    idx_l = list(idx_list)
    reg_vec = list(move_pos_list)

    idx = 0
    if delta_pos == 0:
        for regv in reg:
            servo_set_pwm(idx_l[idx], servo_convert_servo_val(idx_l[idx], regv))
            idx = idx + 1
        return

    idx = 0
    for regv in reg:
        servo_set_pwm(idx_l[idx], servo_convert_servo_val(idx_l[idx], regv))
        if regv < reg_vec[idx]:
            reg_vec[idx] = 1
        elif regv > reg_vec[idx]:
            reg_vec[idx] = -1
        else:
            return
        idx = idx + 1

    idx = 0
    fin = False
    while not fin:
        idx = 0
        for regv in reg:
            servo_set_pwm(idx_l[idx], servo_convert_servo_val(idx_l[idx], regv))
            time.sleep(0.03)
            regv = regv + delta_pos * reg_vec[idx]
            reg[idx] = regv
            if reg_vec[idx] > 0:
                if reg[idx] < move_pos_list[idx]:
                    fin = False
                else:
                    fin = True
            elif reg_vec[idx] < 0:
                if reg[idx] > move_pos_list[idx]:
                    fin = False
                else:
                    fin = True
            else:
                return
            idx = idx + 1


def servo_demo3():
    # 187 - 449
    reg_close = [30, 30, 30, 30]
    reg = [45, 45, 45, 45]
    reg_open = [180, 180, 180, 180]
    reg_open2 = [45, 45, 45, 45]

    idx_l = [
        SERVO_FW_2ND_JOINT_R_IDX,
        SERVO_FW_2ND_JOINT_L_IDX,
        SERVO_BW_2ND_JOINT_R_IDX,
        SERVO_BW_2ND_JOINT_L_IDX,
    ]
    idx_1st_l = [
        SERVO_FW_1ST_JOINT_R_IDX,
        SERVO_FW_1ST_JOINT_L_IDX,
        SERVO_BW_1ST_JOINT_R_IDX,
        SERVO_BW_1ST_JOINT_L_IDX,
    ]

    servo_move_to_pos_list(idx_1st_l, reg, reg, 0)
    time.sleep(0.5)
    j = 0
    for idx in idx_l:
        reg[j] = servo_get_current_deg(idx)
        j = j + 1
    servo_move_to_pos_list(idx_l, reg, reg_open, 3)


# 	servo_move_to_pos_list(idx_l, reg_close, reg_open2, 6)


def servo_demo4():
    servo_move_to_pos(SERVO_BW_2ND_JOINT_L_IDX, 45, 105, 10)

    servo_move_to_pos(SERVO_FW_2ND_JOINT_R_IDX, 0, 180, 2)
    servo_move_to_pos(SERVO_FW_1ST_JOINT_R_IDX, 0, 90, 10)
    servo_move_to_pos(SERVO_FW_1ST_JOINT_R_IDX, 90, 0, 10)
    servo_move_to_pos(SERVO_FW_1ST_JOINT_R_IDX, 0, 90, 10)
    servo_move_to_pos(SERVO_FW_1ST_JOINT_R_IDX, 90, 0, 10)
    servo_move_to_pos(SERVO_FW_1ST_JOINT_R_IDX, 0, 90, 10)
    servo_move_to_pos(SERVO_FW_1ST_JOINT_R_IDX, 90, 0, 10)
    servo_move_to_pos(SERVO_FW_1ST_JOINT_R_IDX, 0, 90, 10)
    servo_move_to_pos(SERVO_FW_1ST_JOINT_R_IDX, 90, 0, 10)
    servo_move_to_pos(SERVO_FW_1ST_JOINT_R_IDX, 0, 90, 10)
    servo_move_to_pos(SERVO_FW_1ST_JOINT_R_IDX, 90, 0, 10)
    servo_move_to_pos(SERVO_FW_1ST_JOINT_R_IDX, 0, 45, 10)
    servo_move_to_pos(SERVO_FW_2ND_JOINT_R_IDX, 180, 45, 2)


def servo_demo7():
    idx_l = [
        SERVO_FW_2ND_JOINT_R_IDX,
        SERVO_BW_2ND_JOINT_L_IDX,
        SERVO_FW_2ND_JOINT_L_IDX,
        SERVO_BW_2ND_JOINT_R_IDX,
        SERVO_FW_1ST_JOINT_R_IDX,
        SERVO_BW_1ST_JOINT_L_IDX,
        SERVO_FW_1ST_JOINT_L_IDX,
        SERVO_BW_1ST_JOINT_R_IDX,
    ]
    pos_l = [0, 0, 0, 0, 0, 0, 0, 0]

    i = 0
    for idx in idx_l:
        pos_l[i] = servo_get_current_deg(idx)
        i = i + 1

    i = 0
    servo_move_to_pos(SERVO_FW_2ND_JOINT_R_IDX, pos_l[0], 80, 5)  # up leg
    servo_move_to_pos(SERVO_BW_2ND_JOINT_L_IDX, pos_l[1], 80, 5)  # up leg
    servo_move_to_pos(SERVO_FW_2ND_JOINT_L_IDX, pos_l[2], 80, 5)  # up leg
    servo_move_to_pos(SERVO_BW_2ND_JOINT_R_IDX, pos_l[3], 80, 5)  # up leg
    servo_move_to_pos(SERVO_FW_1ST_JOINT_R_IDX, pos_l[4], 40, 5)
    servo_move_to_pos(SERVO_BW_1ST_JOINT_L_IDX, pos_l[5], 40, 5)
    servo_move_to_pos(SERVO_FW_1ST_JOINT_L_IDX, pos_l[6], 40, 5)
    servo_move_to_pos(SERVO_BW_1ST_JOINT_R_IDX, pos_l[7], 40, 5)
    while i < 6:
        servo_move_to_pos(SERVO_FW_2ND_JOINT_R_IDX, 75, 110, 30)  # up leg
        servo_move_to_pos(SERVO_BW_2ND_JOINT_L_IDX, 75, 110, 30)  # up leg
        servo_move_to_pos(SERVO_FW_1ST_JOINT_R_IDX, 40, 10, 10)
        servo_move_to_pos(SERVO_BW_1ST_JOINT_L_IDX, 40, 10, 10)
        servo_move_to_pos(SERVO_FW_1ST_JOINT_L_IDX, 40, 10, 10)  # role
        servo_move_to_pos(SERVO_BW_1ST_JOINT_R_IDX, 40, 10, 10)  # role
        servo_move_to_pos(SERVO_FW_2ND_JOINT_R_IDX, 110, 75, 30)  # down leg
        servo_move_to_pos(SERVO_BW_2ND_JOINT_L_IDX, 110, 75, 30)  # down leg
        time.sleep(0.1)

        servo_move_to_pos(SERVO_FW_2ND_JOINT_L_IDX, 75, 110, 30)  # up leg
        servo_move_to_pos(SERVO_BW_2ND_JOINT_R_IDX, 75, 110, 30)  # up leg
        servo_move_to_pos(SERVO_FW_1ST_JOINT_L_IDX, 10, 40, 10)
        servo_move_to_pos(SERVO_BW_1ST_JOINT_R_IDX, 10, 40, 10)
        servo_move_to_pos(SERVO_FW_1ST_JOINT_R_IDX, 10, 40, 10)  # role
        servo_move_to_pos(SERVO_BW_1ST_JOINT_L_IDX, 10, 40, 10)  # role
        servo_move_to_pos(SERVO_FW_2ND_JOINT_L_IDX, 110, 75, 30)  # down leg
        servo_move_to_pos(SERVO_BW_2ND_JOINT_R_IDX, 110, 75, 30)  # down leg
        time.sleep(0.1)

        i = i + 1
    servo_move_to_pos(SERVO_FW_1ST_JOINT_R_IDX, 40, 45, 15)
    servo_move_to_pos(SERVO_BW_1ST_JOINT_L_IDX, 40, 45, 15)
    servo_move_to_pos(SERVO_FW_1ST_JOINT_L_IDX, 40, 45, 15)
    servo_move_to_pos(SERVO_BW_1ST_JOINT_R_IDX, 40, 45, 15)


def servo_demo8():
    idx_l = [
        SERVO_FW_2ND_JOINT_R_IDX,
        SERVO_BW_2ND_JOINT_L_IDX,
        SERVO_FW_2ND_JOINT_L_IDX,
        SERVO_BW_2ND_JOINT_R_IDX,
        SERVO_FW_1ST_JOINT_R_IDX,
        SERVO_BW_1ST_JOINT_L_IDX,
        SERVO_FW_1ST_JOINT_L_IDX,
        SERVO_BW_1ST_JOINT_R_IDX,
    ]
    pos_l = [0, 0, 0, 0, 0, 0, 0, 0]

    i = 0
    for idx in idx_l:
        pos_l[i] = servo_get_current_deg(idx)
        i = i + 1

    i = 0
    servo_move_to_pos(SERVO_FW_2ND_JOINT_R_IDX, pos_l[0], 80, 5)  # up leg
    servo_move_to_pos(SERVO_BW_2ND_JOINT_L_IDX, pos_l[1], 80, 5)  # up leg
    servo_move_to_pos(SERVO_FW_2ND_JOINT_L_IDX, pos_l[2], 80, 5)  # up leg
    servo_move_to_pos(SERVO_BW_2ND_JOINT_R_IDX, pos_l[3], 80, 5)  # up leg
    servo_move_to_pos(SERVO_FW_1ST_JOINT_R_IDX, pos_l[4], 10, 5)
    servo_move_to_pos(SERVO_BW_1ST_JOINT_L_IDX, pos_l[5], 10, 5)
    servo_move_to_pos(SERVO_FW_1ST_JOINT_L_IDX, pos_l[6], 10, 5)
    servo_move_to_pos(SERVO_BW_1ST_JOINT_R_IDX, pos_l[7], 10, 5)
    while i < 6:
        servo_move_to_pos(SERVO_FW_2ND_JOINT_R_IDX, 75, 110, 30)  # up leg
        servo_move_to_pos(SERVO_BW_2ND_JOINT_L_IDX, 75, 110, 30)  # up leg
        servo_move_to_pos(SERVO_FW_1ST_JOINT_R_IDX, 10, 40, 10)
        servo_move_to_pos(SERVO_BW_1ST_JOINT_L_IDX, 10, 40, 10)
        servo_move_to_pos(SERVO_FW_1ST_JOINT_L_IDX, 10, 40, 10)  # role
        servo_move_to_pos(SERVO_BW_1ST_JOINT_R_IDX, 10, 40, 10)  # role
        servo_move_to_pos(SERVO_FW_2ND_JOINT_R_IDX, 110, 75, 30)  # down leg
        servo_move_to_pos(SERVO_BW_2ND_JOINT_L_IDX, 110, 75, 30)  # down leg
        time.sleep(0.1)

        servo_move_to_pos(SERVO_FW_2ND_JOINT_L_IDX, 75, 110, 30)  # up leg
        servo_move_to_pos(SERVO_BW_2ND_JOINT_R_IDX, 75, 110, 30)  # up leg
        servo_move_to_pos(SERVO_FW_1ST_JOINT_L_IDX, 40, 10, 10)
        servo_move_to_pos(SERVO_BW_1ST_JOINT_R_IDX, 40, 10, 10)
        servo_move_to_pos(SERVO_FW_1ST_JOINT_R_IDX, 40, 10, 10)  # role
        servo_move_to_pos(SERVO_BW_1ST_JOINT_L_IDX, 40, 10, 10)  # role
        servo_move_to_pos(SERVO_FW_2ND_JOINT_L_IDX, 110, 75, 30)  # down leg
        servo_move_to_pos(SERVO_BW_2ND_JOINT_R_IDX, 110, 75, 30)  # down leg
        time.sleep(0.1)

        i = i + 1
    servo_move_to_pos(SERVO_FW_1ST_JOINT_R_IDX, 10, 45, 15)
    servo_move_to_pos(SERVO_BW_1ST_JOINT_L_IDX, 10, 45, 15)
    servo_move_to_pos(SERVO_FW_1ST_JOINT_L_IDX, 10, 45, 15)
    servo_move_to_pos(SERVO_BW_1ST_JOINT_R_IDX, 10, 45, 15)


def servo_demo9():
    servo_move_to_pos(SERVO_MISSILE_IDX, 90, 0, 10)
    servo_move_to_pos(SERVO_MISSILE_IDX, 0, 180, 10)
    servo_move_to_pos(SERVO_MISSILE_IDX, 180, 90, 10)


def servo_get_current_deg(idx):
    if servo_pos[idx] > 0:
        return servo_convert_servo_deg(idx, servo_pos[idx])
    else:
        return 45


def servo_and_motor_move(idx_l, role_mode_l, move_pos_l):
    init_pos = list(idx_l)
    mov_pos = list(idx_l)
    pig = list(idx_l)

    j = 0
    for idx in idx_l:
        init_pos[j] = servo_get_current_deg(idx)
        mov_pos[j] = move_pos_l[j]
        j = j + 1

    # print(init_pos)

    j = 0
    for idx in idx_l:
        if idx == SERVO_FW_1ST_JOINT_L_IDX:
            pig[j] = fw_l_motor_get_pwm(role_mode_l[j])
        elif idx == SERVO_FW_1ST_JOINT_R_IDX:
            pig[j] = fw_r_motor_get_pwm(role_mode_l[j])
        elif idx == SERVO_BW_1ST_JOINT_L_IDX:
            pig[j] = bw_l_motor_get_pwm(role_mode_l[j])
        elif idx == SERVO_BW_1ST_JOINT_R_IDX:
            pig[j] = bw_r_motor_get_pwm(role_mode_l[j])
        j = j + 1

    for pigv in pig:
        pigv.start(15)
        pigv.ChangeDutyCycle(15)

    servo_move_to_pos_list(idx_l, init_pos, mov_pos, 3)


def servo_open_for_wheel():
    servo_idx_1st_l = [SERVO_FW_1ST_JOINT_L_IDX, SERVO_FW_1ST_JOINT_R_IDX]
    motor_role_mode = ["cw", "ccw"]
    move_pos_l = [90, 90]

    servo_and_motor_move(servo_idx_1st_l, motor_role_mode, move_pos_l)

    servo_idx_1st_l = [SERVO_BW_1ST_JOINT_L_IDX, SERVO_BW_1ST_JOINT_R_IDX]
    motor_role_mode = ["ccw", "cw"]

    servo_and_motor_move(servo_idx_1st_l, motor_role_mode, move_pos_l)


def servo_open_for_walk():
    servo_idx_1st_l = [SERVO_FW_1ST_JOINT_L_IDX, SERVO_FW_1ST_JOINT_R_IDX]
    motor_role_mode = ["ccw", "cw"]
    move_pos_l = [45, 45]

    servo_and_motor_move(servo_idx_1st_l, motor_role_mode, move_pos_l)

    servo_idx_1st_l = [SERVO_BW_1ST_JOINT_L_IDX, SERVO_BW_1ST_JOINT_R_IDX]
    motor_role_mode = ["cw", "ccw"]

    servo_and_motor_move(servo_idx_1st_l, motor_role_mode, move_pos_l)


def servo_walking():
    servo_walking_list_dict = [
        {
            "fw_1st_l": 45,
            "fw_1st_r": 90,
            "fw_2nd_l": 80,
            "fw_2nd_r": 80,
            "bw_1st_l": 45,
            "bw_1st_r": 90,
            "bw_2nd_l": 80,
            "bw_2nd_r": 80,
        },
        {
            "fw_1st_r": 45,
            "fw_2nd_r": 100,
            "bw_1st_r": 85,
        },
        {
            "bw_2nd_l": 99,
            "bw_2nd_r": 80,
        },
        {
            "fw_1st_l": 90,
            "bw_1st_r": 30,
        },
        {
            "fw_1st_l": 90,
            "fw_1st_r": 45,
            "fw_2nd_l": 80,
            "fw_2nd_r": 80,
            "bw_1st_l": 90,
            "bw_1st_r": 45,
            "bw_2nd_l": 80,
            "bw_2nd_r": 80,
        },
        {
            "fw_1st_l": 45,
            "fw_2nd_l": 100,
            "fw_2nd_r": 90,
        },
        {
            "bw_2nd_l": 80,
            "bw_2nd_r": 99,
        },
        {
            "fw_1st_r": 90,
            "bw_1st_l": 30,
        },
    ]

    i = 0
    nn = 4
    while i < nn:
        for dct in servo_walking_list_dict:
            for kk in dct:
                servo_set_pwm(
                    servo_idx_dict[kk],
                    servo_convert_servo_val(servo_idx_dict[kk], dct[kk]),
                )
            time.sleep(0.2)
        i = i + 1


def servo_walking_reverse():
    servo_walking_list_dict = [
        {
            "fw_1st_l": 90,
            "fw_2nd_l": 80,
            "fw_1st_r": 45,
            "fw_2nd_r": 80,
            "bw_1st_l": 90,
            "bw_2nd_l": 80,
            "bw_1st_r": 45,
            "bw_2nd_r": 80,
        },
        {
            "bw_1st_l": 45,
            "bw_2nd_l": 100,
            "fw_1st_l": 85,
        },
        {
            "fw_2nd_r": 99,
            "fw_2nd_l": 80,
        },
        {
            "bw_1st_r": 90,
            "fw_1st_l": 30,
        },
        {
            "fw_1st_l": 45,
            "fw_2nd_l": 80,
            "fw_1st_r": 90,
            "fw_2nd_r": 80,
            "bw_1st_l": 45,
            "bw_2nd_l": 80,
            "bw_1st_r": 90,
            "bw_2nd_r": 80,
        },
        {
            "bw_1st_r": 45,
            "bw_2nd_r": 100,
            "bw_2nd_l": 90,
        },
        {
            "bw_1st_r": 45,
            "bw_2nd_r": 100,
            "bw_2nd_l": 90,
        },
        {
            "fw_2nd_r": 80,
            "fw_2nd_l": 99,
            "bw_1st_l": 90,
            "fw_1st_r": 30,
        },
    ]

    i = 0
    nn = 4
    while i < nn:
        for dct in servo_walking_list_dict:
            for kk in dct:
                servo_set_pwm(
                    servo_idx_dict[kk],
                    servo_convert_servo_val(servo_idx_dict[kk], dct[kk]),
                )
            time.sleep(0.2)
        i = i + 1


@webiopi.macro
def main_arm_proc(move):
    if move is "q":
        print("q")
    elif move is "m":
        # servo_motion(0)
        servo_quick_stand_up()
    elif move is "M":
        servo_quick_sit_down()
    elif move is "w":
        servo_walking()
    elif move is "W":
        servo_walking_reverse()
    elif move is "1":
        servo_demo1()
    elif move is "2":
        servo_demo2()
    elif move is "3":
        servo_demo3()
    elif move is "4":
        servo_demo4()
    elif move is "5":
        servo_motion_slow_stand_up()
    elif move is "6":
        servo_motion_slow_sit_down()
    elif move is "7":
        servo_demo7()
    elif move is "8":
        servo_demo8()
    elif move is "9":
        servo_demo9()
    elif move is "l":
        fw_l_motor("cw")
    elif move is "L":
        fw_l_motor("ccw")
    elif move is "r":
        fw_r_motor("cw")
    elif move is "R":
        fw_r_motor("ccw")
    elif move is "t":
        motor_demo0()
    elif move is "k":
        bw_l_motor("cw")
    elif move is "K":
        bw_l_motor("ccw")
    elif move is "e":
        bw_r_motor("cw")
    elif move is "E":
        bw_r_motor("ccw")
    elif move is "T":
        motor_demo1()
    elif move is "y":
        motor_move_foward(2.5 * 10)
    elif move is "Y":
        motor_move_backward(2.5 * 10)
    elif move is "u":
        motor_move_foward(50)
    elif move is "U":
        motor_move_backward(50)
    elif move is "i":
        motor_move_foward(100)
    elif move is "I":
        motor_move_backward(100)
    elif move is "o":
        servo_open_for_wheel()
    elif move is "O":
        servo_open_for_walk()
    elif move is "p":
        motor_role_left(15)
    elif move is "P":
        motor_role_right(15)
    elif move is "h":
        print("quit:q")
        print("m:quick stand up")
        print("M:quick sit down")
        print("1:leg up and down")
        print("2:walking")
        print("3:all leg up and down")
        print("4:bye bye")
        print("5:stand up")
        print("6:sit down")
        print("7:role right")
        print("8:role left")
        print("l:fw  left motor cw, L:ccw")
        print("r:fw right motor cw, R:ccw")
        print("k:bw  left motor cw, K:ccw")
        print("e:bw right motor cw, E:ccw")
        print("o:open for wheel")
        print("O:open for walk")
        print("h:help")
    # commit_servo(servo_pos)


if __name__ == "__main__":
    init_motor()
    # servo_motion_slow_stand_up()
    # servo_motion(0)
    time.sleep(1)

    while True:
        move = getch()  # input("move [close:c open:o]")
        main_arm_proc(move)
        if move is "q":
            break

    # servo_motion_slow_sit_down()
    # servo_reset(0)
    GPIO.cleanup()
