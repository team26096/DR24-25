#!/usr/bin/env python3

import hub
import sys
import time

import color, color_sensor, device, motor, motor_pair, orientation, runloop
from hub import light_matrix, button, motion_sensor, light, sound, port


# CONSTANTS
#----------------------------------------

WHEEL_CIRCUMFERENCE = 17.584

WHITE_COLOR_INTENSITY_MIN = 97
BLACK_COLOR_INTENSITY_MAX = 18

COLOR_SENSOR_CENTER_PORT = port.C
COLOR_SENSOR_LEFT_PORT = port.D

# END CONSTANTS
#----------------------------------------


# COMMON FUNCTIONS
#----------------------------------------

# initialize motor and reset yaw
async def doInit():
    # reset yaw to 0
    motion_sensor.set_yaw_face(motion_sensor.TOP)
    motion_sensor.reset_yaw(0)
    i = 0
    while (hub.motion_sensor.stable() == False):
        i = i + 1
        await runloop.sleep_ms(10)
        hub.light_matrix.write(str(i))
        if i >= 100:
            break


# Return true if LEFT button is pressed
def is_left_button_pressed():
    return button.pressed(button.LEFT) > 0


# Return true if RIGHT button is pressed
def is_right_button_pressed():
    return button.pressed(button.RIGHT) > 0


def follow_for_distance(initial_position=0,
                        distance_to_cover=0):
    current_position = abs(motor.relative_position(port.A))
    distance_covered = current_position - initial_position
    if distance_covered < 0 : distance_covered = distance_covered * -1
    if (distance_covered >= abs(distance_to_cover)):
        return False
    else:
        return True


def get_color_values():
    return color_sensor.reflection(COLOR_SENSOR_CENTER_PORT), color_sensor.reflection(COLOR_SENSOR_LEFT_PORT)


def follow_for_color_white_center():
    return get_color_values()[0] <= WHITE_COLOR_INTENSITY_MIN


def follow_for_color_black_center():
    return get_color_values()[0] >= BLACK_COLOR_INTENSITY_MAX


def follow_for_color_white_left():
    return get_color_values()[1] <= WHITE_COLOR_INTENSITY_MIN


def follow_for_color_black_left():
    return get_color_values()[1] >= BLACK_COLOR_INTENSITY_MAX


def get_yaw_value():
    return motion_sensor.tilt_angles()[0] * -0.1


def degreesForDistance(distance_cm):
    # Add multiplier for gear ratio if needed
    return int((distance_cm/WHEEL_CIRCUMFERENCE) * 360)


def wait_for_yaw_abs(angle=0):
    abs_angle = abs(angle)
    abs_current_yaw = abs(get_yaw_value())
    if angle == 0:
        if get_yaw_value() > 0:
            while get_yaw_value() >= angle: runloop.sleep_ms(10)
        elif get_yaw_value() < 0:
            while get_yaw_value() <= angle: runloop.sleep_ms(10)
    elif abs_current_yaw > abs_angle:
        while abs(get_yaw_value()) >= abs_angle: runloop.sleep_ms(10)
    elif abs_current_yaw < abs_angle:
        while abs(get_yaw_value()) <= abs_angle: runloop.sleep_ms(10)


async def follow_gyro_angle(kp,
                            ki,
                            kd,
                            speed,
                            target_angle,
                            sleep_time,
                            follow_for, **kwargs):
    # get initial reading from left motor
    integral = 0.0
    last_error = 0.0
    derivative = 0.0
    while (follow_for(**kwargs)):
        current_angle = get_yaw_value()
        error = current_angle - target_angle
        integral = integral + error
        derivative = error - last_error
        last_error = error
        # compute steering correction
        steering_value = (error * kp) + (integral * ki) + (derivative * kd)

        if sleep_time:
            runloop.sleep_ms(sleep_time)
        # kp value should be +ve for forward movement (positive speed value), and -ve for backward movement (negative speed value)
        motor_pair.move(motor_pair.PAIR_1, int(steering_value), velocity=speed)

    # stop when follow_for condition is met
    motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)


async def pivot_gyro_turn_abs(left_speed=0, right_speed=50, angle=90, stop=False):
    motor_pair.move_tank(motor_pair.PAIR_1, left_speed, right_speed)
    # print("pivot_gyro_turn - " + "target angle=" + str(angle) + "current angle ="+ str(get_yaw_value()))
    wait_for_yaw_abs(angle=angle)
    if stop: motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)


async def turn_left(speed=50, angle=90, stop=True):
    await pivot_gyro_turn_abs(left_speed=0, right_speed=speed, angle=angle, stop=stop)


async def turn_right(speed=-50, angle=90, stop=True):
    await pivot_gyro_turn_abs(left_speed=speed, right_speed=0, angle=angle, stop=stop)


def get_time_taken_in_seconds(start_time, end_time):
    return int(time.ticks_diff(end_time, start_time)/1000)

# END COMMON FUNCTIONS
#----------------------------------------

#---------------------------------------------------------------------------------------------------------------------------------------------------------------

# RUN FUNCTIONS
#----------------------------------------

# RUN 1A
#----------------------------------------
async def run1a():

    # go backward to collect krill
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-400, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degreesForDistance(5)))

    # turn left to get in alignment with krill
    await pivot_gyro_turn_abs(left_speed=-200, right_speed=200, angle=-45, stop=True)

    # go backward to collect krill
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-400, target_angle=-45, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degreesForDistance(20)))

    # turn right to get in alignment with neon pink coral pieces
    await pivot_gyro_turn_abs(left_speed=200, right_speed=-200, angle=0, stop=True)

    # go backward to collect coral piece and krill
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-400, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degreesForDistance(45)))

    # turn right to collect last krill
    await pivot_gyro_turn_abs(left_speed=200, right_speed=-200, angle=45, stop=True)

    # go forward to collect last krill
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-400, target_angle=45, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degreesForDistance(8)))

    # turn left to align with plankton
    await pivot_gyro_turn_abs(left_speed=-100, right_speed=100, angle=-90, stop=True)

    motor_pair.move_for_degrees(motor_pair.PAIR_1, degreesForDistance(6),0,velocity=200)

    # sleep to make sure plakton gets into the one-way gate
    await runloop.sleep_ms(500)

    # go forward to pull the plankton
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-400, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degreesForDistance(4)))

    # turn to align with plankton
    await pivot_gyro_turn_abs(left_speed=-100, right_speed=100, angle=-5, stop=True)

    # go forward to pull the plankton
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-400, target_angle=-5, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degreesForDistance(15)))

    # go forward to pull the plankton
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-800, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degreesForDistance(90)))

    # turn to align with seabed sample
    await pivot_gyro_turn_abs(left_speed=-100, right_speed=100, angle=-19, stop=True)

    # go forward while collecting krill and plankton and sample
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-800, target_angle=-19, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degreesForDistance(25)))

    # turn to collect new krill and plankton and water sample
    await pivot_gyro_turn_abs(left_speed=-100, right_speed=100, angle=-90, stop=True)

    # go forward with all the krill and plankton and samples
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-800, target_angle=-90, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degreesForDistance(30)))

    # turn to get in positon to get into base
    await pivot_gyro_turn_abs(left_speed=100, right_speed=-100, angle=-45, stop=True)

    # go forward to get in base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-800, target_angle=-45, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degreesForDistance(45)))

# END RUN 1A
#----------------------------------------

# RUN 1
#----------------------------------------
async def run1():

    # turn left to get in alignment with krill
    await pivot_gyro_turn_abs(left_speed=0, right_speed=-400, angle=75, stop=True)

    # go forward to collect krill
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-400, target_angle=75, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degreesForDistance(25)))

    # turn left to get in alignment with neon pink coral pieces
    await pivot_gyro_turn_abs(left_speed=200, right_speed=-200, angle=85, stop=True)

    # go forward to collect neon pink coral piece and krill
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-400, target_angle=85, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degreesForDistance(26)))

    # turn to collect last krill
    await pivot_gyro_turn_abs(left_speed=200, right_speed=-200, angle=110, stop=True)

    # go forward to collect last krill
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-400, target_angle=110, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degreesForDistance(17)))

    # turn to align with plankton
    await pivot_gyro_turn_abs(left_speed=-100, right_speed=100, angle=0, stop=True)

    # go backward to hook into plankton 
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=100, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
           initial_position=initial_position, distance_to_cover=(degreesForDistance(6)))

    # sleep to make sure plakton gets into the one-way gate
    await runloop.sleep_ms(500)

    # go forward to pull the plankton
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-400, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degreesForDistance(4)))
    
    # turn to align with plankton
    await pivot_gyro_turn_abs(left_speed=-100, right_speed=100, angle=-5, stop=True)

    # go forward to pull the plankton
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-400, target_angle=-5, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degreesForDistance(15)))
    
    # go forward to pull the plankton
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-800, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degreesForDistance(90)))

     # turn to align with seabed sample
    await pivot_gyro_turn_abs(left_speed=-100, right_speed=100, angle=-19, stop=True)

    # go forward while collecting krill and plankton and sample
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-800, target_angle=-19, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degreesForDistance(25)))

     # turn to collect new krill and plankton and water sample 
    await pivot_gyro_turn_abs(left_speed=-100, right_speed=100, angle=-90, stop=True)

    # go forward with all the krill and plankton and samples 
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-800, target_angle=-90, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degreesForDistance(30)))

    # turn to get in positon to get into base
    await pivot_gyro_turn_abs(left_speed=100, right_speed=-100, angle=-45, stop=True)

    # go forward to get in base 
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-800, target_angle=-45, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degreesForDistance(45)))

# END RUN 1
#----------------------------------------


# RUN 2
#----------------------------------------
async def run2():
    # go forward to take attatchment out of base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=500, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(10)))

    # go backward to get the attatchment flat
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-500, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(12)))

    # move forward to approach and move mission up
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degreesForDistance(47), 0, velocity=180)

    # Shake to ensure the hoop let's go of attatchment
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degreesForDistance(6), 0, velocity=400)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degreesForDistance(3), 0, velocity=-400)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degreesForDistance(3), 0, velocity=400)

    # go backward to get back to base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-600, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(44)))

    # turn left to get fully in base
    await turn_left(speed=100, angle=24, stop=True)

# END RUN 2
#----------------------------------------


# RUN 3
#----------------------------------------
# run 3 program - Raise the mast, Kraken's treasure, Diver Pickup, Coral buds
async def run3():

    # go straight to get out of base (backward)
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-600, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(18)))

    # turn right to get away from coral tree
    await pivot_gyro_turn_abs(left_speed=100, right_speed=-100, angle=30, stop=True)

    # go straight (backward) to align with shipwreck
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-600, target_angle=30, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(27)))

    # turn right to get in front of shipwreck
    await pivot_gyro_turn_abs(left_speed=100, right_speed=-100, angle=90, stop=True)

    # go straight to engage with shipwreck
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-150, target_angle=90, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(27)))

    # come back to collect treasure and release mast
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=300, target_angle=90, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(15)))

    # lower fork arm to get in position to pick up diver
    motor.run_for_degrees(port.B, 2075, -800)

    # turn left to prepare for alignment with coral tree
    await pivot_gyro_turn_abs(left_speed=-100, right_speed=100, angle=0, stop=True)

    # go forward to prepare for alignment with coral tree
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-600, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(15)))

    # turn right to get in alignment with coral tree
    await pivot_gyro_turn_abs(left_speed=150, right_speed=-150, angle=88, stop=True)

    # go straight to push coral tree buds
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degreesForDistance(14), 0, velocity=150)

    # reset yaw to 0
    motion_sensor.set_yaw_face(motion_sensor.TOP)
    motion_sensor.reset_yaw(0)
    await runloop.sleep_ms(1000)

    # lower Shark Hook to push the shark misson lever
    motor.run_for_degrees(port.C, 200, 700)

    # raise fork arm to pick up diver
    await motor.run_for_degrees(port.B, 1100, 300)

    # raise shark hook
    motor.run_for_degrees(port.C, -120, 650)

    # come back from Coral tree to get in alignment with Coral buds
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-500, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(7.5)))

    # turn right to get in alignment with coral buds
    await pivot_gyro_turn_abs(left_speed=100, right_speed=-100, angle=65, stop=True)

    # go forward to align with coral buds
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=400, target_angle=65, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(9)))

    # lower Shark Hook to push the coral buds mission
    await motor.run_for_degrees(port.C, 120, 600)

    # raise shark hook
    await motor.run_for_degrees(port.C, -100, 700)

    # come back to get in alignment with coral buds
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-400, target_angle=65, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(13)))

    # turn right to get in alignment with coral buds
    await pivot_gyro_turn_abs(left_speed=75, right_speed=-75, angle=97, stop=True)

    # come back to get in take fork out
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-400, target_angle=97, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(2)))

    # put fork down to drop off Scuba Diver
    await motor.run_for_degrees(port.B, 840, -400)

    # go forward towards scuba diver drop off
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=100, target_angle=97, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(8)))

    # put fork down to fully release Scuba Diver
    await motor.run_for_degrees(port.B, 250, -400)

    # come back from to get away from coral nursery
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-500, target_angle=97, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(10)))

    # come back to get to base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-950, target_angle=130, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(60)))
# END RUN 3
#----------------------------------------


# RUN 4
#----------------------------------------

async def run4():

    # go forward very slow to drop off coral pieces, get out of base, and get to research vessel
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=125, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(18)))

    # put ramp down to drop research vessel pieces
    await motor.run_for_degrees(port.C, 1200, 1500)

    # sleep to let pieces fall into boat
    await runloop.sleep_ms(500)

    # bring ramp up so it doesn't get in the way of coral pieces
    await motor.run_for_degrees(port.C, 500, -1000)

    # come back to base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-1000, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(20)))

# END RUN 4
#----------------------------------------


# RUN 5
#----------------------------------------
# run 5 program
async def run5():

    # go straight to get out of base and in position to drop shark
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=1000, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(72)))

    # move rack to the left to drop off shark
    await motor.run_for_degrees(port.B, 700, -1000)

    # come back to leave shark
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-1000, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(3.5)))

    # bring rack inside so it doesn't pull back shark
    await motor.run_for_degrees(port.B, 500, 1000)

    # come back to get in alignment with krill
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-500, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(20.5)))

    # bring arm down to engage with research vessel
    await motor.run_for_degrees(port.C, 2025, 1000)

    # move rack more to catch krill
    await motor.run_for_degrees(port.B, 750, -700)

    # go a little forward to catch the krill
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=200, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(12)))

    # bring the rack inside to catch krill
    await motor.run_for_degrees(port.B, 900, 1000)

    # go forward with boat to get in the docking area
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-5.5, ki=0, kd=0, speed=350, target_angle=-2, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(62)))

    # come back to ensure arm dosen't get stuck
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-300, target_angle=-2, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(6)))

    # raise arm so it doesn't get in the way
    await motor.run_for_degrees(port.C, -2000, 1500)

    # go forward to leave ship and get in alignment with unexpected encounter
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=1000, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(30)))

    # turn to align with unexpected encounter
    await pivot_gyro_turn_abs(left_speed=250, right_speed=-250, angle=45, stop=True)

    # go forward (back) to push unexpected encounter lever and catch creature
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-600, target_angle=45, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(34)))

    # go back (forward) to base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=1000, target_angle=45, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(37)))

# END RUN 6
#----------------------------------------


# RUN 7
#----------------------------------------
# run 7 program
async def run7():
    # move forward to get out of base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=1000, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degreesForDistance(37))
    motor.reset_relative_position(port.A, 0)
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=200, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degreesForDistance(5.5))

    # turn right to flick mission into new position
    await pivot_gyro_turn_abs(-400, 400, -15, True)
    await pivot_gyro_turn_abs(800, -800, 50, True)

    # turn left to get back in alignment with Artificil Habitat
    await pivot_gyro_turn_abs(-200, 200, -0, True)

    # move forward to get closer to the mission so mission is set up correctly
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=300, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degreesForDistance(17))

    # move robot back to complete alignment with Artificial Habitat
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-600, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degreesForDistance(16))

    # bring scooper down to get ready to slightly lift mission up
    motor.run_for_degrees(port.B, 100, 150)

    # move robot forward to get scooper under artificial habitat
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=300, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degreesForDistance(8))

    # bring scooper up to get ready to complete mission
    await motor.run_for_degrees(port.B, -150, 1000)

    # bring scooper down to push last crab facing up
    motor.run_for_degrees(port.B, 90, 200)

    # move robot forward to push crab facing up and to align with mission
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=300, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degreesForDistance(8))

    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degreesForDistance(11.5), 0, velocity=100)

    # move robot back to move away from Artificial Habitat
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-700, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degreesForDistance(16))

    # turn right to go towards Angler Fish
    await pivot_gyro_turn_abs(200, -200, 46, True)

    # move robot forward to keep going towards Angler Fish
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=700, target_angle=48, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degreesForDistance(54))

    # turn left to flick the handle of the Angler fish
    await pivot_gyro_turn_abs(-200, 200, 35 , True)

    # turn right to go towards Submersible
    await pivot_gyro_turn_abs(200, -200, 86, True)

    # bring scooper down to get ready to slightly lift misiion up
    motor.run_for_degrees(port.B, 20, 300)

    # go forward towards Submersible
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=300, target_angle=86, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degreesForDistance(10))

    # bring scoopr up to hold mission's yellow beam
    await motor.run_for_degrees(port.B, -10, 200, stop=motor.HOLD)

    # sleep to hold submersible yellow beam up
    await runloop.sleep_ms(2000)

    # turn right to align with dropping octupus
    await pivot_gyro_turn_abs(300, -300, 160, True)

# END RUN 7
#----------------------------------------

# END RUN FUNCTIONS
#----------------------------------------

#-------------------------------------------------------------------------------------------------------------------------------------------------------------

# MAIN FUNCTIONS
#----------------------------------------

async def execute(run_numbers=None):
    
    runs_to_execute = list()

    if isinstance(run_numbers, int):
        run_numbers = [run_numbers]

    # If run_numbers are not provided execute all runs
    runs_to_execute = run_numbers if run_numbers else [1, 3, 4, 5, 6, 7] # Skipping run 2

    start_times = [time.ticks_ms() for _ in runs_to_execute]
    end_times = [time.ticks_ms() for _ in runs_to_execute]

    run_functions_map = {
                            1: run1, # or run1a
                            2: run2,
                            3: run3,
                            4: run4,
                            5: run5,
                            6: run6,
                            7: run7
                        }
    print("Start - Execute")

    # Initialization
    # Define motor pai for robot movements
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)

    light_matrix.write("0")
    light.color(light.POWER, color.RED)

    # Reset Yaw
    await doInit()

    for i, run_number in enumerate(runs_to_execute):

        # waiting for left button to be pressed to start the run
        await runloop.until(is_left_button_pressed)
        print("Starting Run: " + str(run_number))

        light.color(light.POWER, color.MAGENTA)
        light_matrix.show_image(light_matrix.IMAGE_BUTTERFLY)

        start_times[i] = time.ticks_ms()
        await doInit()

        runloop.run(run_functions_map[run_number]())
        end_times[i] = time.ticks_ms()
        light.color(light.POWER, color.YELLOW)

        print("Completed Run: " + str(run_number))
        if i > 0:
            print("Time Taken for Transition after last run was *" + str(get_time_taken_in_seconds(end_times[i - 1], start_times[i])) + " seconds.*")
        print("Time Taken for Run " + str(run_number) + " is *" + str(get_time_taken_in_seconds(start_times[i], end_times[i])) + " seconds.*")
        print("---------------------------------------------------------------------------")

    # Print execution times
    print("---------------------------------------------------------------------------")
    print("SUMMARY:")
    for i, run_number in enumerate(runs_to_execute):
        if i > 0:
            print("Time Taken for Transition after last run was *" + str(get_time_taken_in_seconds(end_times[i - 1], start_times[i])) + "* seconds.")
        
        print("Time Taken for Run *" + str(run_number) + " is " + str(get_time_taken_in_seconds(start_times[i], end_times[i])) + "* seconds.")
        
# END MAIN FUNCTION
#----------------------------------------

# Integrated Runs

# SLOT 0 - Indiviual Run
runloop.run(execute([1]))

# SLOT 1 - All Runs
# runloop.run(execute([1, 3, 4, 5, 6, 7]))

# SLOT 2 - Run 3 Onwards
# runloop.run(execute([3, 4, 5, 6, 7]))

# SLOT 3 - Run 4 Onwards
# runloop.run(execute([4, 5, 6, 7]))

# SLOT 4 - Run 5 Onwards
# runloop.run(execute([5, 6, 7]))

# SLOT 5 - Run 5 Onwards
# runloop.run(execute([6, 7]))

# SLOT 5 - Run 5 Onwards
# runloop.run(execute([7]))
