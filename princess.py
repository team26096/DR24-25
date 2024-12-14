#!/usr/bin/env python3

import hub
import sys
import time

import color, motor, motor_pair, runloop
from hub import light_matrix, button, motion_sensor, light, port


# CONSTANTS
#----------------------------------------

WHEEL_CIRCUMFERENCE = 17.584

# END CONSTANTS
#----------------------------------------


# UTILITY FUNCTIONS
#----------------------------------------

# initialize motor and reset yaw
def do_init():
    # reset yaw to 0
    motion_sensor.set_yaw_face(motion_sensor.TOP)
    motion_sensor.reset_yaw(0)
    i = 0
    while (hub.motion_sensor.stable() == False):
        i = i + 1
        # Use time.sleep_ms instead of time.sleep_ms
        # to ensure it is synchronized
        time.sleep_ms(10)
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


def get_yaw_value():
    return motion_sensor.tilt_angles()[0] * -0.1


def degrees_for_distance(distance_cm):
    # Add multiplier for gear ratio if needed
    return int((distance_cm/WHEEL_CIRCUMFERENCE) * 360)


def wait_for_yaw_abs(angle=0):
    abs_angle = abs(angle)
    abs_current_yaw = abs(get_yaw_value())
    if angle == 0:
        if get_yaw_value() > 0:
            while get_yaw_value() >= angle: time.sleep_ms(10)
        elif get_yaw_value() < 0:
            while get_yaw_value() <= angle: time.sleep_ms(10)
    elif abs_current_yaw > abs_angle:
        while abs(get_yaw_value()) >= abs_angle: time.sleep_ms(10)
    elif abs_current_yaw < abs_angle:
        while abs(get_yaw_value()) <= abs_angle: time.sleep_ms(10)


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
            time.sleep_ms(sleep_time)
        # kp value should be +ve for forward movement (positive speed value), and -ve for backward movement (negative speed value)
        motor_pair.move(motor_pair.PAIR_1, int(steering_value), velocity=speed)

    # stop when follow_for condition is met
    motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)


async def pivot_gyro_turn_abs(left_speed=0, right_speed=50, angle=90, stop=False):
    motor_pair.move_tank(motor_pair.PAIR_1, left_speed, right_speed)
    wait_for_yaw_abs(angle=angle)
    if stop: motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)


async def turn_left(speed=50, angle=90, stop=True):
    await pivot_gyro_turn_abs(left_speed=0, right_speed=speed, angle=angle, stop=stop)


async def turn_right(speed=-50, angle=90, stop=True):
    await pivot_gyro_turn_abs(left_speed=speed, right_speed=0, angle=angle, stop=stop)


def get_time_taken_in_seconds(start_time, end_time):
    return int(time.ticks_diff(end_time, start_time)/1000)

# END UTILITY FUNCTIONS
#----------------------------------------

# RUN FUNCTIONS
#----------------------------------------

# RUN 1
#----------------------------------------
# run 1 program
async def run1():

    # go backward to get out of base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-500, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degrees_for_distance(3)))

    # turn left to get in alignment with krill
    await pivot_gyro_turn_abs(left_speed=-200, right_speed=200, angle=-45, stop=True)

    # go backward to collect krill
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-600, target_angle=-45, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degrees_for_distance(21)))

    # turn right to get in alignment with coral piece
    await pivot_gyro_turn_abs(left_speed=200, right_speed=-200, angle=0, stop=True)

    # go backward to collect coral piece and krill
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-600, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degrees_for_distance(42)))

    # turn right to collect krill
    await pivot_gyro_turn_abs(left_speed=200, right_speed=-200, angle=45, stop=True)

    # go forward to collect krill
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-400, target_angle=45, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degrees_for_distance(9)))

    # turn left to align with plankton
    await pivot_gyro_turn_abs(left_speed=200, right_speed=-200, angle=165, stop=True)
    await pivot_gyro_turn_abs(left_speed=150, right_speed=-150, angle=-93, stop=True)

    # go forward to hook into plankton
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(10),0,velocity=200)

    # go backward to pull plankton
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(2),0,velocity=-300)

    # go forward to get away from sonar discovery
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-500, target_angle=-93, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degrees_for_distance(5)))

    # turn right to go forward
    await pivot_gyro_turn_abs(left_speed=150, right_speed=-150, angle=-88, stop=True)

    # go backward toward seabed
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-800, target_angle=-88, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degrees_for_distance(100)))

    # bring send over the submersible attachment down
    motor.run_for_degrees(port.B, 2500, 900)

    # turn to place pieces
    await pivot_gyro_turn_abs(left_speed=-200, right_speed=200, angle=-90, stop=True)

    # go forward (back) to leave pieces
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=500, target_angle=-90, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degrees_for_distance(20)))

    # turn to align to seabed sample
    await pivot_gyro_turn_abs(left_speed=200, right_speed=-200, angle=0, stop=True)

    # go forward to engage with seabed sample
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(17),0,velocity=-400)

    # raise seabed sample hook to raise the sample and collect it
    # raise send over the submersible attachment
    motor.run_for_degrees(port.C, 1200, 900)
    await motor.run_for_degrees(port.B, 2000, -1000)
    motor.run_for_degrees(port.C, 800, 900)

    # come back (go forward) to leave seabed
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=400, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degrees_for_distance(11)))

    # bring down send over the submersible attachment for coral reef buds mission
    motor.run_for_degrees(port.B, 1300, 1000)

    # turn to leave seabed sample
    await pivot_gyro_turn_abs(left_speed=-200, right_speed=200, angle=-95, stop=True)

    # go forward to recollect samples
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-700, target_angle=-95, sleep_time=0, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(17)))

    # turn left to align with water sample/krill
    await pivot_gyro_turn_abs(left_speed=-200, right_speed=200, angle=-102, stop=True)

    # go forward to collect water sample and krill
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-500, target_angle=-102, sleep_time=0, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(14)))

    # bring down send over the submersible attachment for coral reef buds mission
    await motor.run_for_degrees(port.B, 1500, 1000)

    # go forward to collect water sample and krill
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-500, target_angle=-102, sleep_time=0, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(14)))

    # bring up send over the submersible attachment
    motor.run_for_degrees(port.B, 1500, -1000)

    # turn left collect last coral piece
    await pivot_gyro_turn_abs(left_speed=-200, right_speed=200, angle=-155, stop=True)

    # go forward to collect last coral piece
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-1000, target_angle=-155, sleep_time=0, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(40)))

    # go forward to get into base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-1000, target_angle=-125, sleep_time=0, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(30)))

# END RUN 1
#----------------------------------------

# RUN 2
#----------------------------------------
# run 2 program - Raise the mast, Kraken's treasure, Diver Pickup, Coral buds
async def run2():

        # go straight to get out of base (backward)
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-600, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(18)))

    # turn right to get away from coral tree
    await pivot_gyro_turn_abs(left_speed=100, right_speed=-100, angle=30, stop=True)

    # go straight (backward) to align with shipwreck
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-600, target_angle=30, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(27)))

    # turn right to get in front of shipwreck
    await pivot_gyro_turn_abs(left_speed=100, right_speed=-100, angle=90, stop=True)

    # go straight to engage with shipwreck
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-150, target_angle=90, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(27)))

    # come back to collect treasure and release mast
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=300, target_angle=90, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(23)))
    
    # lower fork arm to get in position to pick up diver
    motor.run_for_degrees(port.B, -2100, 1000)

    # turn left to prepare for alignment with coral tree
    await pivot_gyro_turn_abs(left_speed=-100, right_speed=100, angle=30, stop=True)

    # go forward to prepare for alignment with coral tree
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-600, target_angle=30, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(13)))

    # turn right to get in alignment with coral tree
    await pivot_gyro_turn_abs(left_speed=150, right_speed=-150, angle=90, stop=True)

    # Go back slightly to position fork to align with scuba driver
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-300, target_angle=90, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(5)))

    # go straight to push coral tree buds
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(13), 0, velocity=300)

    # reset yaw to 0
    do_init()

    # lower Shark Hook to push the shark misson lever
    motor.run_for_degrees(port.C, -100, 600)

    # raise fork arm to pick up diver
    await motor.run_for_degrees(port.B, 800, 1000)
    motor.run_for_degrees(port.B, 300, 1000)

    # come back from Coral tree to get in alignment with Scuba Diver
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-500, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(7)))

    # raise Shark Hook AFTER COMING BACK
    motor.run_for_degrees(port.C, 75, 250)

    # turn right to get in alignment with scuba diver
    await pivot_gyro_turn_abs(left_speed=75, right_speed=-75, angle=98, stop=True)

    # come back to get in alignment with scuba diver
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-400, target_angle=98, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(3)))

    # put fork down to drop off Scuba Diver
    await motor.run_for_degrees(port.B, -790, 500)

    # go forward towards scuba diver drop off
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(9.5), 0, velocity=100)

    # move fork down to fully release Scuba Diver
    await motor.run_for_degrees(port.B, -250, 500)

    # come back a bit to get to base
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(8), 0, velocity=-100)

    # raise fork arm to ensurre it isn't out of base
    motor.run_for_degrees(port.B, 2000, 1000)

    # come back to get to base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-1100, target_angle=130, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(60.5)))

# END RUN 2
#----------------------------------------
# RUN 3
#----------------------------------------
# run 3 program
async def run3():

    # go straight to get out of base and in position to drop shark
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=1000, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(72)))

    # move rack to the left to drop off shark
    await motor.run_for_degrees(port.B, 700, -1000)

    # come back to leave shark
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-1000, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(3.5)))

    # bring rack inside so it doesn't pull back shark
    await motor.run_for_degrees(port.B, 500, 1000)

    # come back to get in alignment with krill
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-500, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(20.5)))

    # move rack more to catch krill
    motor.run_for_degrees(port.B, 750, -700)

    # bring arm down to engage with research vessel
    await motor.run_for_degrees(port.C, 2025, 1000)

    # go a little forward to catch the krill
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=200, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(12)))

    # bring the rack inside to catch krill
    await motor.run_for_degrees(port.B, 900, 1000)

    # go forward with boat to get in the docking area
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-5.5, ki=0, kd=0, speed=350, target_angle=-2, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(62)))

    # come back to ensure arm dosen't get stuck
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-300, target_angle=-2, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(7)))

    # raise arm so it doesn't get in the way
    await motor.run_for_degrees(port.C, -1000, 1000)
    motor.run_for_degrees(port.C, -1000, 1000)

    # go forward to leave ship and get in alignment with unexpected encounter
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=1000, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(31)))

    # turn to align with unexpected encounter
    await pivot_gyro_turn_abs(left_speed=250, right_speed=-250, angle=45, stop=True)

    # go forward (back) to push unexpected encounter lever and catch creature
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-800, target_angle=45, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(34)))

    # go back (forward) to base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=1000, target_angle=45, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(37)))

# END RUN 3
#----------------------------------------

# RUN 4
#----------------------------------------
# run 4 program
async def run4():

    # # go backward to to get out of base and go towards feed the whale
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-500, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(2)))
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-500, target_angle=-25, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(15)))
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-500, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(10)))
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-1000, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(38)))
    motor.reset_relative_position(port.A, 0)
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-500, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(10)))

    # turn left to align with feed the whale
    await pivot_gyro_turn_abs(150, -150, 39, True)

    # move backward to open whale's mouth
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(18), 0, velocity=-300)

    # turn motor to move food tray down
    await motor.run_for_degrees(port.B, 1150, -1000)

    # reset yaw to 0 and ensure the krill are in the whale
    motor.reset_relative_position(port.A, 0)
    do_init()

    # move motor to lift food tray so it does not make whale vomit while coming back
    await motor.run_for_degrees(port.B, 350, 1000)
    motor.run_for_degrees(port.B, 700, 1000)

    # move robot forward to move away from feed the whale
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=400, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degrees_for_distance(15))

    # turn robot to align for Sonar Discovery
    await pivot_gyro_turn_abs(105, -105, 45, stop=True)

    # move backward to complete alignment with Sonar Discovery
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=400, target_angle=45, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degrees_for_distance(16))

    # turn Sonar Discovery attachment motor to complete Sonar Discovery
    await motor.run_for_degrees(port.C, 580, -400)

    # turn attachment other way so it does not get stuck
    await motor.run_for_degrees(port.C, -260, -450)

    # move backward to come move away from Sonar Discovery
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-1000, target_angle=45, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degrees_for_distance(5))

    # turn robot right to align for coming to base
    await pivot_gyro_turn_abs(300, -300, 117, stop=True)

    # move backward to come back to base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-1000, target_angle=117, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degrees_for_distance(70))

    # turn left to align with feed the whale
    await pivot_gyro_turn_abs(-150, 150, 80, True)

# END RUN 4
#----------------------------------------


# RUN 5
#----------------------------------------
# run 5 program
async def run5():
    # move forward to get out of base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=1000, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degrees_for_distance(36))
    motor.reset_relative_position(port.A, 0)
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=200, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degrees_for_distance(5.5))

    # turn right to flick mission into new position
    await pivot_gyro_turn_abs(-400, 400, -15, True)
    await pivot_gyro_turn_abs(800, -800, 50, True)

    # turn left to get back in alignment with Artifical Habitat
    await pivot_gyro_turn_abs(-200, 200, -0, True)

    # move forward to get closer to the mission so mission is set up correctly
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=300, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degrees_for_distance(17))

    # move robot back to complete alignment with Artificial Habitat
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-600, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degrees_for_distance(16))

    # bring scooper down to get ready to slightly lift mission up
    motor.run_for_degrees(port.B, 100, 150)

    # move robot forward to get scooper under artificial habitat
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=300, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degrees_for_distance(8))

    # bring scooper up to get ready to complete mission
    await motor.run_for_degrees(port.B, -150, 1000)

    # bring scooper down to push last crab facing up
    motor.run_for_degrees(port.B, 90, 200)

    # move robot forward to push crab facing up and to align with mission
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=300, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degrees_for_distance(8))

    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(11.5), 0, velocity=100)

    # move robot back to move away from Artificial Habitat
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-700, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degrees_for_distance(10))

    # turn right to go towards Angler Fish
    await pivot_gyro_turn_abs(200, -200, 60, True)

    # move robot forward to keep going towards Angler Fish
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=800, target_angle=60, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degrees_for_distance(46))


# END RUN 5
#----------------------------------------

# END RUN FUNCTIONS
#----------------------------------------

#-------------------------------------------------------------------------------------------------------------------------------------------------------------

# MAIN EXECUTE FUNCTION
#----------------------------------------

async def execute(run_numbers=None):

    runs_to_execute = list()

    if isinstance(run_numbers, int):
        run_numbers = [run_numbers]

    # If run_numbers are not provided execute all runs
    runs_to_execute = run_numbers if run_numbers else [1, 2, 3, 4, 5]

    start_times = [time.ticks_ms() for _ in runs_to_execute]
    end_times = [time.ticks_ms() for _ in runs_to_execute]

    run_functions_map = {
                            1: run1, # or run1a
                            2: run2,
                            3: run3,
                            4: run4,
                            5: run5
                        }
    print("Start - Execute")

    # Initialization
    # Define motor pai for robot movements
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)

    do_init()
    light_matrix.write("0")
    light.color(light.POWER, color.RED)

    for i, run_number in enumerate(runs_to_execute):

        # waiting for left button to be pressed to start the run
        await runloop.until(is_left_button_pressed)
        print("Starting Run: " + str(run_number))

        light.color(light.POWER, color.MAGENTA)
        light_matrix.show_image(light_matrix.IMAGE_BUTTERFLY)

        start_times[i] = time.ticks_ms()
        do_init()

        runloop.run(run_functions_map[run_number]())
        end_times[i] = time.ticks_ms()
        light.color(light.POWER, color.YELLOW)

        if i > 0:
            print("Transition time: " + str(get_time_taken_in_seconds(end_times[i - 1], start_times[i])) + " s")
        print("Run " + str(run_number) + " time " + str(get_time_taken_in_seconds(start_times[i], end_times[i])) + " s")
        print("---------------------------------------------------------------------------")

    # Print execution times
    print("---------------------------------------------------------------------------")
    print("SUMMARY:")
    total_runs_time = 0
    total_transitions_time = 0
    total_time = 0

    for i, run_number in enumerate(runs_to_execute):
        if i > 0:
            transition_time = get_time_taken_in_seconds(end_times[i - 1], start_times[i])
            print("Transition time: " + str(transition_time) + " s")
            total_transitions_time += transition_time
            total_time += transition_time

        run_time = get_time_taken_in_seconds(start_times[i], end_times[i])
        print("Run " + str(run_number) + " time " + str(run_time) + " s")
        total_runs_time += run_time
        total_time += run_time

    print("***************************************************************************")

    print("TOTAL RUN TIME = " + str(total_runs_time) + " s")
    print("TOTAL TRANSITIONS TIME = " + str(total_transitions_time) + " s")
    print("TOTAL TIME = " + str(total_transitions_time) + " s")

    print("***************************************************************************")


# END MAIN EXECUTE FUNCTION
#----------------------------------------

# Integrated Runs

# SLOT 0 - All Runs#
runloop.run(execute([1, 2, 3, 4, 5]))

# SLOT 1 - Run 2 Onwards
#runloop.run(execute([2, 3, 4, 5]))

# SLOT 2 - Run 3 Onwards
#runloop.run(execute([3, 4, 5]))

# SLOT 3 - Run 4 Onwards
#runloop.run(execute([4, 5]))

# SLOT 4 - Run 5
#runloop.run(execute([5]))
