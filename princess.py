#!/usr/bin/env python3
import color, color_sensor, device, motor, motor_pair, orientation, runloop
import hub
import sys

from hub import light_matrix, button, motion_sensor, light, sound, port
# START Common Functions--------------------------------------------------------------------------------------------
WHEEL_CIRCUMFERENCE = 17.584

WHITE_COLOR_INTENSITY_MIN = 97
BLACK_COLOR_INTENSITY_MAX = 18

COLOR_SENSOR_CENTER_PORT = port.C
COLOR_SENSOR_LEFT_PORT = port.D

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

# END Common Functions--------------------------------------------------------------------------------------------

async def test_follow_gyro_angle_for_distance(distance):
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    print("degreesForDistance = {}".format(str(degreesForDistance(distance))))
    await follow_gyro_angle(kp=-1.45*(int(distance/abs(distance))), ki=0, kd=0, speed=250*(int(distance/abs(distance))), target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(distance)))

async def test_turn_left(angle=90):
    await turn_left(speed=350, angle=angle, stop=True)

async def test_turn_right(angle=0):
    await turn_right(speed=350, angle=0, stop=True)

async def test_go_to_black_center(reverse=False):
    await follow_gyro_angle(kp=-1.45*(1 if reverse else -1), ki=0, kd=0,
                            speed=250*(-1 if reverse else 1), target_angle=0, sleep_time=0, follow_for=follow_for_color_black_center)

async def test_go_to_white_center(reverse=False):
    await follow_gyro_angle(kp=-1.45*(1 if reverse else -1), ki=0, kd=0,
                            speed=250*(-1 if reverse else 1), target_angle=0, sleep_time=0, follow_for=follow_for_color_white_center)

async def test_go_to_black_left(reverse=False):
    await follow_gyro_angle(kp=-1.45*(1 if reverse else -1), ki=0, kd=0,
                            speed=250*(-1 if reverse else 1), target_angle=0, sleep_time=0, follow_for=follow_for_color_black_left)

async def test_go_to_white_left(reverse=False):
    await follow_gyro_angle(kp=-1.45*(1 if reverse else -1), ki=0, kd=0,
                            speed=250*(-1 if reverse else 1), target_angle=0, sleep_time=0, follow_for=follow_for_color_white_left)

async def test_fake_missions():
    # Go forward 20 cm
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    distance = 20
    await follow_gyro_angle(kp=-1.45*(int(distance/abs(distance))), ki=0, kd=0, speed=250*(int(distance/abs(distance))), target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(distance)))

    # turn left 45 degrees
    await turn_left(speed=100, angle=45, stop=True)

    # go forward 12 cm
    distance = 12
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45*(int(distance/abs(distance))), ki=0, kd=0, speed=250*(int(distance/abs(distance))), target_angle=-45, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(distance)))

    # go back 12 cm
    distance = -12
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45*(int(distance/abs(distance))), ki=0, kd=0, speed=250*(int(distance/abs(distance))), target_angle=-45, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(distance)))


    # Turn right to 45
    await turn_right(speed=150, angle=45, stop=True)

    # Go forward 25 cm
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    distance = 25
    await follow_gyro_angle(kp=-1.45*(int(distance/abs(distance))), ki=0, kd=0, speed=250*(int(distance/abs(distance))), target_angle=45, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(distance)))

    # Go back 25 cm
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    distance = -25
    await follow_gyro_angle(kp=-1.45*(int(distance/abs(distance))), ki=0, kd=0, speed=250*(int(distance/abs(distance))), target_angle=45, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(distance)))


    # Turn right to 180 (facing to the pit)
    await turn_right(speed=150, angle=179, stop=True)

    # Go to pit
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    distance = 20
    await follow_gyro_angle(kp=-1.45*(int(distance/abs(distance))), ki=0, kd=0, speed=250*(int(distance/abs(distance))), target_angle=179, sleep_time=0, follow_for=follow_for_distance,
                            initial_position=initial_position, distance_to_cover=(degreesForDistance(distance)))

# START RUN Functions--------------------------------------------------------------------------------------------

# run 1 program 
async def run1():
    # turn 47 degrees to get in alignment wiht unexpected encounter
    await turn_right(speed=100, angle=47, stop=True)

    # go forward to push unexpected encounter lever and collect octopus
    distance = -55
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45*(int(distance/abs(distance))), ki=0, kd=0, speed=300*(int(distance/abs(distance))), target_angle=47, sleep_time=0, follow_for=follow_for_distance,
                initial_position=initial_position, distance_to_cover=(degreesForDistance(distance)))

    # lower fork attachment to get in line with shipping lane levers
    motor.run_for_degrees(port.C, 2400, 1000)
    
    # go backward from unexpected encounter to get in position with Changing Shipping Lane
    distance = 10
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45*(int(distance/abs(distance))), ki=0, kd=0, speed=300*(int(distance/abs(distance))), target_angle=47, sleep_time=0, follow_for=follow_for_distance,
                initial_position=initial_position, distance_to_cover=(degreesForDistance(distance)))

    # turn left to get in alignment with changing shipping lanes
    await turn_left(speed=100, angle=-50, stop=True)

    # go forward to engage with changing shipping lanes
    distance = 7
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=300, target_angle=-50, sleep_time=0, follow_for=follow_for_distance,
                initial_position=initial_position, distance_to_cover=(degreesForDistance(distance)))

    # raise the ship and get to the amout where we can turn
    await motor.run_for_degrees(port.C, 550, -800)

    # turn right to get in position to drop changing shipping lanes
    await turn_right(speed=100, angle=-20, stop=True)

    # drop ship by going backward
    distance = -7
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45*(int(distance/abs(distance))), ki=0, kd=0, speed=300*(int(distance/abs(distance))), target_angle=-20, sleep_time=0, follow_for=follow_for_distance,
                initial_position=initial_position, distance_to_cover=(degreesForDistance(distance)))

# run 2 program
async def Run2():

    # go forward to take attatchment out of base 
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    distance = 20
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=400, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(20)))

    # go backward to get the attatchment flat
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-400, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(23)))
    
    # move forward to approach and move mission up
    # motor.reset_relative_position(port.A, 0)
    # initial_position = abs(motor.relative_position(port.A))
    # await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=100, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
    #                 initial_position=initial_position, distance_to_cover=(degreesForDistance(50)))
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degreesForDistance(51), 0, velocity=175)

    # go backward to get back to base 
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-600, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(42)))

    # turn left to get fully in base
    await turn_left(speed=100, angle=24, stop=True)

# END RUN Functions--------------------------------------------------------------------------------------------

async def mainProgram():
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    print("mainProgram -- START")

    light_matrix.write("0")
    light.color(light.POWER, color.RED)

    # reset yaw to 0
    motion_sensor.set_yaw_face(motion_sensor.TOP)
    motion_sensor.reset_yaw(0)
    await runloop.sleep_ms(1000)

    print("calling run1")
    print("calling run2")
    await run1()
    await run2()
    

    # i = 0
    # while (hub.motion_sensor.stable() == False):
    #    i = i+1
    #    await runloop.sleep_ms(100)
    #    hub.light_matrix.write(str(i))

runloop.run(mainProgram())
