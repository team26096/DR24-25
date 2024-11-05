#!/usr/bin/env python3
import color, color_sensor, device, motor, motor_pair, orientation, runloop
import hub
import sys, time

from hub import light_matrix, button, motion_sensor, light, sound, port
# START Common Functions--------------------------------------------------------------------------------------------
WHEEL_CIRCUMFERENCE = 17.584

WHITE_COLOR_INTENSITY_MIN = 97
BLACK_COLOR_INTENSITY_MAX = 18

COLOR_SENSOR_CENTER_PORT = port.F
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

def get_center_color_values():
    return color_sensor.reflection(COLOR_SENSOR_CENTER_PORT)

def get_left_color_values():
    return color_sensor.reflection(COLOR_SENSOR_LEFT_PORT)

def follow_for_color_white_center():
    return get_center_color_values() <= WHITE_COLOR_INTENSITY_MIN

def follow_for_color_black_center():
    return get_center_color_values() >= BLACK_COLOR_INTENSITY_MAX

def follow_for_color_white_left():
    return get_left_color_values() <= WHITE_COLOR_INTENSITY_MIN

def follow_for_color_black_left():
    return get_left_color_values() >= BLACK_COLOR_INTENSITY_MAX

def get_yaw_value():
    return motion_sensor.tilt_angles()[0] * -0.1

def degreesForDistance(distance_cm):
    # Add multiplier for gear ratio if needed
    return int((distance_cm/WHEEL_CIRCUMFERENCE) * 360)

def wait_for_yaw_abs(angle=0.0):
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

def pivot_gyro_turn_abs_sync(left_speed=0, right_speed=50, angle=90.0, stop=False):
    motor_pair.move_tank(motor_pair.PAIR_1, left_speed, right_speed)
    # print("pivot_gyro_turn - " + "target angle=" + str(angle) + "current angle ="+ str(get_yaw_value()))
    wait_for_yaw_abs(angle=angle)
    if stop: motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)


async def pivot_gyro_turn_abs(left_speed=0, right_speed=50, angle=90.0, stop=False, accleration=1000):
    motor_pair.move_tank(motor_pair.PAIR_1, left_speed, right_speed, acceleration=accleration)
    # print("pivot_gyro_turn - " + "target angle=" + str(angle) + "current angle ="+ str(get_yaw_value()))
    wait_for_yaw_abs(angle=angle)
    if stop: motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)

async def turn_left(speed=50, angle=90, stop=True):
    await pivot_gyro_turn_abs(left_speed=0, right_speed=speed, angle=angle, stop=stop)

async def turn_right(speed=-50, angle=90, stop=True):
    await pivot_gyro_turn_abs(left_speed=speed, right_speed=0, angle=angle, stop=stop)

# END Common Functions--------------------------------------------------------------------------------------------

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
    #                initial_position=initial_position, distance_to_cover=(degreesForDistance(50)))
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degreesForDistance(51), 0, velocity=175)

    # go backward to get back to base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    distance = -42
    await follow_gyro_angle(kp=-1.25*(int(distance/abs(distance))), ki=0.002, kd=-0.001, speed=400*(int(distance/abs(distance))), target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(distance)))

    runloop.sleep_ms(500)
    await turn_left(speed=100, angle=-25, stop=True)
    runloop.sleep_ms(500)

    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    distance = 5
    await follow_gyro_angle(kp=-1.25*(int(distance/abs(distance))), ki=0.002, kd=-0.001, speed=400*(int(distance/abs(distance))), target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(distance)))



    # turn left to get fully in base
    # await turn_left(speed=100, angle=24, stop=True)

 # run 6 program
async def run6():

    # # go forward to to get out of base and go towards feed the whale
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=1000, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(75)))

    # turn right to align with feed the whale
    await pivot_gyro_turn_abs(150, -150, 45, True)

    # move forward to open whale's mouth
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degreesForDistance(12), 0, velocity=190)

    # turn motor to move food tray down
    await motor.run_for_degrees(port.B, 1080, 1000)

    # move motor to lift food tray so it does not make whale vomit while coming back
    await motor.run_for_degrees(port.B, 720, -1000)

    # move robot back to move away from feed the whale
    motor.reset_relative_position(port.A, 0)
    motion_sensor.set_yaw_face(motion_sensor.TOP)
    motion_sensor.reset_yaw(0)
    await runloop.sleep_ms(500)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=-400, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degreesForDistance(22))

    # turn robot to align for Sonar Discovery
    await pivot_gyro_turn_abs(105, -105, 138, stop=True)

    # move robot back to complete alignment with Sonar Discovery
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-500, target_angle=138, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degreesForDistance(38))

    # turn Sonar Discovery attachment motor to complete Sonar Discovery
    await motor.run_for_degrees(port.C, 720, -500)

    # turn attachment other way so it does not get stuck
    await motor.run_for_degrees(port.C, -300, -500)

    # # move forward to come back to base
    # await motor_pair.move_for_degrees(motor_pair.PAIR_1, degreesForDistance(35), 0, velocity=1000)

    # move forward to come to base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=850, target_angle=130, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degreesForDistance(35))

    # go forward to end in base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=1000, target_angle=105, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degreesForDistance(45))

 







    # # turn robot to go into base without being out
    # await pivot_gyro_turn_abs(-700, 700, 110, True)

    # # move forward to go and end in base
    # await motor_pair.move_for_degrees(motor_pair.PAIR_1, degreesForDistance(50), 0, velocity=1000)





    # # Start going to Sonar Discovery
    # motor.reset_relative_position(port.A, 0)

    # await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=-300, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
    #                initial_position=initial_position, distance_to_cover=(degreesForDistance(11.5)))

    # # Turn robot to align with sonar discovery
    # motion_sensor.set_yaw_face(motion_sensor.TOP)
    # motion_sensor.reset_yaw(0)
    # await pivot_gyro_turn_abs(0, -150, 50, True)

    # # Go reverse past Sonar Discovery
    # motion_sensor.set_yaw_face(motion_sensor.TOP)
    # motion_sensor.reset_yaw(0)
    # motor.reset_relative_position(port.A, 0)
    # initial_position = abs(motor.relative_position(port.A))
    # distance = -65
    # await follow_gyro_angle(kp=-1.25*(int(distance/abs(distance))), ki=0.002, kd=-0.001, speed=350*(int(distance/abs(distance))), target_angle=0, sleep_time=0, follow_for=follow_for_distance,
    #                initial_position=initial_position, distance_to_cover=(degreesForDistance(distance)))

    # Turn left slightly
    # await turn_left(angle=-15)

    # # Go forward towards Sonar Discovery mission
    # motion_sensor.set_yaw_face(motion_sensor.TOP)
    # motion_sensor.reset_yaw(0)
    # motor.reset_relative_position(port.A, 0)
    # initial_position = abs(motor.relative_position(port.A))
    # distance = 13
    # await follow_gyro_angle(kp=-1.25*(int(distance/abs(distance))), ki=0.002, kd=-0.001, speed=350*(int(distance/abs(distance))), target_angle=0, sleep_time=0, follow_for=follow_for_distance,
    #                initial_position=initial_position, distance_to_cover=(degreesForDistance(distance)))

    # Turn right slightly to align with Sonar Discovery mission
    # await turn_right(speed=100, angle=13)

    # motion_sensor.set_yaw_face(motion_sensor.TOP)
    # motion_sensor.reset_yaw(0)
    # motor.reset_relative_position(port.A, 0)
    # initial_position = abs(motor.relative_position(port.A))
    # distance = 17
    # await follow_gyro_angle(kp=-1.25*(int(distance/abs(distance))), ki=0.002, kd=-0.001, speed=350*(int(distance/abs(distance))), target_angle=0, sleep_time=0, follow_for=follow_for_distance,
    #                initial_position=initial_position, distance_to_cover=(degreesForDistance(distance)))

    # Move robot back to get ready to complete sonar discorvery
    # motor.reset_relative_position(port.A, 0)
    # initial_position = abs(motor.relative_position(port.A))

    # await pivot_gyro_turn_abs(0, -150, 0, True)
    # await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=-175, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
    #                initial_position=initial_position, distance_to_cover=(degreesForDistance(25)))

    # # reset yaw to 0
    # motion_sensor.set_yaw_face(motion_sensor.TOP)
    # motion_sensor.reset_yaw(0)
    # await runloop.sleep_ms(1000)


 # run 7 program
async def run7():
    # move forward to get out of base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=400, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degreesForDistance(37))
    motor.reset_relative_position(port.A, 0)
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=200, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degreesForDistance(5.5))

    # turn right to flick mission into new position
    await pivot_gyro_turn_abs(-400, 400, -15, True, accleration=5000)
    await pivot_gyro_turn_abs(1000, -1000, 50, True, accleration=5000)
    
    # turn left to get back in alignment with Artificil Habitat
    await pivot_gyro_turn_abs(-200, 200, -0, True)

    # move forward to get closer to the mission so mission is set up correctly
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=300, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degreesForDistance(20))

    # move robot back to complete alignment with Artificial Habitat
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-600, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degreesForDistance(16))

    # bring scooper down to get ready to slightly lift misiion up
    await motor.run_for_degrees(port.B, -100, 800)

    # move robot forward to complete alignment with Artificial Habitat
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=200, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degreesForDistance(10))

    # bring scooper up to get ready to complete mission
    await motor.run_for_degrees(port.B, 150, 1000)

    # move robot forward to align with Artificial Habitat
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=300, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degreesForDistance(8))

    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degreesForDistance(11.5), 0, velocity=100, )    

    # move robot back to move away from Artificial Habitat
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-200, target_angle=2, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degreesForDistance(14))

    # turn right to go towards Angler Fish
    await pivot_gyro_turn_abs(200, -200, 48, True)    

    # move robot forward to keep going towards Angler Fish
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=300, target_angle=48, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degreesForDistance(54))
    
    # turn left to flick the handle of the Angler fish
    await pivot_gyro_turn_abs(-200, 200, 35 , True)

    # turn right to go towards Submersible
    await pivot_gyro_turn_abs(200, -200, 86, True)

    # bring scooper down to get ready to slightly lift misiion up
    motor.run_for_degrees(port.B, -110, 800)

    # go forward towards Submersible
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=200, target_angle=86, sleep_time=0, follow_for=follow_for_color_white_center)
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=200, target_angle=86, sleep_time=0, follow_for=follow_for_color_black_center)

    # turn right to align with Submersible
    # await pivot_gyro_turn_abs(200, -200, 90, True)

    # # go forward towards Submersible
    # motor.reset_relative_position(port.A, 0)
    # initial_position = abs(motor.relative_position(port.A))
    # await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=300, target_angle=86, sleep_time=0, follow_for=follow_for_distance,
    #     initial_position=initial_position, distance_to_cover=degreesForDistance(3))

    # bring scoopr up to hold mission's yellow beam
    await motor.run_for_degrees(port.B, 115, 850)

    # sleep to hold submersible yellow beam up
    await runloop.sleep_ms(3000)   
    
    # turn left to align with dropping octupus
    await pivot_gyro_turn_abs(-200, 200, 45, True)

    # move robot back to end in postion of dropping off octupus
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-600, target_angle=10, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degreesForDistance(10))


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

    # print("calling run1")
    # print("calling run2")
    # await run1()
    # await run2()
    ct=time.ticks_ms()
    await run7()
    et=time.ticks_ms()
    print("Total run time =" + str(time.ticks_diff(et, ct)/1000))

    # i = 0
    # while (hub.motion_sensor.stable() == False):
    #    i = i+1
    #    await runloop.sleep_ms(100)
    #    hub.light_matrix.write(str(i))

runloop.run(mainProgram())
