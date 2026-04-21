#!/usr/bin/env pybricks-micropython

####################################################################################
# Bloc d'importation des bibliothèques nécessaires
####################################################################################

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

####################################################################################
# Classe de contrôle du robot
####################################################################################

class ControlRobot:
    def __init__(self, left_motor_port=Port.B, right_motor_port=Port.C):
        self.left_motor = Motor(left_motor_port)
        self.right_motor = Motor(right_motor_port)
        self.robot = DriveBase(self.left_motor, self.right_motor, wheel_diameter=55.5, axle_track=104)

    def rotate_with_speed(self, speed=100, turn_rate=0):
        self.robot.drive(speed, turn_rate)

    def move_forward(self, speed=100):
        self.robot.drive(speed, 0)

    def move_backward(self, speed=100):
        self.robot.drive(-speed, 0)

    def init_distance_mesurement(self):
        self.robot.reset()

    def get_distance_mesure(self):
        return self.robot.distance()
    
    def emergency_stop(self):
        self.robot.stop()


class SensRobot:
    def __init__(self, ultrasonic_sensor_port=Port.S4, color_sensor_port=Port.S3):
        self.color_sensor = ColorSensor(color_sensor_port)
        self.ultrasonic_sensor = UltrasonicSensor(ultrasonic_sensor_port)

    def read_sensors(self):
        color = self.color_sensor.color()
        distance = self.ultrasonic_sensor.distance()
        reflection = self.color_sensor.reflection()
        return {"color": color, "distance": distance, "reflection": reflection}

class PrintInfo:

    def __init__(self):
        pass

    def print_sensor_info(self,sensor_data):
        print(sensor_data)

    def print_on_lcd(self, sensor_data):
        ev3 = EV3Brick()
        ev3.screen.clear()
        ev3.screen.draw_text(sensor_data)

# class RobotNetwork:
#     def __init__(self, ssid, password):
#         self.wlan = network.WLAN(network.STA_IF)
#         self.wlan.active(True)
#         self.wlan.connect(ssid, password)
#         while not self.wlan.isconnected():
#             pass
#         print('network config:', self.wlan.ifconfig())

class RobotLoging:
    def __init__(self, *columns, commit_name=None):
        self.log = DataLog(*columns, name=commit_name)

    def log_data(self, *data):
        self.log.log(*data)

    def save_log(self, filename):
        self.log.save(filename)


########################################################################################
# commit_name = "line_follower_version_bang_bang_26_09_2025_speed100correction15line2m"
# #Gestion du logging
# log = RobotLoging("Time in ms", "Reflection", "Error", "Distance in mm", "Command", commit_name=commit_name)

########################################################################################
#Implementation du suivis de ligne
#########################################################################################


# Initialiser le chronomètre
stopwatch = StopWatch()

# Démarrer le chronomètre
stopwatch.reset()
stopwatch.resume()

execution_time = 100000  # 1000 ms = 1 seconde
distance_to_travel = 3000  # Distance in mm


##############################################################################

# Initialize the motors.
left_motor = (Port.B)
right_motor = (Port.C)

color_sensor = Port.S3
ultrasonic_sensor = Port.S2

control_robot = ControlRobot(right_motor_port=right_motor, left_motor_port=left_motor)
sens_robot = SensRobot(ultrasonic_sensor_port=ultrasonic_sensor, color_sensor_port=color_sensor)
print_info = PrintInfo()

##############################################################################

# Set statics values

white_line_reflection =  60  # Adjust this value based on your environment
black_line_reflection = 10   # Adjust this value based on your environment
middle_reflection = 30 #(white_line_reflection + black_line_reflection) / 2
#base_speed = 100
actual_distance = 0

# ##############################################################################
# # Bang-bang line following algorithm
# ##############################################################################

# bang_bang_threshold = 15

# control_robot.init_distance_mesurement()

# while actual_distance < distance_to_travel:  # Loop forever
#     # Optionnel : afficher le temps écoulé
# #    print(f"Temps écoulé : {stopwatch.time()} ms")

# #    control_robot.move_forward(speed=base_speed)

#     sensor_data = sens_robot.read_sensors()
#     reflection = sensor_data['reflection']
#     error = reflection - middle_reflection
#     sign = (error > 0) - (error < 0)

# #    control_robot.rotate_with_speed(turn_rate = error, speed=(base_speed))
#     if error > 0 :
#         control_robot.rotate_with_speed(turn_rate = bang_bang_threshold, speed=(base_speed))
#     else:
#         control_robot.rotate_with_speed(turn_rate = -bang_bang_threshold, speed=(base_speed))

# #    control_robot.rotate_with_speed(turn_rate = error, speed=(base_speed))

#     print('Time:', stopwatch.time(), 'reflection:', reflection, 'error:', error, 'distance:', control_robot.get_distance_mesure(), 'command:', sign * bang_bang_threshold)

#     log.log_data(stopwatch.time(), reflection, error, control_robot.get_distance_mesure(), sign * bang_bang_threshold)
#     actual_distance = control_robot.get_distance_mesure()

#     wait(100)  # Small delay to avoid overwhelming the CPU

# #print(f"Temps final : {stopwatch.time()} ms")

##############################################################################
# PID line following algorithm
##############################################################################

########################################################################################
commit_name = "line_follower_version_P=1.2I=0.2D=0.01_26_09_2025_speed100line_3000_add_antiwindup_limt100"
#Gestion du logging
log = RobotLoging("Time in ms", "Reflection", "Error", "Distance in mm", "Command","Kp", "Ki", "Kd", commit_name=commit_name)


# Initialiser les constantes PID
Kp = 1.2  # Proportional gain
Ki = 0.2 # Integral gain
Kd = 0.01  # Derivative gain
sum_error = 0
last_error = 0
wait_time = 100  #Division par zero
base_speed = 100

command_limit = 100
last_command = 0
control_robot.init_distance_mesurement()

while actual_distance < distance_to_travel:  # Loop forever

    sensor_data = sens_robot.read_sensors()
    reflection = sensor_data['reflection']
    error = reflection - middle_reflection

    if abs(last_command) < command_limit:
        sum_error += error

    if error ==0 : 
        error == 10**-12
    command = error * Kp + sum_error * Ki + ((error - last_error) / (wait_time/1000)) * Kd

    last_error = error

    control_robot.rotate_with_speed(turn_rate = command, speed=(base_speed))

#remove after
    if stopwatch.time() % 1000 < 100:  # toutes les ~1s
        print('Time:', stopwatch.time(), 'reflection:', reflection, 'error:', error,
        'distance:', control_robot.get_distance_mesure(),
        'command:', command, 'Kp:', Kp, 'Ki:', Ki, 'Kd:', Kd)

    log.log_data(stopwatch.time(), reflection, error, control_robot.get_distance_mesure(), command, Kp, Ki, Kd)

    actual_distance = control_robot.get_distance_mesure()

    wait(wait_time)  # Small delay to avoid overwhelming the CPU
