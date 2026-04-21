#!/usr/bin/env pybricks-micropython

####################################################################################
# Bloc d'importation des bibliothèques nécessaires
####################################################################################

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor,TouchSensor,ColorSensor,InfraredSensor,UltrasonicSensor,GyroSensor
from pybricks.parameters import Port,Stop,Direction,Button,Color
from pybricks.tools import wait, StopWatch,DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time

####################################################################################
# Classe de contrôle du robot
####################################################################################

class ControlRobot:
    def __init__(self, left_motor_port, right_motor_port, ultrasonic_sensor_port=None):
        self.left_motor = Motor(left_motor_port)
        self.right_motor = Motor(right_motor_port)
        self.robot = DriveBase(self.left_motor, self.right_motor, wheel_diameter=55.5, axle_track=104)

        # Capteur ultrason pas dans la classe sensor j'en ai besoin pour l'arret à distance x
        self.ultrasonic_sensor = UltrasonicSensor(ultrasonic_sensor_port) if ultrasonic_sensor_port else None

    # Par defaut i can us it like move_forward
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

    ############################################################################
    # Bloc des fonction outils
    ############################################################################
    def adjust_speed_for_curve(self, base_speed, error, threshold=10, slow_factor=0.4, boost_factor=1.4):
        """
        Ajuste la vitesse :
        - Ralentit si l'erreur est grande ( serré)
        - Accélère si l'erreur est faible (droite)
        """
        if abs(error) > threshold:
            return int(base_speed * slow_factor)   # ralentir
        else:
            return int(base_speed * boost_factor)  # accélérer

    def adapt_speed_with_obstacle(self, speed, min_distance=200, stop_distance=80, slow_factor=0.5):
        """
        Adapte la vitesse selon la distance mesurée par l'ultrason :
        - Si un obstacle est à moins de stop_distance -> vitesse 0.
        - Si un obstacle est entre stop_distance et min_distance -> ralentir.
        - Sinon -> vitesse normale.
        """
        if self.ultrasonic_sensor:
            distance = self.ultrasonic_sensor.distance()
            if distance <= stop_distance:
                print("obstacle critique à", distance, "mm : STOP")
                return 0
            elif distance < min_distance:
                print("obstacle détecté à", distance, "mm : ralentissement")
                return int(speed * slow_factor)
        return speed

####################################################################################
# Classe des capteurs
####################################################################################

class SensRobot:
    def __init__(self,ultrasonic_sensor_port, color_sensor_port):
        self.color_sensor = ColorSensor(color_sensor_port)
        self.ultrasonic_sensor = UltrasonicSensor(ultrasonic_sensor_port)

    def read_sensors(self):
        color = self.color_sensor.color()
        distance = self.ultrasonic_sensor.distance()
        reflection = self.color_sensor.reflection()
        return {"color": color, "distance": distance, "reflection": reflection}

####################################################################################
# Classe affichage
####################################################################################

class PrintInfo:
    def __init__(self):
        pass

    def print_sensor_info(self, sensor_data):
        print(sensor_data)

# a debuguer

    # def print_on_lcd(self, sensor_data):
    #     ev3 = EV3Brick()
    #     ev3.screen.clear()
    #     ev3.screen.draw_text(sensor_data)

####################################################################################
# Classe logging
####################################################################################

class RobotLoging:
    def __init__(self, *columns, commit_name=None):
        self.log = DataLog(*columns, name=commit_name)

    def log_data(self, *data):
        self.log.log(*data)

    def save_log(self, filename):
        self.log.save(filename)


####################################################################################
# Classe a debuguer connexion entre robots
####################################################################################

# class RobotNetwork:
#     def __init__(self, ssid, password):
#         self.wlan = network.WLAN(network.STA_IF)
#         self.wlan.active(True)
#         self.wlan.connect(ssid, password)
#         while not self.wlan.isconnected():
#             pass
#         print('network config:', self.wlan.ifconfig())


########################################################################################
# Initialisation
########################################################################################

stopwatch = StopWatch()
stopwatch.reset()
stopwatch.resume()

execution_time = 100000  # en ms
distance_to_travel = 30000  # en mm

# Ports
ultrasonic_sensor = Port.S2

left_motor = Port.B
right_motor = Port.C
color_sensor = Port.S3
gyro_sensor = Port.S4

# Création objets
control_robot = ControlRobot(right_motor_port=right_motor, left_motor_port=left_motor,
                             ultrasonic_sensor_port=ultrasonic_sensor)
sens_robot = SensRobot(ultrasonic_sensor_port=ultrasonic_sensor, color_sensor_port=color_sensor)
print_info = PrintInfo()

##############################################################################
# Paramètres du suivi de ligne
##############################################################################

white_line_reflection = 60  
black_line_reflection = 10   
middle_reflection = 40
actual_distance = 0

# Bang-bang
bang_bang_threshold = 15

# PID
Kp = 1.2
# Kp = None
Ki = 0.1
# Ki = None
Kd = 0.001
# Kd = None

sum_error = 0
last_error = 0
wait_time = 80  # en ms
base_speed = 150
last_command = 0

control_robot.init_distance_mesurement()

########################################################################################
# Génération automatique du commit_name
########################################################################################

algo_type = "PID" if (Kp or Ki or Kd) else "BangBang"
date_str = time.strftime("%d_%m_%Y")  # JJ_MM_AAAA

if algo_type == "PID":
    commit_name = "line_follower_{0}_P={1}I={2}D={3}_speed{4}_distance{5}_{6}".format(
        algo_type, Kp, Ki, Kd, base_speed, distance_to_travel, date_str
    )
else:
    commit_name = "line_follower_{0}_threshold{1}_speed{2}_distance{3}_{4}".format(
        algo_type, bang_bang_threshold, base_speed, distance_to_travel, date_str
    )

print("Commit name auto-généré :", commit_name)


########################################################################################
#Implementation du suivis de ligne
#########################################################################################


# #Gestion du logging
# log = RobotLoging("Time in ms", "Reflection", "Error", "Distance in mm", "Command", commit_name=commit_name)

# ##############################################################################
# # Bang-bang line following algorithm
# ##############################################################################

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


########################################################################################
# Boucle PID
########################################################################################

# Gestion du logging 
log = RobotLoging("Time in ms", "Reflection", "Error", "Distance in mm", "Command",
                  "Kp", "Ki", "Kd", commit_name=commit_name)

while actual_distance < distance_to_travel:

    sensor_data = sens_robot.read_sensors()
    reflection = sensor_data['reflection']
    error = reflection - middle_reflection

    # Calcul PID
    sum_error += error
    command = error * Kp + sum_error * Ki + ((error - last_error) / (wait_time / 1000)) * Kd
    last_error = error

    # Ajustement pour virages
    current_speed = control_robot.adjust_speed_for_curve(base_speed, error, threshold=15)

    # Ajustement pour obstacle
    current_speed = control_robot.adapt_speed_with_obstacle(current_speed, min_distance=200, stop_distance=80)

    # Mouvement
    control_robot.rotate_with_speed(turn_rate=command, speed=current_speed)

    # Debug toutes les ~1s
    if stopwatch.time() % 1000 < 100:
        print('Time:', stopwatch.time(),
              'reflection:', reflection,
              'error:', error,
              'distance parcourue:', control_robot.get_distance_mesure(),
              'command:', command,
              'speed:', current_speed)

    log.log_data(stopwatch.time(), reflection, error, control_robot.get_distance_mesure(), command, Kp, Ki, Kd)

    actual_distance = control_robot.get_distance_mesure()
    wait(wait_time)

control_robot.emergency_stop()
print("Parcours terminé")
