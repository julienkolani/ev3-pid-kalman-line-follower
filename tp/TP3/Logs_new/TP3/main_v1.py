
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
import math
import time

####################################################################################
# Classe de contrôle du robot
####################################################################################

class ControlRobot:
    def __init__(self, left_motor_port, right_motor_port, gyro_port=None, ultrasonic_sensor_port=None, commit_name=None):
        self.left_motor = Motor(left_motor_port)
        self.right_motor = Motor(right_motor_port)
        self.robot = DriveBase(self.left_motor, self.right_motor, wheel_diameter=55.5, axle_track=104)

        # Capteurs
        self.ultrasonic_sensor = UltrasonicSensor(ultrasonic_sensor_port) if ultrasonic_sensor_port else None
        self.gyro = GyroSensor(gyro_port) if gyro_port else None



        # Variables globales
        self.drive_speed = 0
        self.turn_rate = 0
        self.dt = 0.1  # pas de temps (s)

        # États initiaux
        self.x_odo = self.y_odo = self.theta_odo = 0
        self.x_gyro = self.y_gyro = self.theta_gyro = 0
        self.x_model = self.y_model = self.theta_model = 0

        self.last_distance = 0
        self.last_angle = 0

        # Logger
        self.log = DataLog(
            "time",
            "x_odo", "y_odo", "theta_odo",
            "x_gyro", "y_gyro", "theta_gyro",
            "x_model", "y_model", "theta_model",
            "vitesse_lineaire",
            name=commit_name
        )

        self.start_time = time.time()

    ###############################################################################
    # Commandes
    ###############################################################################

    # Par defaut i can us it like move_forward
    def rotate_with_speed(self, speed=100, turn_rate=0):
        self.drive_speed = speed
        self.turn_rate = turn_rate
        self.robot.drive(speed, turn_rate)

    def move_forward(self, speed=100):
        self.robot.drive(speed, 0)

    def move_backward(self, speed=100):
        self.robot.drive(-speed, 0)

    def init_distance_measurement(self):
        self.robot.reset()

    def get_distance_mesure(self):
        return self.robot.distance()
    
    def emergency_stop(self):
        self.robot.stop()


    ###############################################################################
    # Calculs de position
    ###############################################################################
    def get_data_from_odo(self):
        """Odométrie incrémentale."""
        new_distance = self.robot.distance()
        new_angle = self.robot.angle()

        delta_d = new_distance - self.last_distance
        delta_theta = new_angle - self.last_angle

        self.last_distance = new_distance
        self.last_angle = new_angle

        self.theta_odo = new_angle
        self.x_odo += delta_d * math.cos(math.radians(self.theta_odo))
        self.y_odo += delta_d * math.sin(math.radians(self.theta_odo))

        return {'x_odo': self.x_odo, 'y_odo': self.y_odo, 'theta_odo': self.theta_odo}

    def get_data_from_gyro(self):
        """Position basée sur l'angle gyroscope et la vitesse linéaire."""
        if not self.gyro:
            return {'x_gyro': None, 'y_gyro': None, 'theta_gyro': None}

        self.theta_gyro = self.gyro.angle()
        v = self.drive_speed
        self.x_gyro += v * math.cos(math.radians(self.theta_gyro)) * self.dt
        self.y_gyro += v * math.sin(math.radians(self.theta_gyro)) * self.dt

        return {'x_gyro': self.x_gyro, 'y_gyro': self.y_gyro, 'theta_gyro': self.theta_gyro}

    def get_data_from_modele(self):
        """Modèle cinématique (commandes envoyées)."""
        v = self.drive_speed
        w = math.radians(self.turn_rate)

        self.theta_model += w * self.dt
        self.x_model += v * math.cos(self.theta_model) * self.dt
        self.y_model += v * math.sin(self.theta_model) * self.dt

        return {
            'x_model': self.x_model,
            'y_model': self.y_model,
            'theta_model': math.degrees(self.theta_model),
            'vitesse_lineaire': v
        }

    def log_data(self):
        """Enregistre les 3 méthodes dans DataLog."""
        now = round(time.time() - self.start_time, 2)
        odo = self.get_data_from_odo()
        gyro = self.get_data_from_gyro()
        model = self.get_data_from_modele()

        self.log.log(
            now,
            odo['x_odo'], odo['y_odo'], odo['theta_odo'],
            gyro['x_gyro'], gyro['y_gyro'], gyro['theta_gyro'],
            model['x_model'], model['y_model'], model['theta_model'], model['vitesse_lineaire']
        )

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
left_motor = Port.B
right_motor = Port.C
color_sensor = Port.S3
ultrasonic_sensor = Port.S2
gyro_sensor = Port.S4

# Création objets
date_str = time.strftime("%d_%m_%Y")
commit_name = "TP_Kalman_" + date_str

# Création objets
control_robot = ControlRobot(
    left_motor_port=left_motor,
    right_motor_port=right_motor,
    ultrasonic_sensor_port=ultrasonic_sensor,
    gyro_port=gyro_sensor,
    commit_name=commit_name
)

sens_robot = SensRobot(ultrasonic_sensor_port=ultrasonic_sensor, color_sensor_port=color_sensor)
print_info = PrintInfo()

##############################################################################
# Paramètres du suivi de ligne
##############################################################################

white_line_reflection = 60  
black_line_reflection = 10   
middle_reflection = 40
actual_distance = 0

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


pid_log = RobotLoging("Time", "Reflection", "Error", "Distance", "Command", "Kp", "Ki", "Kd", commit_name="PID_" + date_str)


control_robot.init_distance_mesurement()

########################################################################################
# Génération automatique du commit_name
########################################################################################

date_str = time.strftime("%d_%m_%Y")  # JJ_MM_AAAA


########################################################################################
# Boucle PID
########################################################################################

while actual_distance < distance_to_travel:

    sensor_data = sens_robot.read_sensors()
    reflection = sensor_data['reflection']
    error = reflection - middle_reflection

    # Calcul PID
    sum_error += error
    command = error * Kp + sum_error * Ki + ((error - last_error) / (wait_time / 1000)) * Kd
    last_error = error

    # # Ajustement pour virages
    # current_speed = control_robot.adjust_speed_for_curve(base_speed, error, threshold=15)

    # # Ajustement pour obstacle
    # current_speed = control_robot.adapt_speed_with_obstacle(current_speed, min_distance=200, stop_distance=80)

    current_speed = base_speed

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


    # Logs
    control_robot.log_data()  # positions x,y,θ
    pid_log.log_data(stopwatch.time(), reflection, error, control_robot.get_distance_mesure(), command, Kp, Ki, Kd)


    actual_distance = control_robot.get_distance_mesure()
    wait(wait_time)

control_robot.emergency_stop()
print("Parcours terminé")
