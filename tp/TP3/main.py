#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
import time
import math


# PARAMÈTRES
####################################################################################

WHEEL_DIAMETER = 55.5   
AXLE_TRACK = 108        

####################################################################################
# CLASSES
####################################################################################

class  ControlRobot:
    def __init__(self, left_motor_port, right_motor_port, wheel_diameter, axle_track):
        self.left_motor = Motor(left_motor_port)
        self.right_motor = Motor(right_motor_port)
        self.robot = DriveBase(self.left_motor, self.right_motor,
                               wheel_diameter=wheel_diameter,
                               axle_track=axle_track)
        self.last_distance = 0
        self.last_angle = 0

    def rotate_with_speed(self, speed=100, turn_rate=0):
        self.robot.drive(speed, turn_rate)

    def  emergency_stop(self):
        self.robot.stop()

    def  reset(self ):
        self.robot.reset()
        self.last_distance = 0
        self.last_angle = 0

    def get_state( self):
        return self.robot.state()

    def get_distance(self):
        return self.robot.distance()

    def get_incremental_distance(self):
        current_distance = self.robot.distance()
        delta = current_distance - self.last_distance
        self.last_distance = current_distance
        return delta

    def get_incremental_angle(self ):
        current_angle = self.robot.angle()
        delta = current_angle - self.last_angle
        self.last_angle = current_angle
        return delta


class SensRobot:
    def __init__(self, color_sensor_port, gyro_sensor_port):
        self.color_sensor = ColorSensor(color_sensor_port)
        self.gyro_sensor = GyroSensor(   gyro_sensor_port)
        wait(2500  )
        self.gyro_sensor.reset_angle(0)
        wait(1000 )
        self.bias = 0
        self.bias_samples = []

    def read_sensors(self):
        """Lecture des capteurs + correction du biais du gyro"""
        reflection = self.color_sensor.reflection()
        raw_angle = self.gyro_sensor.angle()

        # Moyenne des 50 premières valeurs pour estimer le biais
        if len(self.bias_samples) < 50:
            self.bias_samples.append(raw_angle)
            self.bias = sum(self.bias_samples) / len(self.bias_samples)

        corrected_angle = -(raw_angle - self.bias)  # inversion du signe + compensation biais
        return {"reflection": reflection, "gyro_angle": corrected_angle}


class PositionTracker:
    def __init__(self):
        self.positions = {
            'pid':   {'x': 0.0, 'y': 0.0, 'theta': 0.0},
                'state': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'gyro':  {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        }

    def  update_position_pid( self, d, da):
        """Mise à jour de la position selon les consignes PID."""
        rad = math.radians(da)
        self.positions['pid']['theta'] += rad
        t = self.positions['pid']['theta']
        self.positions['pid']['x'] += math.cos(t) * d
        self.positions['pid']['y'] += math.sin(t) * d

    def update_position_state( self, d, da):
        """Mise à jour de la position selon la fonction state()."""
        rad = math.radians(da)
        self.positions['state']['theta'] += rad
        t = self.positions['state']['theta']
        self.positions['state']['x'] += math.cos(t) * d
        self.positions['state']['y'] += math.sin(t) * d

    def update_position_gyro (self, v, ag, dt):
        """Position à partir du gyroscope (angle corrigé et signe cohérent)."""
        self.positions['gyro']['theta'] = math.radians(ag)
        d = v * dt / 1000.0
        t = self.positions['gyro']['theta']
        self.positions['gyro']['x'] += math.cos(t) * d
        self.positions['gyro']['y'] += math.sin(t) * d

    def get_position( self, method):
        return self.positions[method]


class PIDController :
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.sum_error = 0
        self.last_error = 0

    def compute(self, error, dt):
        self.sum_error += error
        derivative = (error - self.last_error) / (dt / 1000.0) if dt > 0 else 0
        output = self.kp * error + self.ki * self.sum_error + self.kd * derivative
        self.last_error = error
        return output


class RobotLogging:
    def __init__(self, name):
        self.log = DataLog(
            "time",
            "x_pid", "y_pid", "theta_pid",
                "x_state", "y_state", "theta_state",
            "x_gyro", "y_gyro", "theta_gyro" ,
            "velocity", "turn_rate",
            "reflection", "error", "command" ,
            name=name
        )

    def log_data (self, t, pos,   refl, err, cmd, vel, turn_rate):
        self.log.log(
            t,
            pos['pid']['x'], pos['pid']['y'], math.degrees(pos['pid']['theta']),
            pos['state']['x'], pos['state']['y'], math.degrees(pos['state']['theta']),
            pos['gyro']['x'], pos['gyro']['y'], math.degrees(pos['gyro']['theta']),
            vel, turn_rate, refl, err,  cmd
        )

####################################################################################
# INITIALISATION
####################################################################################

left_motor_port =   Port.C
right_motor_port = Port.B
color_sensor_port =  Port.S3
gyro_sensor_port = Port.S1

ev3 = EV3Brick()
control = ControlRobot(left_motor_port, right_motor_port, WHEEL_DIAMETER, AXLE_TRACK)
sensors = SensRobot(color_sensor_port, gyro_sensor_port)
tracker = PositionTracker()
clock = StopWatch()

# === PARAMÈTRES DE CONFIGURATION ===
middle_reflection =  40
base_speed=120          # vitesse moyenne (mm/s)
wait_time =  15            # période de boucle (ms)
k = 0.58                  # facteur d’amortissement PID
pid = PIDController(1.2, 0.1, 0.001)
distance_to_travel =24000
sample_time =  100         # période d’échantillonnage (ms)

commit_name = "D{0}_V{1}_{2}".format(
    WHEEL_DIAMETER, AXLE_TRACK, time.strftime("%d_%m_%Y")
)
logger = RobotLogging(commit_name)

control.reset()
clock.reset()
clock.resume()

last_sample_time = 0
actual_distance = 0
last_dist_state = 0
last_angle_state = 0

print("Démarrage du robot... Circuit en 8")
print("=" * 50)

####################################################################################
# BOUCLE PRINCIPALE
####################################################################################

while actual_distance < distance_to_travel:
    t = clock.time()
    data = sensors.read_sensors()
    reflection = data['reflection']
    gyro_angle = data['gyro_angle']
    error = reflection - middle_reflection

    # PID + amortissement
    cmd =  pid.compute(error, wait_time)
    cmd *=  k
    control.rotate_with_speed(base_speed, cmd)

    # Échantillonnage
    if (t - last_sample_time) >= sample_time:
        dt = t - last_sample_time
        d_pid = control.get_incremental_distance()
        a_pid = control.get_incremental_angle()
        dist_state, velocity, a_state, turn_rate = control.get_state()

        delta_d_state  = dist_state - last_dist_state
        delta_a_state = a_state - last_angle_state
        last_dist_state  = dist_state
        last_angle_state = a_state

        # Mise à jour des positions
        tracker.update_position_pid (d_pid, a_pid)
        tracker.update_position_state(delta_d_state, delta_a_state)
        tracker.update_position_gyro(velocity if velocity else base_speed, gyro_angle, dt)

        # Log uniquement les trois méthodes
        logger.log_data(t, tracker.positions, reflection, error, cmd, velocity, turn_rate)

        if t % 1000 < sample_time:
            pos = tracker.get_position('state')
            print('T:', t, 'ms | Dist:', int(actual_distance),
                  'mm | State: x=', int(pos['x']),
                  'y=', int(pos['y']),
                  'θ=', int(math.degrees(pos['theta'])), '°')

        last_sample_time=t

    actual_distance =control.get_distance()
    wait(wait_time )

control.emergency_stop( )
