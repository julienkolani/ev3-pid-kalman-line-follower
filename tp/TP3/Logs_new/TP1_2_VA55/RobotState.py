#!/usr/bin/env pybricks-micropython
from  pybricks.hubs import EV3Brick
from  pybricks.ev3devices import ColorSensor as color
from pybricks.parameters import Port
from ColorSensor import ColorSensor
from DistanceSensor import DistanceSensor
from RobotController import RobotController
import time 

class RobotState:
    def __init__(self):
        self.color_sensor = ColorSensor()
        self.distance_sensor = DistanceSensor()
        self.robot_controller = RobotController()
        self.current_color = None
        self.current_distance = 0
        self.timestamp = time.time()
        self.driven_distance = 0
     
        
    def update_state(self):
        return self.robot_controller.drive_base.state()
        # self.current_color = self.color_sensor.get_color()
        # self.current_distance = self.distance_sensor.get_distance()
        # self.driven_distance = self.distance_sensor.get_driven_distance(self.robot_controller.drive_base)

        # self.speed = self.robot_controller.drive_base.speed()
        
        # self.angular_speed = self.robot_controller.drive_base.angle()
        # return  self.current_distance, self.driven_distance, self.speed, self.angular_speed
