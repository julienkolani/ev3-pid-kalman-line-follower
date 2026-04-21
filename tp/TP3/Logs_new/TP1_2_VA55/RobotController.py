#!/usr/bin/env pybricks-micropython
from  pybricks.hubs import EV3Brick
from  pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase


class RobotController:
    
    def __init__(self):
        self.left_motor=Motor(Port.B)
        self.right_motor=Motor(Port.C)

        self.drive_base= DriveBase(self.left_motor, self.right_motor, 55, 104)
    
    def forward(self,speed):

        self.left_motor.run(speed)
        self.right_motor.run(speed)
    
    def stop(self):
           self.logger.info("Stopping robot.")
           self.left_motor.stop()
           self.right_motor.stop()
        
    def rotate(self,angle,aSpeed):
        
        if(angle>0):
            self.left_motor.run_angle(aSpeed,angle)
        else:
            self.right_motor.run_angle(aSpeed,-angle)
        
    def straight(self, distance):

        self.drive_base.straight(distance)

    def turn(self, angle):
        #    self.logger.info(f"Turning by {angle} degrees.")
        self.drive_base.turn(angle)

    def bang_bang(self, angle_speed, speed,error):
        if error > 0 :
            self.drive_base.drive(speed, angle_speed)
        else:
            self.drive_base.drive(speed, -angle_speed)
    
    
