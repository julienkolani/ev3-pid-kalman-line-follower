#!/usr/bin/env pybricks-micropython
import math

class KalmanFilter:

    def __init__(self, q, r, p, initial=0.0):
        #Incertitude du modèle
        self.q = q
        #Incertitude de la mesure
        self.r = r
        self.x = initial
        self.p = p
    
    def prediction(self, u=0.0):
        self.x = self.x + u
        self.p = self.p + self.q
        return self.x

    def update(self, z):
        k = self.p / (self.p + self.r)
        self.x = self.x + k * (z - self.x)
        self.p = (1 - k) * self.p
        return self.x

    def step(self, z, u=0.0):
        self.prediction(u)  
        return self.update(z)
