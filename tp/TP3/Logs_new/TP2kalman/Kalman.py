import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

class KalmanAngle:
    def __init__(self, process_var=0.00, measurement_var=0.1):
        # État : angle en radians
        self.theta = 0.0
        self.P = 1.0  # variance initiale

        # Bruits
        self.Q = process_var        # bruit du modèle
        self.R = measurement_var    # bruit de la mesure

    def predict(self, delta_theta):
        """Prédiction selon la variation d’angle odométrique."""
        self.theta = delta_theta
        self.P += self.Q
        return self.theta

    def update(self, theta_meas):
        """Mise à jour avec la mesure d’angle PID."""
        y = theta_meas - self.theta  # innovation
        K = self.P / (self.P + self.R)
        self.theta += K * y
        self.P = (1 - K) * self.P
        return self.theta
    
class KalmanFilter2D:
    def __init__(self, dt, v_const, process_var, measurement_var):
        self.dt = dt
        self.v = v_const  # vitesse constante

        # État initial [x, y, theta]
        self.x = np.zeros((3, 1))

        # Covariance initiale
        self.P = np.eye(3)

        # Bruit de processus et de mesure
        self.Q = np.eye(3) * process_var
        self.R = np.eye(2) * measurement_var  # mesure: [x, y]

        # Matrices du modèle
        self.H = np.array([[1, 0, 0],
                           [0, 1, 0]])  # on mesure seulement x et y

    def predict(self):
        theta = self.x[2, 0]
        # Modèle de transition non linéaire
        F = np.array([
            [1, 0, -self.v * self.dt * np.sin(theta)],
            [0, 1,  self.v * self.dt * np.cos(theta)],
            [0, 0,  1]
        ])
        # Prédiction de l’état
        self.x[0, 0] += self.v * self.dt * np.cos(theta)
        self.x[1, 0] += self.v * self.dt * np.sin(theta)
        # Covariance
        self.P = F @ self.P @ F.T + self.Q
        return self.x

    def update(self, z):
        # z = [x_mes, y_mes]
        z = np.array(z).reshape(2, 1)
        y = z - self.H @ self.x               # innovation
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(3) - K @ self.H) @ self.P
        return self.x
    

dt = 0.1
v = 1.0
kf = KalmanFilter2D(dt, v_const=v, process_var=0.01, measurement_var=0.1)

# Nuage de points représentant la ligne (ex : un virage)
df = pd.read_csv("robot_logs.csv", sep=",")
print(df.columns.tolist())
df.columns = df.columns.str.strip()
print(df.columns.tolist())  # pour vérifier

# Extraire les colonnes par leur nom (plus sûr que par index numérique)
x_meas = df['x_gyro']
y_meas = df['y_gyro']

x_est, y_est = [], []

for i in range(len(x_meas)):
    # Prediction + Update
    kf.predict()
    z = [x_meas[i], y_meas[i]]
    state = kf.update(z)
    x_est.append(state[0, 0])
    y_est.append(state[1, 0])

# ----------------------------
# 📈 Visualisation
# ----------------------------
plt.figure(figsize=(8, 5))
#plt.plot(x_true, y_true, 'g-', label='Trajectoire réelle')
plt.scatter(x_meas, y_meas, c='r', s=15, alpha=0.5, label='Mesures bruitées')
plt.plot(x_est, y_est, 'b-', linewidth=2, label='Kalman filtré')
plt.legend()
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.title('Filtre de Kalman 2D pour robot à vitesse constante')
plt.grid(True)
plt.show()