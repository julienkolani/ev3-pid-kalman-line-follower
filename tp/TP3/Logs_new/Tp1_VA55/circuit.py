import pandas as pd
import math
import matplotlib.pyplot as plt
from kalman import KalmanFilter

df = pd.read_csv("robot_log.csv")
df.columns = [c.strip().lower() for c in df.columns]

x_raw = [0]  
y_raw = [0]
x_kalman = [0]
y_kalman = [0]

q = 0.9
r = 5
p = 1
initial = 0.0
fk = KalmanFilter(q, r, p, initial)


for i in range(1, len(df)):
    distance = (df.loc[i, 'distance'] - df.loc[max(i-1,0) , 'distance'])
    time=df.loc[i,'time']
    theta_raw = df.loc[i, 'gyro_cumulative_angle'] 
    theta_raw = theta_raw+(8.9*time) * 0.97

    #  filtre de Kalman 
    theta_corr = fk.step(theta_raw)
    
    # trajectoire normale
    theta_rad_raw = math.radians(theta_raw)
    x_raw.append(x_raw[-1] + math.cos(theta_rad_raw) * distance)
    y_raw.append(y_raw[-1] + math.sin(theta_rad_raw) * distance)
    
    #  Trajectoire corrigée
    theta_rad_corr = math.radians(theta_corr)
    x_kalman.append(x_kalman[-1] + math.cos(theta_rad_corr) * distance)
    y_kalman.append(y_kalman[-1] + math.sin(theta_rad_corr) * distance)

#  Affichage
plt.figure(figsize=(8, 8))
plt.plot(x_raw, y_raw, color='blue', linestyle='--', label='Sans filtre (gyro brut)')
plt.plot(x_kalman, y_kalman, color='red', linestyle='--', marker='o', label='Filtre de Kalman')
plt.title("Trajectoires du robot avec et sans correction Kalman")
plt.xlabel("X (mm)")
plt.ylabel("Y (mm)")
plt.grid(True)
plt.axis('equal')
plt.legend()

# évolution de l'angle gyro 
time=df['time']
angle=df['gyro_cumulative_angle']
corr=angle+(8.9*time)
plt.figure(figsize=(10, 5))
plt.plot(df['time'], df['gyro_cumulative_angle'], label='Gyro brut', color='blue')
plt.plot(time,corr,color='red',label='Gyro corrigé')
plt.title("Évolution de l'angle gyroscopique au cours du temps")
plt.xlabel("Temps (s)")
plt.ylabel("Angle cumulé (°)")
plt.legend()
plt.grid(True)
plt.show()

plt.show()
