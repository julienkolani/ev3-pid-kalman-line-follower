import pandas as pd
import matplotlib.pyplot as plt

# Lire le CSV avec séparateur ";"
df = pd.read_csv("robot_logs.csv", sep=",")
print(df.columns.tolist())
df.columns = df.columns.str.strip()
print(df.columns.tolist())  # pour vérifier

# Extraire les colonnes par leur nom (plus sûr que par index numérique)
x_pid = df['x_PID']
y_pid = df['y_PID']

x_state = df['x_state']
y_state = df['y_state']

x_gyro = df['x_gyro']
y_gyro = df['y_gyro']

# --- 1. Définir les coefficients de correction ---
wheel_diameter_correction = 1.02
track_width_correction = 0.95

# --- 3. Correction Offline ---
# → On corrige les coordonnées en appliquant un facteur sur la distance radiale
# ou une rotation sur les angles (selon le type d’erreur constatée)

# Correction simple : mise à l’échelle globale des distances
x_pid_corr = x_pid * wheel_diameter_correction
y_pid_corr = y_pid * wheel_diameter_correction

x_state_corr = x_state * wheel_diameter_correction
y_state_corr = y_state * wheel_diameter_correction

x_gyro_corr = x_gyro * wheel_diameter_correction
y_gyro_corr = y_gyro * wheel_diameter_correction

# Correction angulaire (facultative)
# rotation = np.radians(5)  # exemple d’erreur de cap de 5°
# rotation_matrix = np.array([[np.cos(rotation), -np.sin(rotation)],
#                             [np.sin(rotation),  np.cos(rotation)]])
# x_pid_corr, y_pid_corr = np.dot(rotation_matrix, [x_pid_corr, y_pid_corr])

# Créer le nuage de points
plt.figure(figsize=(8, 6))
plt.scatter(x_pid, y_pid, color="blue", label="PID")
plt.scatter(x_state, y_state, color="red", label="State")
plt.scatter(x_gyro, y_gyro, color="black", label="Gyro")

plt.scatter(x_pid_corr, y_pid_corr, color="green", label="PID Corrigé")
plt.scatter(x_state_corr, y_state_corr, color="orange", label="State Corrigé")
plt.scatter(x_gyro_corr, y_gyro_corr, color="pink", label="Gyro Corrigé")


plt.title("Trajectoires corrigées (Offline Calibration)")
plt.xlabel("X (mm)")
plt.ylabel("Y (mm)")
plt.axis("equal")
plt.legend()
plt.grid(True)
plt.show()
