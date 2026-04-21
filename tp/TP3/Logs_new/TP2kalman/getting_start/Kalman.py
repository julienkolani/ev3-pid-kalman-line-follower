
class KalmanAngle:
    def __init__(self, process_var=0.01, measurement_var=0.1):
        # État : angle en radians
        self.theta = 0.0
        self.P = 1.0  # variance initiale

        # Bruits
        self.Q = process_var        # bruit du modèle
        self.R = measurement_var    # bruit de la mesure

    def predict(self, delta_theta):
        """Prédiction selon la variation d’angle odométrique."""
        self.theta += delta_theta
        self.P += self.Q
        return self.theta

    def update(self, theta_meas):
        """Mise à jour avec la mesure d’angle PID."""
        y = theta_meas - self.theta  # innovation
        K = self.P / (self.P + self.R)
        self.theta += K * y
        self.P = (1 - K) * self.P
        return self.theta
"""
# ----------------------------
# 🔧 Simulation
# ----------------------------
dt = 0.1
n_steps = 100

# Véritable angle du robot (en radians)
theta_true = np.deg2rad( np.linspace(0, 45, n_steps) + 5*np.sin(np.linspace(0, 4*np.pi, n_steps)) )

# Odométrie : variation d’angle par pas (avec bruit)
delta_theta_odom = np.gradient(theta_true) + np.random.normal(0, np.deg2rad(0.3), size=n_steps)

# PID : mesure directe de l’angle (bruitée)
theta_pid_meas = theta_true + np.random.normal(0, np.deg2rad(1.5), size=n_steps)

# Kalman
kf = KalmanAngle(process_var=np.deg2rad(0.1)**2, measurement_var=np.deg2rad(1.5)**2)

theta_est = []

for i in range(n_steps):
    kf.predict(delta_theta_odom[i])          # prédiction par odométrie
    theta = kf.update(theta_pid_meas[i])     # correction par PID
    theta_est.append(np.rad2deg(theta))      # conversion en degrés

# ----------------------------
# 📈 Visualisation
# ----------------------------
plt.figure(figsize=(9, 5))
plt.plot(np.rad2deg(theta_true), 'g-', label='Angle réel (°)')
plt.plot(np.rad2deg(theta_pid_meas), 'r:', alpha=0.6, label='Mesure PID (°)')
plt.plot(theta_est, 'b-', linewidth=2, label='Angle filtré (Kalman)')
plt.legend()
plt.xlabel('Temps (itérations)')
plt.ylabel('Angle (degrés)')
plt.title('Filtre de Kalman pour estimation d’angle (PID + odométrie)')
plt.grid(True)
plt.show()

# 🔹 Exemple de valeur finale :
print(f"Angle filtré actuel : {theta_est[-1]:.2f}°")
"""