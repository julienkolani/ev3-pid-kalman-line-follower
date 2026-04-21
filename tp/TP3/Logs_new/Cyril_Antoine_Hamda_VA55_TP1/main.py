#!/usr/bin/env pybricks-micropython

import time
import math
from src.actuators.motor_controller import MotorController
from src.sensors.color_sensor import ColorSensorWrapper
from src.calculs.bangbang import BangBangController
from src.calculs.P import PController
from src.calculs.PI import PIController
from src.calculs.PID import PIDController
from src.core.logger import Logger
from src.core.robot_status import RobotStatus
from pybricks.parameters import Port
from pybricks.ev3devices import ColorSensor

# =============================================================================
# CONFIGURATION HARDWARE
# =============================================================================
LEFT_MOTOR_PORT = Port.B
RIGHT_MOTOR_PORT = Port.C
COLOR_SENSOR_PORT = Port.S3

# =============================================================================
# CONFIGURATION DRIVEBASE
# =============================================================================
WHEEL_DIAMETER = 55     # mm - Diamètre des roues
AXLE_TRACK = 104        # mm - Distance entre roues

# =============================================================================
# CONFIGURATION CONTRÔLEURS
# =============================================================================
# Thresholds et setpoints (basé sur mesures: blanc=68, noir=9)
OPTIMAL_THRESHOLD = 39  # Seuil optimal calculé: (68+9)/2 ≈ 39

# Bang-Bang Controller
BANGBANG_THRESHOLD = OPTIMAL_THRESHOLD
BANGBANG_DELTA = 50
BANGBANG_SPEED = 200  # mm/s

# Proportional Controller
P_KP = 2.0
P_SETPOINT = OPTIMAL_THRESHOLD
P_SPEED = 300  # mm/s

# Proportional-Integral Controller
PI_KP = 1.5
PI_KI = 0.10
PI_SETPOINT = OPTIMAL_THRESHOLD
PI_SPEED = 250  # mm/s

# Proportional-Integral-Derivative Controller
PID_KP = 1.2
PID_KI = 0.06
PID_KD = 0.6
PID_SETPOINT = OPTIMAL_THRESHOLD
PID_SPEED = 150  # mm/s

# Paramètres généraux
LOOP_ITERATIONS = 200   # Nombre d'itérations par test
LOOP_DELAY = 0.1       # Délai entre itérations (secondes)

# =============================================================================
# FONCTIONS DE TEST DES CONTRÔLEURS
# =============================================================================

def test_bangbang(motors, color_sensor, logger, status):
    """Test du contrôleur Bang-Bang (tout ou rien)"""
    print("=== DÉBUT TEST BANG-BANG ===")
    
    controller = BangBangController(
        threshold=BANGBANG_THRESHOLD, 
        delta=BANGBANG_DELTA
    )
    base_speed = BANGBANG_SPEED
    start_time = time.time()
    
    for i in range(LOOP_ITERATIONS):
        reflection = color_sensor.get_reflection()
        correction = controller.compute(reflection)
        
        # Commande du robot avec DriveBase
        motors.drive_base.drive(base_speed, correction)
        
        # Calcul des vitesses individuelles pour affichage
        left_speed = base_speed - correction
        right_speed = base_speed + correction
        
        # Mise à jour du statut
        status.update(
            controller_type="BangBang",
            iteration=i,
            time=time.time() - start_time,
            reflection=reflection,
            error=None,
            correction=correction,
            speed=base_speed,
            left_speed=left_speed,
            right_speed=right_speed
        )
        
        # Logging
        logger.log(status.get_status())
        
        # Affichage console
        #print(f"BB | Iter: {i:3d} | Refl: {reflection:2d} | Corr: {correction:+4.0f}")
        time.sleep(LOOP_DELAY)
    
    motors.drive_base.stop()
    print("=== FIN TEST BANG-BANG ===\n")


def test_proportional(motors, color_sensor, logger, status):
    """Test du contrôleur Proportionnel (P)"""
    print("=== DÉBUT TEST PROPORTIONNEL ===")
    
    controller = PController(kp=P_KP, setpoint=P_SETPOINT)
    base_speed = P_SPEED
    start_time = time.time()
    
    for i in range(LOOP_ITERATIONS):
        reflection = color_sensor.get_reflection()
        correction = controller.compute(reflection)
        
        # Commande du robot
        motors.drive_base.drive(base_speed, correction)
        
        # Calculs pour affichage et logging
        error = controller.setpoint - reflection
        left_speed = base_speed - correction
        right_speed = base_speed + correction
        
        # Mise à jour du statut
        status.update(
            controller_type="Proportional",
            iteration=i,
            time=time.time() - start_time,
            reflection=reflection,
            error=error,
            correction=correction,
            setpoint=controller.setpoint,
            speed=base_speed,
            left_speed=left_speed,
            right_speed=right_speed
        )
        
        # Logging
        logger.log(status.get_status())
        
        # Affichage console
        #print(f"P | Iter: {i:3d} | Refl: {reflection:2d} | Err: {error:+3.0f} | Corr: {correction:+4.0f}")
        time.sleep(LOOP_DELAY)
    
    motors.drive_base.stop()
    print("=== FIN TEST PROPORTIONNEL ===\n")


def test_pi_controller(motors, color_sensor, logger, status):
    """Test du contrôleur Proportionnel-Intégral (PI)"""
    print("=== DÉBUT TEST PROPORTIONNEL-INTÉGRAL ===")
    
    controller = PIController(kp=PI_KP, ki=PI_KI, setpoint=PI_SETPOINT)
    base_speed = PI_SPEED
    start_time = time.time()
    
    for i in range(LOOP_ITERATIONS):
        reflection = color_sensor.get_reflection()
        correction = controller.compute(reflection)
        
        # Commande du robot
        motors.drive_base.drive(base_speed, correction)
        
        # Calculs pour affichage détaillé
        error = controller.setpoint - reflection
        integral = sum(controller.errors)
        proportional_part = controller.kp * error
        integral_part = controller.ki * integral
        left_speed = base_speed - correction
        right_speed = base_speed + correction
        
        # Mise à jour du statut
        status.update(
            controller_type="PI",
            iteration=i,
            time=time.time() - start_time,
            reflection=reflection,
            error=error,
            correction=correction,
            setpoint=controller.setpoint,
            integral_error=integral,
            speed=base_speed,
            left_speed=left_speed,
            right_speed=right_speed
        )
        
        # Logging
        logger.log(status.get_status())
        
        # Affichage console détaillé
        #print(f"PI | Iter: {i:3d} | Refl: {reflection:2d} | P: {proportional_part:+4.0f} | I: {integral_part:+4.0f} | Corr: {correction:+4.0f}")
        time.sleep(LOOP_DELAY)
    
    motors.drive_base.stop()
    print("=== FIN TEST PROPORTIONNEL-INTÉGRAL ===\n")


def test_pid_controller(motors, color_sensor, logger, status):
    """Test du contrôleur Proportionnel-Intégral-Dérivé (PID)"""
    print("=== DÉBUT TEST PROPORTIONNEL-INTÉGRAL-DÉRIVÉ ===")
    
    controller = PIDController(
        kp=PID_KP, 
        ki=PID_KI, 
        kd=PID_KD, 
        setpoint=PID_SETPOINT
    )
    base_speed = PID_SPEED
    start_time = time.time()
    
    x = 0
    y = 0
    
    for i in range(LOOP_ITERATIONS):
        reflection = color_sensor.get_reflection()
        correction = controller.compute(reflection)
        
        # Commande du robot
        motors.drive_base.drive(base_speed, correction)
        
        # Calculs pour affichage des 3 composantes PID
        error = controller.setpoint - reflection
        integral = sum(controller.errors)
        derivative = error - controller.last_error if i > 0 else 0
        proportional_part = controller.kp * error
        integral_part = controller.ki * integral
        derivative_part = controller.kd * derivative
        left_speed = base_speed - correction
        right_speed = base_speed + correction
        
        # Mise à jour du statut
        status.update(
            controller_type="PID",
            iteration=i,
            time=time.time() - start_time,
            reflection=reflection,
            error=error,
            correction=correction,
            setpoint=controller.setpoint,
            integral_error=integral,
            derivative_error=derivative,
            speed=base_speed,
            left_speed=left_speed,
            right_speed=right_speed
        )
        
        # Logging
        logger.log(status.get_status())
        
        x += math.cos(math.radians(motors.drive_base.angle())) * motors.drive_base.distance()
        y += math.sin(math.radians(motors.drive_base.angle())) * motors.drive_base.distance()
        motors.drive_base.reset()
        print(str(x) + ", " + str(y))
        
        # Affichage console avec les 3 composantes
        #print(f"PID | Iter: {i:3d} | P: {proportional_part:+4.0f} | I: {integral_part:+4.0f} | D: {derivative_part:+4.0f} | Corr: {correction:+4.0f}")
        time.sleep(LOOP_DELAY)
    
    motors.drive_base.stop()
    print("=== FIN TEST PROPORTIONNEL-INTÉGRAL-DÉRIVÉ ===\n")

# =============================================================================
# FONCTIONS UTILITAIRES ET TESTS
# =============================================================================
 
def find_color_sensor():
    """Trouve automatiquement le port du capteur de couleur"""
    print("=== RECHERCHE DU CAPTEUR DE COULEUR ===")
    ports = [Port.S1, Port.S2, Port.S3, Port.S4]
    
    for port in ports:
        try:
            sensor = ColorSensor(port)
            reflection = sensor.reflection()
            #print(f"✅ CAPTEUR TROUVÉ sur {port} - Réflexion: {reflection}")
            return port
        except Exception as e:
            print("❌ Pas de capteur")
    
    print("⚠️ AUCUN CAPTEUR TROUVÉ !")
    return None


def test_motor_balance():
    """Test individuel des moteurs pour vérifier l'équilibrage"""
    print("=== TEST D'ÉQUILIBRAGE DES MOTEURS ===")
    motors = MotorController(LEFT_MOTOR_PORT, RIGHT_MOTOR_PORT, WHEEL_DIAMETER, AXLE_TRACK)
    
    # Test moteur gauche
    print("🔄 Test moteur GAUCHE à 300 deg/s pendant 2s...")
    motors.left_motor.run(300)
    time.sleep(2)
    motors.drive_base.stop()
    time.sleep(1)
    
    # Test moteur droit
    print("🔄 Test moteur DROIT à 300 deg/s pendant 2s...")
    motors.right_motor.run(300)
    time.sleep(2)
    motors.drive_base.stop()
    
    print("✅ Test d'équilibrage terminé\n")


def calibrate_color_sensor():
    """Calibration du capteur de couleur sur blanc et noir"""
    print("=== CALIBRATION DU CAPTEUR DE COULEUR ===")
    color_sensor = ColorSensorWrapper(COLOR_SENSOR_PORT)
    
    print("📍 Placez le robot sur la LIGNE NOIRE et appuyez sur le bouton central...")
    # TODO: Ajouter attente bouton
    time.sleep(3)  # Temporaire
    black_reflection = color_sensor.get_reflection()
    #print(f"⚫ Réflexion NOIR: {black_reflection}")
    
    print("📍 Placez le robot sur le BLANC et appuyez sur le bouton central...")
    # TODO: Ajouter attente bouton  
    time.sleep(3)  # Temporaire
    white_reflection = color_sensor.get_reflection()
    #print(f"⚪ Réflexion BLANC: {white_reflection}")
    
    optimal_threshold = (black_reflection + white_reflection) / 2
    #print(f"🎯 Seuil optimal calculé: {optimal_threshold:.1f}")
    #print(f"🔧 Ajustez OPTIMAL_THRESHOLD = {optimal_threshold:.0f} dans la configuration\n")
    
    return black_reflection, white_reflection, optimal_threshold


def print_configuration():
    """Affiche la configuration actuelle"""
    print("=== CONFIGURATION ACTUELLE ===")
    # print(f"Hardware:")
    # print(f"  - Moteur gauche: {LEFT_MOTOR_PORT}")
    # print(f"  - Moteur droit: {RIGHT_MOTOR_PORT}")
    # print(f"  - Capteur couleur: {COLOR_SENSOR_PORT}")
    # print(f"DriveBase:")
    # print(f"  - Diamètre roues: {WHEEL_DIAMETER} mm")
    # print(f"  - Écartement: {AXLE_TRACK} mm")
    # print(f"Contrôleurs:")
    # print(f"  - Seuil optimal: {OPTIMAL_THRESHOLD}")
    # print(f"  - Bang-Bang: Δ={BANGBANG_DELTA}, V={BANGBANG_SPEED} mm/s")
    # print(f"  - Proportionnel: Kp={P_KP}, V={P_SPEED} mm/s")
    # print(f"  - PI: Kp={PI_KP}, Ki={PI_KI}, V={PI_SPEED} mm/s")
    # print(f"  - PID: Kp={PID_KP}, Ki={PID_KI}, Kd={PID_KD}, V={PID_SPEED} mm/s")
    print()

# =============================================================================
# FONCTION PRINCIPALE
# =============================================================================

def main():
    """Fonction principale - Exécute les tests des contrôleurs"""
    print("🚀 DÉMARRAGE DU PROGRAMME DE SUIVI DE LIGNE")
    print("=" * 50)
    
    # Affichage de la configuration
    print_configuration()
    
    # Initialisation des composants
    print("🔧 Initialisation des composants...")
    motors = MotorController(LEFT_MOTOR_PORT, RIGHT_MOTOR_PORT, WHEEL_DIAMETER, AXLE_TRACK)
    color_sensor = ColorSensorWrapper(COLOR_SENSOR_PORT)
    robot_status = RobotStatus()
    
    # Initialisation des loggers
    print("📝 Initialisation des loggers...")
    logger_bb = Logger("logs_bangbang")
    logger_p = Logger("logs_proportional")
    logger_pi = Logger("logs_pi")
    logger_pid = Logger("logs_pid")
    
    print("✅ Initialisation terminée\n")
    
    # Exécution des tests (décommenter selon besoin)
    
    # Test Bang-Bang
    # test_bangbang(motors, color_sensor, logger_bb, robot_status)
    
    # Test Proportionnel
    # test_proportional(motors, color_sensor, logger_p, robot_status)
    
    # Test Proportionnel-Intégral
    # test_pi_controller(motors, color_sensor, logger_pi, robot_status)
    
    # Test Proportionnel-Intégral-Dérivé
    test_pid_controller(motors, color_sensor, logger_pid, robot_status)
    
    # Affichage des fichiers de logs générés
    print("📋 FICHIERS DE LOGS GÉNÉRÉS :")
    # print(f"  - Bang-Bang: {logger_bb.filepath}")
    # print(f"  - Proportionnel: {logger_p.filepath}")
    #print(f"  - PI: {logger_pi.filepath}")
    # print(f"  - PID: {logger_pid.filepath}")
    
    print("\n🏁 PROGRAMME TERMINÉ")

# =============================================================================
# POINT D'ENTRÉE
# =============================================================================

if __name__ == "__main__":
    # Tests et calibration (décommenter selon besoin)
    # find_color_sensor()
    # test_motor_balance()
    # calibrate_color_sensor()
    main()