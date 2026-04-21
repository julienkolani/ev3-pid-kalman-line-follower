import time
from pybricks import ev3brick as brick

class Logger:
    def __init__(self, filename="robot_logs.csv"):
        """
        Initialise le logger pour enregistrer les données dans un fichier CSV.
        :param filename: Nom du fichier CSV où les logs seront stockés.
        """
        self.filename = filename
        
        # Vérifie si le fichier existe déjà, sinon crée-le et écrit l'en-tête
        try:
            with open(self.filename, mode='r') as file:
                pass  # Le fichier existe déjà
        except OSError:
            # Si le fichier n'existe pas, créer le fichier avec un en-tête
            with open(self.filename, mode='w') as file:
                file.write("Time, Speed, Distance, Color, P, PI, PID, x_PID, y_PID, x_state, y_state, x_gyro, y_gyro, state_distance, state_angle\n")
                # En-tête du fichier CSV


    def log(self, time, speed, distance, color, P, PI, PID):
        battery_level = brick.battery.current()
        # Ajouter les données dans le fichier CSV
        with open(self.filename, mode='a') as file:
            data ="{},{},{},{},{},{},{}\n".format(time, speed, distance, color, P, PI, PID)
            file.write(data)
    
    def log_kalman(self, x_PID, y_PID, x_state, y_state, x_gyro, y_gyro, distance, angle):
        with open(self.filename, mode='a') as file:
            data =",150,,,,,,{},{},{},{},{},{},{},{}\n".format(x_PID, y_PID, x_state, y_state, x_gyro, y_gyro, distance, angle)
            file.write(data)  


