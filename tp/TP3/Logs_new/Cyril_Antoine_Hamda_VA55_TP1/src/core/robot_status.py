class RobotStatus:
    def __init__(self):
        # Attributs de base
        self.distance = None
        self.color = None
        self.reflection = None
        self.speed = None
        self.left_speed = None
        self.right_speed = None
        self.last_error = None
        
        # Ajouts pour contrôleurs
        self.controller_type = None  # "bangbang", "P", "PI", "PID"
        self.correction = None       # Correction appliquée
        self.integral_error = None   # Pour PI/PID
        self.derivative_error = None # Pour PID
        self.setpoint = None        # Consigne
        self.iteration = None       # Numéro d'itération
        self.time = None            # Temps écoulé

    def update(self, **kwargs):
        # Met à jour les attributs du robot
        for key, value in kwargs.items():
            if hasattr(self, key) and value is not None:
                setattr(self, key, value)

    def get_status(self):
        # Retourne un dictionnaire avec l'état actuel du robot
        return {
            "distance": self.distance,
            "color": self.color,
            "reflection": self.reflection,
            "speed": self.speed,
            "left_speed": self.left_speed,
            "right_speed": self.right_speed,
            "last_error": self.last_error,
            "controller_type": self.controller_type,
            "correction": self.correction,
            "integral_error": self.integral_error,
            "derivative_error": self.derivative_error,
            "setpoint": self.setpoint,
            "iteration": self.iteration,
            "time": self.time
        }