class BangBangController:

    def __init__(self, threshold, delta):
        # Initialise le contrôleur Bang-Bang.
        self.threshold = threshold  # valeur de consigne (ex: 50%)
        self.delta = delta          # amplitude de correction

    def compute(self, value):
        if value < self.threshold:  # Sur noir
            return self.delta       # Tourne à DROITE (+)
        return -self.delta          # Sur blanc, tourne à GAUCHE (-)