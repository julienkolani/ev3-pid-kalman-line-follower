class EventManager:

    def __init__(self):
        # Liste pour stocker les événements
        self.events = []

    def trigger(self, event_name, data=None):
        # Ajouter un événement à la liste
        self.events.append((event_name, data))

    def get_events(self):
        # Retourner et vider la liste des événements
        events = self.events[:]
        self.events.clear()
        return events
