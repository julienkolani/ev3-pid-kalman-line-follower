import socket

class WirelessClient:

    def __init__(self, server_ip, server_port):
        # Initialise la connexion sans fil
        self.server_ip = server_ip
        self.server_port = server_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def connect(self):
        # Établit la connexion au serveur
        self.sock.connect((self.server_ip, self.server_port))

    def send(self, message):
        # Envoie un message au serveur
        self.sock.sendall(message.encode())

    def receive(self, bufsize=1024):
        # Reçoit un message du serveur
        return self.sock.recv(bufsize).decode()

    def close(self):
        # Ferme la connexion
        self.sock.close()
