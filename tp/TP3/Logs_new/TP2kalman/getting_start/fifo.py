class FIFO:
    def __init__(self, max_size=10):
        self.max_size = max_size
        self.data = []

    def push(self, value: int):
        """Ajoute un entier si la file n'est pas pleine.
        Retourne True si ok, False si refusé."""
        if not isinstance(value, int):
            raise TypeError("Seuls les entiers sont autorisés.")
        if len(self.data) >= self.max_size:
            self.pop()
        self.data.append(value)

    def pop(self) -> int | None:
        """Retire et renvoie le plus ancien entier (FIFO)."""
        if self.data:
            return self.data.pop(0)
        return None  # file vide

    def peek(self, n) -> int | None:
        """Regarde le n-ième élément sans le retirer."""
        return self.data[n] if self.data else None

    def __len__(self):
        return len(self.data)  

    def __repr__(self):
        return "IntQueue({})".format(self.data)