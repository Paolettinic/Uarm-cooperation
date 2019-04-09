class Cube():

    def __init__(self):
        self._x = 0
        self._y = 0
        self._z = 0
        self.is_on = "table"
        self.color = 0

    def setPosition(self, x: int, y: int, z: int):
        self._x = x
        self._y = y
        self._z = z
        if z>1:
            self.is_on = "cube"

    def set_on(self,place:str):
        self.is_on = place

    def set_index_color(self, color):
        scheme = {
            "cyan":     1,
            "purple":   2,
            "red":      3,
            "green":    4,
            "bue":      5,
            "yellow":   6
        }
        self.color = scheme.get(color, "grey")

    def get_position(self) -> tuple:
        return self._x, self._y, self._z

    def get_x(self) -> int:
        return self._x

    def get_y(self) -> int:
        return self._y

    def get_z(self) -> int:
        return self._z

    def get_index_color(self):
        return self.color

    def get_beneath(self):
        return self.is_on

    def is_on(self, other):
        if not isinstance(other, Cube):
            return True # Becasue it must be somewhere if 'other' is not a cube
        else:
            return other.get_x() == self.get_x() and other.get_y == self.get_y() and other.get_z() < self.get_z()

