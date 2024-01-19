# from __future__ import annotations
import numpy as np
from typing import Union
try:
    from numpy import long
except ImportError:
    long = float


class Point2:
    def __init__(self, *args, **kwargs):
        """
        You can create a Point2 by passing either:
        - a list of 2 elements: [x, y],
        - two scalar numbers: x, y
        - or another Point2: p
        """
        if len(args) == 0:
            self.x = float('nan')
            self.y = float('nan')
        elif len(args) == 1:
            self.x = args[0][0]
            self.y = args[0][1]
        elif len(args) == 2:
            self.x = args[0]
            self.y = args[1]
        else:
            raise ValueError
        self._precision = kwargs.get("precision", 3)   # digits after point | default = 3 (|millimeter)

    def __array__(self):
        return np.array([self.x, self.y])

    def from_SE2(self, inp):
        self.x, self.y = inp.xyt()[:2]
        return self

    def dot(self, other):
        return self.x * other[0] + self.y * other[1]

    def to(self, type_):  # change variables type
        self.x = type_(self.x)
        self.y = type_(self.y)
        return self

    def norm(self):
        return np.linalg.norm(self)

    def unit_vec(self):
        return self / self.norm()

    def __getitem__(self, key: Union[slice, int]):
        if isinstance(key, slice):
            return list([self.x, self.y])[key]
        if key == 0:
            return self.x
        elif key == 1:
            return self.y
        else:
            raise IndexError

    def __len__(self):
        return 2

    # def __add__(self, other: Union[Point2, float, int, long]) -> Point2:
    def __add__(self, other):
        if isinstance(other, Point2):
            return Point2(self.x + other.x, self.y + other.y, precision=self._precision)
        elif isinstance(other, float) or isinstance(other, int) or isinstance(other, long):
            return Point2(self.x + other, self.y + other, precision=self._precision)
        else:
            raise TypeError

    # def __sub__(self, other: Union[Point2, float, int, long]) -> Point2:
    def __sub__(self, other):
        if isinstance(other, Point2):
            return Point2(self.x - other.x, self.y - other.y, precision=self._precision)
        elif isinstance(other, float) or isinstance(other, int) or isinstance(other, long):
            return Point2(self.x - other, self.y - other, precision=self._precision)
        else:
            raise TypeError

    # def __mul__(self, other: Union[Point2, float, int, long]) -> Point2:
    def __mul__(self, other):
        if isinstance(other, Point2):
            return Point2(self.x * other.x, self.y * other.y, precision=self._precision)
        elif isinstance(other, float) or isinstance(other, int) or isinstance(other, long):
            return Point2(self.x * other, self.y * other, precision=self._precision)
        else:
            raise TypeError

    def __truediv__(self, other):
        if isinstance(other, Point2):
            return Point2(self.x / other.x, self.y / other.y, precision=self._precision)
        elif isinstance(other, float) or isinstance(other, int) or isinstance(other, long):
            return Point2(self.x / other, self.y / other, precision=self._precision)
        else:
            raise TypeError

    # def __neg__(self) -> Point2:
    def __neg__(self):
        return Point2(-self.x, -self.y, precision=self._precision)

    # def __eq__(self, other: Union[Point2, None]) -> bool:
    def __eq__(self, other) -> bool:
        if other is None:
            return self.x is None or np.isnan(self.x) is None or self.y is None or np.isnan(self.y)
        else:
            return abs(self.x - other.x) <= 10**-self._precision and \
                   abs(self.y - other.y) <= 10 ** -self._precision

    def __repr__(self) -> str:
        f_precision = ":.{}f".format(self._precision)  # to use in __repr__ and __str__
        return ("Point2({"+f_precision+"}, {"+f_precision+"})").format(self.x, self.y)

    def __str__(self) -> str:
        f_precision = ":.{}f".format(self._precision)  # to use in __repr__ and __str__
        return ("Point2({" + f_precision + "}, {" + f_precision + "})").format(self.x, self.y)

    @staticmethod
    def unit_vec_from_angle(angle_rad):
        return Point2(np.cos(angle_rad), np.sin(angle_rad))


if __name__ == "__main__":
    from spatialmath import SE2
    p_se = Point2().from_SE2(SE2(1, 2, 3))
    p1_ = Point2(110, 120, precision=3)
    p2_ = -Point2(10, 20)
    print(p1_)
    print(p2_ - 3)
    print(p1_ == (p1_+0.0019))
    p_None = Point2()
    print(p_None == None)
