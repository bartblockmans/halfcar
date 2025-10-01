import numpy as np
from dataclasses import dataclass
from scipy.interpolate import PchipInterpolator

@dataclass
class CourseInfo:
    name: str = "extreme"
    note: str = "Synthetic extreme course (placeholder)"

@dataclass
class Road:
    x_course: np.ndarray
    z_course: np.ndarray
    course_info: CourseInfo
    _pchip: PchipInterpolator
    _dp: PchipInterpolator
    _ddp: PchipInterpolator

    @classmethod
    def from_samples(cls, x, z):
        p = PchipInterpolator(x, z, extrapolate=True)
        dp = p.derivative(1)
        ddp = p.derivative(2)
        return cls(x_course=np.asarray(x), z_course=np.asarray(z),
                   course_info=CourseInfo(),
                   _pchip=p, _dp=dp, _ddp=ddp)

    def h(self, x):
        return self._pchip(x)

    def hp(self, x):
        return self._dp(x)

    def hpp(self, x):
        return self._ddp(x)

    def hpC(self, x, s_max=2.0):
        s = self._dp(x)
        return np.clip(s, -s_max, s_max)

    def hppC(self, x):
        return self._ddp(x)


# select_course moved to select_course.py (see road.select_course)
