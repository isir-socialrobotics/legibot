
# Copyright 2023 Mouad Abrini
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.



import numpy as np


class Bezier():
    def __init__(self, ctrl_pts, samples=30):
        self.type = "3D" if len(ctrl_pts[0]) == 3 else "2D"
        self.ctrl_pts = ctrl_pts
        self.t = np.linspace(0, 1, samples)

    def Binomial(self, n, i):
        return np.math.factorial(n) / (np.math.factorial(i) * np.math.factorial(n-i))

    def Jni(self, n, i):
        return self.Binomial(n, i) * (self.t ** i) * (1-self.t) ** (n-i)

    def P(self, n, Bi):
        temp = lambda ind: np.sum([Bi[k][ind] * self.Jni(n, k) for k in range(n + 1)], axis=0)
        return (temp(0), temp(1), temp(2)) if self.type == "3D" else (temp(0), temp(1))
    
    def generate_curve(self):
        return self.P(len(self.ctrl_pts) - 1, self.ctrl_pts)

    def generate_multiple_curves(self, tab):
        return [self.P(len(e) - 1, e) for e in tab]
    

