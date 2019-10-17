import numpy as np
from scipy.optimize import curve_fit
import monte_carlo
from monte_carlo import *
import pandas as pd

def func(X, a, b, c):
    x,y = X
    return np.log(a) + b*np.log(x) + c*np.log(y)

def function_test(X, ss, tau, wallmu):
    POSITION_X = +6.2412245E-002
    POSITION_Y = -2.1650000E-001
    POSITION_Z = -1.4630561E-001
    ORIENTATION_X = +5.0683278E-001
    ORIENTATION_Y = -6.4499440E-001
    ORIENTATION_Z = +5.0683278E-001
    orientation_detector = [ORIENTATION_X,ORIENTATION_Y,ORIENTATION_Z]
    position_detector = [POSITION_X,POSITION_Y,POSITION_Z]
    return simulate_ray(X,position_detector,orientation_detector,ss,tau,wallmu,72,36)

# some artificially noisy data to fit
x = np.linspace(0.1,1.1,101)
y = np.linspace(1.,2., 101)
a, b, c = 10., 4., 6.
z = func((x,y), a, b, c) * 1 + np.random.random(101) / 100

# initial guesses for a,b,c:
p0 = 4.440573,5.17422e-06,0.55

df = pd.read_csv("test.csv", header=None)
x = df[1]
y = df[2]
z = df[3]
zr = df[4]
dummy = [123 for i in range(124)]
curve_fit(function_test, (x,y,z), zr, p0)
