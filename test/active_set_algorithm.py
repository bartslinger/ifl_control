#!/usr/bin/env python

import numpy as np

print("Active Set Algorithm")

def active_set_algorithm(B, wv, v):
	Wv = np.diag(wv)
	A = Wv.dot(B)
	b = Wv.dot(v)
	x, residuals, rank, s = np.linalg.lstsq(A,b,rcond=None)
	return x

#####################
# Test 1: Just roll #
#####################

# Control effectiveness matrix
B = np.array([[-20.0,  20.0, 20.0, -20.0],   # Roll
              [ 17.0, -17.0, 17.0, -17.0],   # Pitch
              [  0.7,   0.7, -0.7,  -0.7],   # Yaw
              [ -1.2,  -1.2, -1.2,  -1.2]])  # Thrust
# Weights
Wv = [100, 100, 1, 10]

# Control upper and lower limits
u_up = [ 1.0,  1.0,  1.0,  1.0]
u_lo = [-1.0, -1.0, -1.0, -1.0]

# Virtual control / Desired output
v = [10.0, 0.0, 0.0, 0.0]

# Calculate required control command
u = active_set_algorithm(B, Wv, v)
print("Test 1:")
print(u)

# Make sure this solution satifies the demand
actual_out = B.dot(u)
diff = np.linalg.norm(actual_out - v)
if diff > 1e-8:
	print("Test 1 broken")
