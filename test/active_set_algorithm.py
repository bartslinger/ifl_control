#!/usr/bin/env python

import numpy as np

print("Active Set Algorithm")

def active_set_algorithm(B, wv, u_up, u_lo, v):

	# multiply with weights
	Wv = np.diag(wv)
	A = Wv.dot(B)
	b = Wv.dot(v)

	# bookkeeping variables
	W = np.zeros(np.shape(b))
	u_k = np.zeros(np.shape(b))

	# iterate a maximum of 10 times
	for i in range(0, 4):

		# construct least squares problem with free actuators
		Af = A[:, W==0]

		# least squares solution
		pp, residuals, rank, s = np.linalg.lstsq(Af,b,rcond=None)

		# construct entire p including constrained actuators
		p = np.zeros(np.shape(b))
		p[W==0] = pp

		# check feasibility of the solution
		largest_alpha = 0
		largest_alpha_idx = 0

		for i, u in enumerate(u_k):
			alpha_up   = p[i] / (u_up[i] - u_k[i])				
			alpha_down = p[i] / (u_lo[i] - u_k[i])
			if (max(alpha_up, alpha_down) > largest_alpha):
				largest_alpha = max(alpha_up, alpha_down)
				largest_alpha_idx = i
				#print("alpha", i, largest_alpha)

		if largest_alpha < 1:
			u_k = u_k + p
			print("feasible")
			break
		else:
			print("not feasible")
			# add to the working set
			# the sign of p[i] tells if it was upper or lower bound
			W[largest_alpha_idx] = np.sign(p[largest_alpha_idx])
			print("active set", W)
			# scale the solution to be feasible
			u_k = u_k + p/largest_alpha
			print("u_k", u_k)
	print("\n\n")
	return u_k

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
print("\nTest 1:")
u = active_set_algorithm(B, Wv, u_up, u_lo, v)
print(u)

# Make sure this solution satifies the demand
actual_out = B.dot(u)
diff = np.linalg.norm(actual_out - v)
if diff > 1e-8:
	print("Test 1 broken")

#############################
# Test 2: Ask too much roll #
#############################

# Virtual control / Desired output
v = np.array([100.0, 0.0, 0.0, 0.0])

# Calculate required control command
print("\nTest 2:")
print("Request:")
print(v)
u = active_set_algorithm(B, Wv, u_up, u_lo, v)
print(u)

# Could not fulfill the demand, 80 is maximum
actual_out = B.dot(u)
print("Actual output:")
print(np.round(actual_out,8))

##########################################
# Test 3: Ask some roll and too much yaw #
##########################################
# this should prioritize roll over yaw

v = np.array([20.0, 0.0, 5.0, 0.0])

print("\nTest 3:")
print("Request:")
print(v)
u = active_set_algorithm(B, Wv, u_up, u_lo, v)
print("Command:")
print(u)

actual_out = B.dot(u)
print("Actual output:")
print(np.round(actual_out,8))
