#!/usr/bin/env python

import numpy as np

print("Active Set Algorithm")

def active_set_algorithm(B, wv, u_up, u_lo, v):

	# multiply with weights
	Wv = np.diag(wv)
	A = Wv.dot(B)
	b = Wv.dot(v)

	# bookkeeping variables
	W = np.zeros(np.shape(u_up), dtype=np.int8)
	u_k = np.zeros(np.shape(u_up))

	# iterate a maximum of 10 times
	for i in range(0, 10):
		free_indices = np.where(W==0)[0]
		active_indices = np.where(W!=0)[0]

		#print("W:", W)

		p = np.zeros(np.shape(u_up))

		if len(free_indices) > 0:
			# construct least squares problem with free actuators
			Af = A[:, free_indices]
			d  = b - A.dot(u_k)

			# least squares solution
			pp, residuals, rank, s = np.linalg.lstsq(Af,d,rcond=None)
			
			# construct entire p including constrained actuators
			p[free_indices] = pp
		else:
			print("no free actuators")

		# check feasibility of the solution
		smallest_alpha = 1
		smallest_alpha_idx = 0

		for z in free_indices:
			alpha = 1			
			if u_k[z] + p[z] > u_up[z]:
				alpha = (u_up[z] - u_k[z]) / p[z]

			elif u_k[z] + p[z] < u_lo[z]:
				alpha = (u_lo[z] - u_k[z]) / p[z]


			if (alpha < smallest_alpha):
				smallest_alpha = alpha
				smallest_alpha_idx = z
				#print("alpha", i, smallest_alpha)

		if smallest_alpha >= 1:
			u_k = u_k + p
			# calculate lagrangian multipliers
			ATAub = A.transpose().dot(A.dot(u_k) - b)
			lamb = -W * ATAub
			#print("feasible, lambda:", lamb)
			#print("u_k", u_k)
			if len(active_indices) == 0:
				break
			smallest_lambda = 0
			smallest_lambda_idx = 0
			for z in active_indices:
				if lamb[z] < smallest_lambda:
					smallest_lambda = lamb[z]
					smallest_lambda_idx = z
			if smallest_lambda < 0:
				print("smallest lambda value:", smallest_lambda)
				W[smallest_lambda_idx] = 0
				print("++++++++++++++++++++++++++++++++++remove from working set:", smallest_lambda_idx, W)
			else:
				print("solution found")
				break
		else:
			#print("not feasible")
			# add to the working set
			# the sign of p[i] tells if it was upper or lower bound
			W[smallest_alpha_idx] = np.sign(p[smallest_alpha_idx])
			#print("add to working set:", smallest_alpha_idx)
			# scale the solution to be feasible
			u_k = u_k + p*smallest_alpha

	print("Iterations:", i)
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
Wv = [1000, 1000, 1, 100]

# Control upper and lower limits
u_up = [ 1.0,  1.0,  1.0,  1.0]
u_lo = [-1.0, -1.0, -1.0, -1.0]

# Virtual control / Desired output
v = [10.0, 0.0, 0.0, 0.0]

# Calculate required control command
print("\nTest 1:")
u = active_set_algorithm(B, Wv, u_up, u_lo, v)
print("u:", u)

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
print("u:", u)

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

##########################################
# Test 4: Over-actuated system           #
##########################################
# The algorithm should apply pseudo-inverso to get an answer

v = np.array([10.0, 10.0, -1.0, 2.0])

# Control effectiveness matrix with additional actuators
B = np.array([[-20.0,  20.0, 20.0, -20.0, 20,  0],   # Roll
              [ 17.0, -17.0, 17.0, -17.0,  0, 20],   # Pitch
              [  0.7,   0.7, -0.7,  -0.7,  0,  0],   # Yaw
              [ -1.2,  -1.2, -1.2,  -1.2,  0,  0]])  # Thrust
# Control upper and lower limits
u_up = [ 1.0,  1.0,  1.0,  1.0,  1.0,  1.0]
u_lo = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
print("\nTest 4:")
print("Request:")
print(v)
u = active_set_algorithm(B, Wv, u_up, u_lo, v)
print("Command:")
print(u)

actual_out = B.dot(u)
print("Actual output:")
print(np.round(actual_out,8))

############################################
# Test 5: Over-actuated system unreachable #
############################################
# This request should trigger part of the code where
# constraints are also removed from the working set

v = np.array([60.0, 50.0, -5.0, 2.0])

# Control effectiveness matrix with additional actuators
B = np.array([[-20.0,  20.0, 20.0, -20.0, 20,  0],   # Roll
              [ 17.0, -17.0, 17.0, -17.0,  0, 20],   # Pitch
              [  0.7,   0.7, -0.7,  -0.7,  0,  0],   # Yaw
              [ -1.2,  -1.2, -1.2,  -1.2,  0,  0]])  # Thrust
# Control upper and lower limits
u_up = [ 1.0,  1.0,  1.0,  1.0,  1.0,  1.0]
u_lo = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
print("\nTest 5:")
print("Request:")
print(v)
u = active_set_algorithm(B, Wv, u_up, u_lo, v)
print("Command:")
print(u)

actual_out = B.dot(u)
print("Actual output:")
print(np.round(actual_out,8))
