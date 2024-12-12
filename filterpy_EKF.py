import numpy as np
import pandas as pd
from filterpy.kalman import ExtendedKalmanFilter as EKF

# Parameters
dt = .001
Q = np.eye(16)
R = np.diag([.5] * 3 + [.9] * 3)
x_0 = np.zeros((16, 1))
x_0[9, 0] = 1.
P_0 = np.eye(16)

B = np.zeros((16, 1))
B[8] = dt

def get_A(x, dt):
    wN, wE, wD = [float(x[i, 0]) for i in range(13, 16)]
    A = np.eye(16)
    for i in range(6):
        A[i, i+3] = dt
    bottom = np.array(
        [[1, -dt*wN/2, -dt*wE/2, -dt*wD/2],
         [dt*wN/2, 1, dt*wD/2, -dt*wE/2],
         [dt*wE/2, -dt*wD/2, 1, dt*wN/2],
         [dt*wD/2, dt*wE/2, -dt*wN/2, 1]],
        dtype=float
    )
    A[9:13, 9:13] = bottom
    return A

def quat2matrix(q):
    q0, q1, q2, q3 = [float(q[i, 0]) for i in range(4)]
    C = np.array(
        [[1 - 2*(q2**2 + q3**2), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
         [2*(q1*q2 + q0*q3), 1 - 2*(q1**2 + q3**2), 2*(q2*q3 - q0*q1)],
         [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1**2 + q2**2)]]
    )
    return C

def get_H(x):
    C = quat2matrix(x[9:13])
    H = np.zeros((6, 16))
    H[0:3, 6:9] = C.T
    H[3:6, 13:16] = C.T
    return H

def quat_norm(q):
    norm = np.linalg.norm(q)
    if norm == 0:
        raise ValueError('Cannot normalize a zero vector')
    return q / norm

# Initialize the EKF
ekf = EKF(dim_x=16, dim_z=6)
ekf.Q = Q
ekf.R = R
ekf.x = x_0
ekf.P = P_0
ekf.B = B

# Prediction step
ekf.F = get_A(ekf.x, dt)
ekf.predict()
ekf.x[9:13] = quat_norm(ekf.x[9:13])

state_estimate = ekf.x

columns = ['pN', 'pE', 'pD', 'vN', 'vE', 'vD', 'aN', 'aE', 'aD', 'q0', 'q1', 'q2', 'q3', 'wN', 'wE', 'wD']
predictions = pd.DataFrame(state_estimate.reshape(1, -1), columns=columns)
print("predicted state:")
print(predictions)

# Update step with a measurement
measurement = np.zeros((6, 1))
measurement[2] = 9.8
def H_function(state):
    H_jacobian = get_H(state)
    corresponding_z = H_jacobian @ state
    print(f'corresponding z before gravity:\n{corresponding_z}')
    rotation_matrix = quat2matrix(state[9:13])
    print(f"quat:\n{state[9:13]}")
    print(f'rotation matrix:\n{rotation_matrix}')
    corresponding_z[0:3] += rotation_matrix @ np.array([[0], [0], [9.8]])
    print(f'corresponding z after gravity:\n{corresponding_z}')
    return corresponding_z
ekf.F = get_A(ekf.x, dt)
ekf.update(measurement, HJacobian=get_H, Hx=H_function)
print(f'residual:\n{ekf.y}')

# Resulting state
state_estimate = ekf.x

columns = ['pN', 'pE', 'pD', 'vN', 'vE', 'vD', 'aN', 'aE', 'aD', 'q0', 'q1', 'q2', 'q3', 'wN', 'wE', 'wD']
estimates = pd.DataFrame(state_estimate.reshape(1, -1), columns=columns)
print("updated state estimate:")
print(estimates)