""" In this project you will recursively estimate the position of a vehicle along a trajectory using available
measurements and a motion model.

The vehicle is equipped with a very simple type of LIDAR sensor, which returns range and bearing measurements
corresponding to individual landmarks in the environment. The global positions of the landmarks are assumed to be
known beforehand. We will also assume known data association, that is, which measurements belong to which landmark."""

import pickle
import numpy as np
import matplotlib.pyplot as plt
import math

#Unpack the Data
with open('data/data.pickle', 'rb') as f:
    data = pickle.load(f)

t = data['t']  # timestamps [s]

x_init  = data['x_init'] # initial x position [m]
y_init  = data['y_init'] # initial y position [m]
th_init = data['th_init'] # initial theta position [rad]

# input signal
v  = data['v']  # translational velocity input [m/s]
om = data['om']  # rotational velocity input [rad/s]

# bearing and range measurements, LIDAR constants
b = data['b']  # bearing to each landmarks center in the frame attached to the laser [rad]
r = data['r']  # range measurements [m]
l = data['l']  # x,y positions of landmarks [m]
d = data['d']  # distance between robot center and laser rangefinder [m]

#Initializing Parameters (Fine tuning)
v_var = 0.004  # translation velocity variance
om_var = 0.008  # rotational velocity variance
r_var = 0.001  # range measurements variance
b_var = 0.0005  # bearing measurement variance

Q_km = np.diag([v_var, om_var]) # input noise covariance
cov_y = np.diag([r_var, b_var])  # measurement noise covariance

x_est = np.zeros([len(v), 3])  # estimated states, x, y, and theta
P_est = np.zeros([len(v), 3, 3])  # state covariance matrices

x_est[0] = np.array([x_init, y_init, th_init]) # initial state
P_est[0] = np.diag([1, 1, 0.1]) # initial state covariance


# Wraps angle to (-pi,pi] range
def wraptopi(x):
    if x > np.pi:
        x = x - (np.floor(x / (2 * np.pi)) + 1) * 2 * np.pi
    elif x < -np.pi:
        x = x + (np.floor(x / (-2 * np.pi)) + 1) * 2 * np.pi
    return x


# Correction step
from numpy.linalg import inv

def measurement_update(lk, rk, bk, P_check, x_check):
    # 1. Compute measurement Jacobian
    x_l = lk[0];
    y_l = lk[1]
    x_k = x_check[0];
    y_k = x_check[1];
    theta_k = wraptopi(x_check[2])
    A = x_l - x_k - d[0] * np.cos(theta_k)
    B = y_l - y_k - d[0] * np.sin(theta_k)

    H_k = np.array([[-np.sqrt(1 / (A ** 2 + B ** 2)) * A, -np.sqrt(1 / (A ** 2 + B ** 2)) * B,
                     np.sqrt(1 / (A ** 2 + B ** 2)) * (A * d[0] * np.sin(theta_k) - B * d[0] * np.cos(theta_k))],
                    [B / (A ** 2 + B ** 2), -A / (A ** 2 + B ** 2),
                     -1 - d[0] * (np.cos(theta_k) * A + np.sin(theta_k) * B)]])
    M_k = np.identity(2)
    # 2. Compute Kalman Gain
    K_k = P_check.dot(H_k.T.dot(inv(H_k.dot(P_check.dot(H_k.T)) + M_k.dot(cov_y.dot(M_k.T)))))
    # 3. Correct predicted state (remember to wrap the angles to [-pi,pi])
    y_check = np.array([np.sqrt(A ** 2 + B ** 2), wraptopi(math.atan2(B, A) - theta_k)])
    tmp = x_check + K_k.dot(np.array([rk, bk]) - y_check)

    x_check[0] = tmp[0]
    x_check[1] = tmp[1]
    x_check[2] = wraptopi(tmp[2])
    # 4. Correct covariance
    P_check = (np.identity(3) - K_k.dot(H_k)).dot(P_check)
    return x_check, P_check

#Prediction Step
#### 5. Main Filter Loop #######################################################################
for k in range(1, len(t)):  # start at 1 because we've set the initial prediciton

    delta_t = t[k] - t[k - 1]  # time step (difference between timestamps)

    # 1. Update state with odometry readings (remember to wrap the angles to [-pi,pi])
    tmp = delta_t*np.array([[np.cos(x_est[k-1,2]),0],[np.sin(x_est[k-1,2]),0],[0,1]])
    x_check = x_est[k-1,:] + tmp.dot(np.array([v[k],om[k]]))
    tmp2 = wraptopi(x_check[2])
    x_check[2] = tmp2
    # 2. Motion model jacobian with respect to last state
    F_km = np.array([[1, 0, -delta_t*v[k-1]*np.sin(x_est[k-1,2])],[0,1,delta_t*v[k-1]*np.cos(x_est[k-1,2])],[0, 0, 1]])

    # 3. Motion model jacobian with respect to noise
    L_km = tmp

    # 4. Propagate uncertainty
    P_check = F_km.dot(P_est[k-1].dot(F_km.T))+L_km.dot(Q_km.dot(L_km.T))
    # 5. Update state estimate using available landmark measurements
    for i in range(len(r[k])):
        x_check, P_check = measurement_update(l[i], r[k, i], b[k, i], P_check, x_check)

    # Set final state predictions for timestep
    x_est[k, 0] = x_check[0]
    x_est[k, 1] = x_check[1]
    x_est[k, 2] = x_check[2]
    P_est[k, :, :] = P_check

#Plot the resulting state estimates
e_fig = plt.figure()
ax = e_fig.add_subplot(111)
ax.plot(x_est[:, 0], x_est[:, 1])
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_title('Estimated trajectory')
plt.show()

e_fig = plt.figure()
ax = e_fig.add_subplot(111)
ax.plot(t[:], x_est[:, 2])
ax.set_xlabel('Time [s]')
ax.set_ylabel('theta [rad]')
ax.set_title('Estimated trajectory')
plt.show()

#Produce solution files
with open('submission.pkl', 'wb') as f:
    pickle.dump(x_est, f, pickle.HIGHEST_PROTOCOL)
