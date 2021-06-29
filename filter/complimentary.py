import numpy as np
from math import sin, cos, tan, atan2, atan2, sqrt, pi

class Complimentary:
    def __init__(self, alpha):
        self.gyro_bias = [0, 0, 0]  # bias
        self.phi_hat = 0.0
        self.theta_hat = 0.0
        self.alpha = alpha

        self.idx = 0

    def imu_callback(self, q, w, a):
        if self.idx < 100:
            self.idx += 1

        ax, ay, az = np.array(a) - np.array(get_gravity_projection(q))
        p, q, r = w
        dt = 0.1 # second

        if sum(map(lambda i : i * i, (ax, ay, az))) < 0.1:
            ax, ay, az = 0, 0, 0
    
        phi_hat_acc = atan2(ay, sqrt(ax ** 2.0 + az ** 2.0))
        theta_hat_acc = atan2(-ax, sqrt(ay ** 2.0 + az ** 2.0))

        # Calculate Euler angle derivatives
        phi_dot = p + sin(self.phi_hat) * tan(self.theta_hat) * q + \
                      cos(self.phi_hat) * tan(self.theta_hat) * r
        theta_dot = cos(self.phi_hat) * q - sin(self.phi_hat) * r

        # Update complimentary filter
        self.phi_hat = (1 - self.alpha) * (self.phi_hat + dt * phi_dot) + \
                       self.alpha * phi_hat_acc
        self.theta_hat = (1 - self.alpha) * (self.theta_hat + dt * theta_dot) \
                         + self.alpha * theta_hat_acc

        # Display results
        print("Phi: ", self.phi_hat * 180.0 / pi, \
              " | Theta: ", self.theta_hat * 180.0 / pi)