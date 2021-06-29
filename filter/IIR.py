import numpy as np


class IIR_Filter:
    COEFFICIENTS_LOW_0_HZ = {
        'alpha': [1, -1.979133761292768, 0.979521463540373],
        'beta': [0.000086384997973502, 0.000172769995947004, 0.000086384997973502]
    }

    COEFFICIENTS_LOW_5_HZ = {
        'alpha': [1, -1.80898117793047, 0.827224480562408],
        'beta': [0.095465967120306, -0.172688631608676, 0.095465967120306]
    }

    COEFFICIENTS_HIGH_1_HZ = {
        'alpha': [1, -1.905384612118461, 0.910092542787947],
        'beta': [0.953986986993339, -1.907503180919730, 0.953986986993339]
    }

    def __init__(self, COEFFICIENTS=COEFFICIENTS_LOW_5_HZ) -> None:
        self.alpha = COEFFICIENTS['alpha']
        self.beta = COEFFICIENTS['beta']

        self.a_in = np.zeros((3, 3))
        self.a_out = np.zeros((3, 3))

    def imu_callback(self, q, w, a):
        self.a_in[:-1] = self.a_in[1:]
        self.a_in[-1] = np.array(a)
        self.a_out[:-1] = self.a_out[1:]

        alpha = self.alpha
        beta = self.beta
        a_in = self.a_in
        a_out = self.a_out
        a_out[2] = alpha[0] * (a_in[2] * beta[0]
                               + a_in[1] * beta[1]
                               + a_in[0] * beta[2]
                               - a_out[1] * alpha[1]
                               - a_out[0] * alpha[2])

        return q, w, a_out[2]
