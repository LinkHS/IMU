import rospy
import numpy as np
import visualiser
import matplotlib.pyplot as plt

from sensor_msgs.msg import Imu
from matplotlib.animation import FuncAnimation
from visualiser import IMULinearAclVisualiser, IMUAngularVelocityVisualiser, IMUVisualiser, IMUAccelerationVisualiser
from tf.transformations import quaternion_multiply, quaternion_inverse


class IMURos:
    def __init__(self) -> None:
        self.functions = []
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("imu", Imu, self.imu_callback)

    def register(self, func):
        self.functions.append(func)

    def imu_callback(self, imudata):
        q = imudata.orientation
        w = imudata.angular_velocity
        a = imudata.linear_acceleration

        self._q = (q.x, q.y, q.z, q.w)
        self._w = (w.x, w.y, w.z)
        self._a = (a.x, a.y, a.z)
        for fun in self.functions:
            fun(self._q, self._w, self._a)


class IMUNoBias:
    def __init__(self, gravity=9.81, cali_num=100) -> None:
        """
        @gravity: 重力加速度
        @cali_num: 校准的次数，大于1。例如，若等于100，就累积100次三个方向重力加速度
        """
        self.gravity = gravity
        self.idx = 1
        self.cali_num = cali_num

        self.bx = 0.0
        self.by = 0.0
        self.bz = 0.0

        self.functions = []

    def register(self, func):
        self.functions.append(func)

    def calibrate(self, q, w, a) -> None:
        if self.idx < self.cali_num:
            self.idx += 1
            gx, gy, gz = get_gravity_projection(q, self.gravity)
            self.bx += a[0] - gx
            self.by += a[1] - gy
            self.bz += a[2] - gz
            return None

        if self.idx == self.cali_num:
            self.idx += 1
            self.bx = self.bx / self.cali_num
            self.by = self.by / self.cali_num
            self.bz = self.bz / self.cali_num
            print("calibrated bias:", self.bx, self.by, self.bz)
            return None

    def imu_callback(self, q, w, a) -> None:
        if self.idx <= self.cali_num:
            self.calibrate(q, w, a)
            return None

        # 减去bias
        _a = list(a)
        _a[0] = a[0] - self.bx
        _a[1] = a[1] - self.by
        _a[2] = a[2] - self.bz
        # print(sum(map(lambda i : i * i, a))-9.81*9.81, sum(map(lambda i : i * i, _a))-9.81*9.81)
        for fun in self.functions:
            fun(q, w, tuple(_a))


def get_gravity_projection(q, gravity=9.81):
    # 将重力加速度映射到IMU坐标系
    g = [0, 0, gravity, 0]
    # q^{-1} * g * q
    gx, gy, gz, _ = quaternion_multiply(
        quaternion_multiply(quaternion_inverse(q), g), q)
    return gx, gy, gz



if __name__ == '__main__':
    imu = IMURos()
    imu_no_bias = IMUNoBias(gravity=0, cali_num=1000)
    imu.register(imu_no_bias.imu_callback)

    # raw_vis = IMURawDataVisualiser()

    # la_vis = IMULinearAclVisualiser('linear acceleration')
    # imu.register(la_vis.imu_callback)
    # la_vis_nb = IMULinearAclVisualiser('linear acceleration no bias')
    # imu_no_bias.register(la_vis_nb.imu_callback)

    acc_vis = IMUAccelerationVisualiser('Acceleration', y_lim=(-20, 20))
    imu.register(acc_vis.imu_callback)
    # acc_vis_nb = IMUAccelerationVisualiser('Acceleration no bias')
    # imu_no_bias.register(acc_vis_nb.imu_callback)

    iir_filter = IIR_Filter()
    imu.register(iir_filter.imu_callback)

    # w_vis = IMUAngularVelocityVisualiser('angular velocity', y_lim=[-10, 10])
    # imu.register(w_vis.imu_callback)

    # imu_vis = IMUVisualiser()
    # imu.register(imu_vis.imu_callback)

    # cpm_filter = Complimentary(0.1)
    # imu.register(cpm_filter.imu_callback)

    plt.show(block=True)
    # rospy.spin()
