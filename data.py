import numpy as np

class Fake3D:
    def __init__(self, fps=10):
        self.x = 0
        self.y = 0
        self.z = 0
        # self.rx = 0
        # self.ry = 0
        # self.rz = 0
        # self.ax = 0
        # self.ay = 0
        # self.az = 0

    def __iter__(self):
        rx, ry, rz = np.random.rand(3)
        return 
        
def IMUupdate(gx, gy, gz, ax, ay, az, q0_l, q1_l, q2_l, q3_l):
    """
        @q0_l, q1_l, q2_l, q3_l, 上一次结果
    """
    # norm = 0.0
    # vx, vy, vz
    # ex, ey, ez

    # 把加速度计的三维向量转成单位向量
    norm = 1 / math.sqrt(ax*ax + ay*ay + az*az)
    if (norm == 0):
        return # handle NaN
    ax = ax * norm
    ay = ay * norm
    az = az * norm

    ## 估计重力加速度方向在飞行器坐标系中的表示，为四元数表示的旋转矩阵的第三行
    vx = 2*(q1*q3 - q0*q2)
    vy = 2*(q0*q1 + q2*q3)
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3


if __name__ == "__main__":
    IMUupdate(0, 0, 0, 0, 0, 0, 1, 0, 0, 0)