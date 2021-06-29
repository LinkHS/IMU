import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation

from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
from tf.transformations import quaternion_multiply, quaternion_inverse, quaternion_matrix


class Arrow3D(FancyArrowPatch):
    @staticmethod
    def PRE_DEF(ax):
        return Arrow3D([0, 1], [0, 1], [0, 1], ax, mutation_scale=20,
                       lw=1, arrowstyle="-|>", color="k")

    def __init__(self, xs, ys, zs, ax, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs
        self._ax = ax

    def set_position(self, xs, ys, zs):
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, self._ax.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)


class Pose3D:
    def __init__(self, title: str = '3D Test'):
        self.fig = plt.figure()
        self.ax = p3.Axes3D(self.fig, auto_add_to_figure=False)
        self.fig.add_axes(self.ax)

        self.xyz = np.zeros((3, 1))
        self.ln, = self.ax.plot([], [], [], 'ro')
        # Make a 3D coordinate with length = 1
        self.vec = np.vstack((np.zeros((3, 3)),
                              np.array([[10, 0, 0], [0, 10, 0], [0, 0, 10]])))
        self.quiver = self.ax.quiver(*self.vec)
        self.plot_init(title)

    def plot_init(self, title=''):
        # Setting the axes properties
        self.ax.set_xlim3d([-10.0, 10.0])
        self.ax.set_xlabel('X')
        self.ax.set_ylim3d([-10.0, 10.0])
        self.ax.set_ylabel('Y')
        self.ax.set_zlim3d([-10.0, 10.0])
        self.ax.set_zlabel('Z')
        self.ax.set_title(title)

    def update_plot(self, frame):
        print('update_plot')
        xyz = self.xyz
        self.ln.set_data(xyz[0], xyz[1])
        self.ln.set_3d_properties(xyz[2])

        # if self.vec is not None:
        self.ax.cla()
        self.plot_init()
        self.quiver = self.ax.quiver(*self.vec)


class IMUVisualiser:
    def __init__(self):
        self._index = 0
        self.vis = Pose3D()
        self._ax = self.vis.ax
        self._arrow = Arrow3D.PRE_DEF(self._ax)
        # self._ani = FuncAnimation(self.vis.fig, self.draw)

        self.pa = [0, 0, 0, 0]
        self.pv = [0, 0, 0, 0]
        self.pp = [0, 0, 0, 0]
        self.a = [0, 0, 0, 0]
        self.v = [0, 0, 0, 0]
        self.p = [0, 0, 0, 0]

    def draw(self, t):
        m = quaternion_matrix(self._imu._q)
        newpoint = m[:3, :3] @ np.array([5, 5, 5])
        self._arrow.set_position(*np.stack(([0, 0, 0], newpoint), axis=1))
        self._ax.add_artist(self._arrow)

    def imu_callback(self, q, w, a):
        dt = 0.01
        self.pa = self.a
        self.pv = self.v
        self.pp = self.p
        a = [*a, 0]
        self.a = quaternion_multiply(
            quaternion_multiply(q, a), quaternion_inverse(q))
        
        print(sum(map(lambda i : i * i, self.a)))
        if sum(map(lambda i : i * i, self.a)) < 1.1:
            self.a = np.array([0., 0., 1., 0.])

        self._index += 1
        if self._index == 1:
            return

        self.v = self.pv + ((self.pa + self.a)/2 - [0, 0, 1, 0]) * 9.81 * dt
        self.p = self.pp + (self.pv + self.v) * dt
        print(self.a, self.v, self.p)


class IMUDataVisualiser:
    def __init__(self, title='xyz', buflen=5000, y_lim=(-5, 5)):
        self._fig, self._ax = plt.subplots()
        self._xyz = np.zeros((3, buflen))
        self._buflen = buflen
        self._index = 0

        self._xaxis = [0]
        self._yaxis_x, = plt.plot([], [], label='x')
        self._yaxis_y, = plt.plot([], [], label='y')
        self._yaxis_z, = plt.plot([], [], label='z')

        self._ax.set_xlim(0, self._buflen)
        self._ax.set_ylim(y_lim[0], y_lim[1])
        self.update_plot(0)
        plt.legend()
        plt.title(title)

        self._ani = FuncAnimation(self._fig, self.update_plot)

    def imu_callback(self, q, w, a):
        raise NotImplementedError

    def update_plot(self, frame_t):
        i = self._index
        self._yaxis_x.set_data(self._xaxis[:i], self._xyz[0][:i])
        self._yaxis_y.set_data(self._xaxis[:i], self._xyz[1][:i])
        self._yaxis_z.set_data(self._xaxis[:i], self._xyz[2][:i])


class IMULinearAclVisualiser(IMUDataVisualiser):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def imu_callback(self, q, w, a):
        # 将重力加速度映射到IMU坐标系
        g = [0, 0, 9.81, 0]
        # q^{-1} * g * q
        gx, gy, gz, _ = quaternion_multiply(
            quaternion_multiply(quaternion_inverse(q), g), q)

        if self._index >= self._buflen:
            self._index = 0

        self._xyz[0][self._index] = a[0] - gx
        self._xyz[1][self._index] = a[1] - gy
        self._xyz[2][self._index] = a[2] - gz

        self._index += 1
        self._xaxis.append(self._index)


class IMUAccelerationVisualiser(IMUDataVisualiser):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def imu_callback(self, q, w, a):
        if self._index >= self._buflen:
            self._index = 0

        self._xyz[0][self._index] = a[0]
        self._xyz[1][self._index] = a[1]
        self._xyz[2][self._index] = a[2]

        self._index += 1
        self._xaxis.append(self._index)


class IMUAngularVelocityVisualiser(IMUDataVisualiser):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def imu_callback(self, q, w, a):
        if self._index >= self._buflen:
            self._index = 0

        self._xyz[0][self._index] = w[0]
        self._xyz[1][self._index] = w[1]
        self._xyz[2][self._index] = w[2]

        self._index += 1
        self._xaxis.append(self._index)


if __name__ == '__main__':
    vis = Pose3D()
    plt.show(block=True)
