import tf
import rospy
import utilities
import numpy as np
from sensor_msgs.msg import Imu
from scipy import signal
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from tf.transformations import quaternion_matrix

class IMUDataVisualiser2D:
    def __init__(self, dt=0.005, buffsize=400, label=['x','y','z'], y=['',''], ylim=[-200,200]):
        self.fig = plt.figure(figsize=(6,8))
        self.ax1 = plt.subplot(212)
        self.ln11, = self.ax1.plot([], [], 'C0', lw=1, label=label[0])
        self.ln12, = self.ax1.plot([], [], 'C1', lw=1, label=label[1])
        self.ln13, = self.ax1.plot([], [], 'C2', lw=1, label=label[2])
        plt.xlabel("t (s)")
        plt.ylabel(y[1])
        plt.legend()
        self.ax2 = plt.subplot(211, sharex=self.ax1)
        self.ln21, = self.ax2.plot([], [], 'C0', lw=1, label=label[0])
        self.ln22, = self.ax2.plot([], [], 'C1', lw=1, label=label[1])
        self.ln23, = self.ax2.plot([], [], 'C2', lw=1, label=label[2])
        plt.setp(self.ax2.get_xticklabels(), visible=False)
        plt.ylabel(y[0])
        plt.legend()
        self.x, self.y11, self.y12, self.y13 = [], [], [], []
        self.y21, self.y22, self.y23 = [], [], []
        self.index = 0
        self.dt = dt
        self.buffsize = buffsize
        self.ax1.set_xlim(0, 2*self.dt*self.buffsize)
        self.ax1.set_ylim(ylim[0], ylim[1])
        self.ax2.set_ylim(ylim[0], ylim[1])
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=5)
        
    def getorientposvel(self, q_mes, omega, acc):
        global q_calc, pos, vel, acc_space
        if self.index == 0:
            q_calc = q_mes
            pos = np.zeros(3)
            vel = np.zeros(3)
        q = utilities.calc_orient(q_ref=q_calc, omega=omega, dt=self.dt)
        q_calc = q[1]
        pos, vel, acc_space = utilities.calc_posvel(
            q_mes, init_pos=pos, init_vel=vel, acc_measured=acc, dt=self.dt, gravity_off=False)
    
        #return q_calc, acc_space, pos 

    def getposvel_filt(self, q_mes, acc):
        global pos_filt, vel_filt, acc_space_filt
        if self.index <= self.buffsize:
            pos_filt = np.zeros(3)
            vel_filt = np.zeros(3)

        pos_filt_seq, vel_filt_seq, acc_space_filt_seq = utilities.calc_posvel(
            q_mes, init_pos=pos_filt, init_vel=vel_filt, acc_measured=acc, dt=self.dt, high_pass=0.1, gravity_off=False)
    
        acc_space_filt = acc_space_filt_seq[-1]
        vel_filt = vel_filt_seq[-1]
        pos_filt = pos_filt_seq[-1]

        #return acc_space_filt, pos_filt  
    
    def imu_callback(self, q, w, a):
        raise NotImplementedError

    def update_plot(self, frame):
        self.ln11.set_data(self.x, self.y11)
        self.ln12.set_data(self.x, self.y12)
        self.ln13.set_data(self.x, self.y13)
        self.ln21.set_data(self.x, self.y21)
        self.ln22.set_data(self.x, self.y22)
        self.ln23.set_data(self.x, self.y23)
        if self.index > self.buffsize:
            self.ax1.set_xlim((self.index-self.buffsize)*self.dt, (self.index+self.buffsize)*self.dt)


class IMUEulerAngle(IMUDataVisualiser2D):
    def __init__(self, *args, **kwargs):
        super().__init__(y=['Measured','Calculated'],label=['yaw','pitch','roll'],*args, **kwargs)

    def imu_callback(self, q_mes, w, a):
        #q_calc, acc_space, pos = 
        self.getorientposvel(q_mes, w, a)
        yaw, pitch, roll = utilities.quat2seq(q_calc).T
        yaw_mes, pitch_mes, roll_mes = utilities.quat2seq(q_mes).T
        self.y11.append(yaw)
        self.y12.append(pitch)
        self.y13.append(roll)
        self.y21.append(yaw_mes)
        self.y22.append(pitch_mes)
        self.y23.append(roll_mes)
        self.x.append(self.dt*self.index)
        self.index += 1

        if self.index > self.buffsize:
            self.y11.pop(0)
            self.y12.pop(0)
            self.y13.pop(0)
            self.y21.pop(0)
            self.y22.pop(0)
            self.y23.pop(0)
            self.x.pop(0)

class IMUAcc(IMUDataVisualiser2D):
    def __init__(self, fbuffsize=10, *args, **kwargs):
        super().__init__(y=['Filtered acc','No filter acc'],ylim=[-2,2], *args, **kwargs)
        self.qbuff, self.abuff = [], []
        self.fbuffsize = fbuffsize

    def imu_callback(self, q_mes, w, a):
        #q_calc, acc_space, pos = self.getorientposvel(q_mes, w, a)
        self.qbuff.append(q_mes)
        self.abuff.append(a)
        
        self.y11.append(acc_space[0,0])
        self.y12.append(acc_space[0,1])
        self.y13.append(acc_space[0,2])

        if self.index < self.fbuffsize:
            self.y21.append(acc_space[0,0])
            self.y22.append(acc_space[0,1])
            self.y23.append(acc_space[0,2])
        else:
            #acc_space_filt, pos_filt = 
            self.getposvel_filt(self.qbuff,self.abuff)
            self.y21.append(acc_space_filt[0])
            self.y22.append(acc_space_filt[1])
            self.y23.append(acc_space_filt[2])
        
        if self.index >= self.buffsize:
            self.y11.pop(0)
            self.y12.pop(0)
            self.y13.pop(0)
            self.y21.pop(0)
            self.y22.pop(0)
            self.y23.pop(0)
            self.qbuff.pop(0)
            self.abuff.pop(0)
            self.x.pop(0)

        self.x.append(self.dt*self.index)
        self.index += 1

class IMUPos(IMUDataVisualiser2D):
    def __init__(self, fbuffsize=10, *args, **kwargs):
        super().__init__(y=['Filtered pos','No filter pos'],ylim=[-20,20], *args, **kwargs)
        #self.qbuff, self.abuff = [], []
        self.fbuffsize = fbuffsize

    def imu_callback(self, q_mes, w, a):
        #q_calc, acc_space, pos = self.getorientposvel(q_mes, w, a)
        #self.qbuff.append(q_mes)
        #self.abuff.append(a)
        
        self.y11.append(pos[0,0])
        self.y12.append(pos[0,1])
        self.y13.append(pos[0,2])

        if self.index < self.fbuffsize:
            self.y21.append(pos[0,0])
            self.y22.append(pos[0,1])
            self.y23.append(pos[0,2])
        else:
            #acc_space_filt, pos_filt = 
            #self.getposvel_filt(self.qbuff,self.abuff)
            self.y21.append(pos_filt[0])
            self.y22.append(pos_filt[1])
            self.y23.append(pos_filt[2])
        
        if self.index >= self.buffsize:
            self.y11.pop(0)
            self.y12.pop(0)
            self.y13.pop(0)
            self.y21.pop(0)
            self.y22.pop(0)
            self.y23.pop(0)
            self.x.pop(0)

        self.x.append(self.dt*self.index)
        self.index += 1

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

        self._q = (q.w, q.x, q.y, q.z)
        self._w = (w.x, w.y, w.z)
        self._a = (a.x, a.y, a.z)
        for fun in self.functions:
            fun(self._q, self._w, self._a)

if __name__ == "__main__":
    imu = IMURos()
    
    viseuler = IMUEulerAngle()
    visacc = IMUAcc()
    vispos = IMUPos()
    imu.register(viseuler.imu_callback)
    imu.register(visacc.imu_callback)
    imu.register(vispos.imu_callback)

    plt.show(block=True)