import utilities
import view
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.constants import g

if __name__ == '__main__':

    ## test on synethetic data -------------------------------------------------------------------------
    """
    omega = np.r_[3, 4, 10]     # [rad/s]
    duration = 2
    rate = 100
    q0 = [1, 0, 0, 0]
    out_file = 'demo_patch.mp4'
     
    ## Calculate the orientation
    dt = 1./rate
    num_rep = duration*rate
    omegas = np.tile(omega, [num_rep, 1])
    quaternion = utilities.calc_quat(omegas, q0, dt, 'bf')
    
    view.orientation(quaternion, out_file, 'Test')
    #viewer = view.Orientation_OGL(quat_in=quaternion)
    #viewer.run(looping=True, rate=20)
    """

    ## test on real IMU data ----------------------------------------------------------------------------
    
    """
    in_file = '../data/imu_stat_y_gon.txt'
    imu_data = utilities.get_data(in_file)
    stamp = imu_data.loc[:,'stamp_secs'].values + imu_data.loc[:,'stamp_nsecs'].values/1E9
    omega = imu_data.loc[:,['angular_velocity_x','angular_velocity_y','angular_velocity_z']].values
    acc_mes = imu_data.loc[:,['linear_acceleration_x','linear_acceleration_y','linear_acceleration_z']].values
    dt = stamp[1:] - stamp[:-1]
    dt = np.append(dt, dt[-1])
    q_mes = imu_data.loc[:,['orientatzion_w','orientatzion_x','orientatzion_y','orientatzion_z']].values
    """
    
    in_file = "data_CH110/data_20210625_160711.csv"
    imu_data = pd.read_csv(in_file)
    stamp = imu_data.loc[:, ['TimeStamp']].values/1E3
    dt = stamp[1:] - stamp[:-1]
    dt = np.append(dt, dt[-1])
    omega = imu_data.loc[:,['GyrX','GyrY','GyrZ']].values/180.*np.pi
    acc_mes = imu_data.loc[:,['AccX','AccY','AccZ']].values*9.7949
    q_mes = imu_data.loc[:,['Qw','Qx','Qy','Qz']].values
    
    q = utilities.calc_orient(q_ref=q_mes[0], omega=omega, dt=dt)
    
    pos, vel, acc_space = utilities.calc_posvel(q_mes, acc_measured=acc_mes, dt=dt, 
                                                high_pass=0., threshold=0, bias=False, 
                                                calibrate_acc=True, gravity_off=False)
    
    plt.figure()
    tseq = np.arange(len(q_mes))*dt.mean()
    ax1 = plt.subplot(414)
    plt.plot(tseq, q[:-1,3])
    plt.plot(tseq, q_mes[:,3])
    #plt.plot(tseq, q_corr[:-1,3])
    plt.xlabel("Time (s)")
    plt.ylabel(r"$q_z$")
    ax2 = plt.subplot(413,sharex=ax1)
    plt.plot(tseq, q[:-1,2])
    plt.plot(tseq, q_mes[:,2])
    #plt.plot(tseq, q_corr[:-1,2])
    plt.ylabel(r"$q_y$")
    plt.setp(ax2.get_xticklabels(), visible=False)
    ax3 = plt.subplot(412,sharex=ax1)
    plt.plot(tseq, q[:-1,1])
    plt.plot(tseq, q_mes[:,1])
    #plt.plot(tseq, q_corr[:-1,1])
    plt.ylabel(r"$q_x$")
    plt.setp(ax3.get_xticklabels(), visible=False)
    ax4 = plt.subplot(411,sharex=ax1)
    plt.plot(tseq, q[:-1,0], label="Calculated")
    plt.plot(tseq, q_mes[:,0], label='Measured')
    #plt.plot(tseq, q_corr[:-1,0],label='Corrected')
    plt.ylabel(r"$q_w$")
    plt.setp(ax4.get_xticklabels(), visible=False)
    plt.legend()
    
    plt.figure()
    plt.plot(tseq, acc_mes)
    plt.xlabel(r"$t\,(s)$")
    plt.ylabel("acc_mes")
    acc_mean = np.mean(np.linalg.norm(acc_mes[:5000],axis=1))
    acc_std = np.std(np.linalg.norm(acc_mes[:5000],axis=1))
    # print("Measured gravity: {0} +- {1} m/s^2".format(acc_mean, acc_std))
    
    plt.figure()
    plt.plot(tseq, acc_space)
    plt.xlabel(r"$t\,(s)$")
    plt.ylabel("acc_space")
    
    plt.figure()
    plt.plot(tseq, vel)
    plt.xlabel(r"$t\,(s)$")
    plt.ylabel("vel_space")
    
    plt.figure()
    plt.plot(tseq, pos)
    plt.xlabel(r"$t\,(s)$")
    plt.ylabel("pos_space")
    
    #view.orientation(q, out_file=None, deltaT=50)
    view.position(pos)
    #view.ani_position(pos, deltaT=1)
    plt.show()