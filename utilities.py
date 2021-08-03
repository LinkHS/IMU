import pandas as pd
import numpy as np
from scipy.constants import g
from scipy import signal
import re
import os
import sys

file_dir = os.path.dirname(__file__)
if file_dir not in sys.path:
    sys.path.insert(0, file_dir)

def get_data(filename):
    """
    Get data from IMU data file
    """
    regex_stamp = r"stamp:\s{0,}secs:\s{0,}(.*)\s{0,}nsecs:\s{0,}(.*)"
    regex_orien = r"orientation:\s{0,}x:\s{0,}(.*)\s{0,}y:\s{0,}(.*)\s{0,}z:\s{0,}(.*)\s{0,}w:\s{0,}(.*)"
    regex_ang_vel = r"angular_velocity:\s{0,}x:\s{0,}(.*)\s{0,}y:\s{0,}(.*)\s{0,}z:\s{0,}(.*)"
    regex_lin_acc  = r"linear_acceleration:\s{0,}x:\s{0,}(.*)\s{0,}y:\s{0,}(.*)\s{0,}z:\s{0,}(.*)"
    
    stamp, orien, ang_vel, lin_acc  = [], [], [], []
    
    with open(filename, 'r') as f:
        text = f.read()
        for seq in text.split('seq:'):
            stamp_search = re.search(regex_stamp, seq)
            if stamp_search:
                secs = stamp_search.group(1)
                nsecs = stamp_search.group(2)
                stamp.append((secs, nsecs))
                
            orien_search = re.search(regex_orien, seq)
            if orien_search:
                orien_x = orien_search.group(1)
                orien_y = orien_search.group(2)
                orien_z = orien_search.group(3)
                orien_w = orien_search.group(4)
                orien.append((orien_x, orien_y, orien_z, orien_w))
                
            ang_vel_search = re.search(regex_ang_vel, seq)          
            if ang_vel_search:
                ang_vel_x = ang_vel_search.group(1)
                ang_vel_y = ang_vel_search.group(2)
                ang_vel_z = ang_vel_search.group(3)
                ang_vel.append((ang_vel_x, ang_vel_y, ang_vel_z))
                
            lin_acc_search = re.search(regex_lin_acc, seq)
            if lin_acc_search:
                lin_acc_x = lin_acc_search.group(1)
                lin_acc_y = lin_acc_search.group(2)
                lin_acc_z = lin_acc_search.group(3)
                lin_acc.append((lin_acc_x, lin_acc_y, lin_acc_z))
    
    imu_data = pd.DataFrame(np.column_stack((stamp, orien, ang_vel, lin_acc)).astype(np.float64), 
                           columns = ['secs', 'nsecs',
                                      'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                                      'angular_velocity_x','angular_velocity_y', 'angular_velocity_z', 
                                      'linear_acceleration_x', 'linear_acceleration_y','linear_acceleration_z'])
        
    return imu_data

def normalize(v):
    ''' 
    Normalization of a given vector
    '''
    
    from numpy.linalg import norm
    
    # Distinguish between a vector and a matrix
    if np.array(v).ndim == 1:
        vectorFlag = True
    else:
        vectorFlag = False
        
    # The 'atleast_2d' ensures that the program works on matrices.
    v = np.double(np.atleast_2d(v)).copy()
    length = norm(v, axis=1)
    v[length!=0] = (v[length!=0].T/length[length!=0]).T
    if vectorFlag:
        v = v.ravel()
    return v

def angle(v1,v2):
    '''
    Calculate angle between two vectors
    '''

    v1 = np.array(v1)
    v2 = np.array(v2)
    
    if v1.ndim < v2.ndim:
        v1, v2 = v2, v1
    n1 = normalize(v1)
    n2 = normalize(v2)
    if v2.ndim == 1:
        angle = np.arccos(n1.dot(n2))
    else:
        angle = np.arccos(list(map(np.dot, n1, n2)))
    return angle

def qrotate(v1,v2):
    '''
    Calculate quaternion indicating the shortest rotation from one vector (v1) into another (v2).
    '''
    
    # calculate the direction
    n = normalize(np.cross(v1,v2))
    
    # make sure vectors are handled correctly
    n = np.atleast_2d(n)
    
    # handle 0-quaternions
    nanindex = np.isnan(n[:,0])
    n[nanindex,:] = 0
    
    # find the angle, and calculate the quaternion
    angle12 = angle(v1,v2)
    q = (n.T*np.sin(angle12/2.)).T
    
    # if you are working with vectors, only return a vector
    if q.shape[0]==1:
        q = q.flatten()
        
    return q

def quat2seq(quats, seq='nautical'):
    '''
    Calculates the corresponding angles of sequenctial rotations from quaternion.
    
    seq : string
        Has to be one the following:
        
        - Euler ... Rz * Rx * Rz
        - Fick ... Rz * Ry * Rx
        - nautical ... same as "Fick"
        - Helmholtz ... Ry * Rz * Rx

    '''
    
    # Ensure that it also works for a single quaternion
    quats = np.atleast_2d(quats)
    
    # If only the quaternion vector is entered, extend it to a full unit quaternion
    if quats.shape[1] == 3:
        quats = unit_q(quats)

    if seq =='Fick' or seq =='nautical':
        R_zx = 2 * (quats[:,1]*quats[:,3] - quats[:,0]*quats[:,2])
        R_yx = 2 * (quats[:,1]*quats[:,2] + quats[:,0]*quats[:,3])
        R_zy = 2 * (quats[:,2]*quats[:,3] + quats[:,0]*quats[:,1])
        
        phi  = -np.arcsin(R_zx)
        theta = np.arcsin(R_yx / np.cos(phi))
        psi   = np.arcsin(R_zy / np.cos(phi))
        
        sequence = np.column_stack((theta, phi, psi))
    
    elif seq == 'Helmholtz':
        R_yx = 2 * (quats[:,1]*quats[:,2] + quats[:,0]*quats[:,3])
        R_zx = 2 * (quats[:,1]*quats[:,3] - quats[:,0]*quats[:,2])
        R_yz = 2 * (quats[:,2]*quats[:,3] - quats[:,0]*quats[:,1])
        
        theta = np.arcsin(R_yx)
        phi  = -np.arcsin(R_zx / np.cos(theta))
        psi  = -np.arcsin(R_yz / np.cos(theta))
        
        sequence = np.column_stack((phi, theta, psi))
        
    elif seq == 'Euler':
        Rs = quat2rot(quats).reshape((-1,3,3))
        
        beta = np.arccos(Rs[:,2,2])
        
        # special handling for (beta == 0)
        bz = beta == 0  
        
        # there the gamma-values are set to zero, since alpha/gamma is degenerated
        alpha = np.nan * np.ones_like(beta)
        gamma = np.nan * np.ones_like(beta)
        
        alpha[bz] = np.arcsin(Rs[bz,1,0])
        gamma[bz] = 0
        
        alpha[~bz] = np.arctan2(Rs[~bz,0,2], Rs[~bz,1,2])
        gamma[~bz] = np.arctan2(Rs[~bz,2,0], Rs[~bz,2,1])
        
        sequence = np.column_stack((alpha, beta, gamma))
    else:
        raise ValueError('Input parameter {0} not known'.format(seq))
    
    return np.rad2deg(sequence)

def rot2quat(rMat):
    """
    Converts a rotation matrix to the corresponding quaternion.
    """
    
    if rMat.shape == (3,3) or rMat.shape == (9,):
        rMat=np.atleast_2d(rMat.ravel()).T
    else:
        rMat = rMat.T
    q = np.zeros((4, rMat.shape[1]))
    
    R11 = rMat[0]
    R12 = rMat[1]
    R13 = rMat[2]
    R21 = rMat[3]
    R22 = rMat[4]
    R23 = rMat[5]
    R31 = rMat[6]
    R32 = rMat[7]
    R33 = rMat[8]
    
    # Catch small numerical inaccuracies, but produce an error for larger problems
    epsilon = 1e-10
    if np.min(np.vstack((1+R11-R22-R33, 1-R11+R22-R33, 1-R11-R22+R33))) < -epsilon:
        raise ValueError('Problems with defintion of rotation matrices')
    
    q[1] = 0.5 * np.copysign(np.sqrt(np.abs(1+R11-R22-R33)), R32-R23)
    q[2] = 0.5 * np.copysign(np.sqrt(np.abs(1-R11+R22-R33)), R13-R31)
    q[3] = 0.5 * np.copysign(np.sqrt(np.abs(1-R11-R22+R33)), R21-R12)
    q[0] = np.sqrt(1-(q[1]**2+q[2]**2+q[3]**2))
    """
    q[0] = np.sqrt(1+R11+R22+R33)/2.
    q[1] = (R32-R23)/(4*q[0])
    q[2] = (R13-R31)/(4*q[0])
    q[3] = (R21-R12)/(4*q[0])
    """
    
    return q.T

def q_vector(inQuat):
    '''
    Extract the quaternion vector from a full quaternion.
    '''
    
    inQuat = np.atleast_2d(inQuat)
    if inQuat.shape[1] == 4:
        vect = inQuat[:,1:]
    else:
        vect = inQuat
    if np.min(vect.shape)==1:
        vect = vect.ravel()
    return vect

def q_scalar(inQuat):
    '''
    Extract the quaternion scalar from a full quaternion. 
    '''
    
    inQuat = np.atleast_2d(inQuat)
    if inQuat.shape[1] == 4:
        scalar = inQuat[:,0]
    else:
        scalar = np.sqrt(1-np.linalg.norm(inQuat, axis=1))
    if np.min(scalar.shape)==1:
        scalar = scalar.ravel()
    return scalar

def quat2rot(quat):
    ''' 
    Calculate the rotation matrix corresponding to the quaternion.     
    '''
    q = unit_q(quat).T
        
    R = np.zeros((9, q.shape[1]))
    R[0] = q[0]**2 + q[1]**2 - q[2]**2 - q[3]**2
    R[1] = 2*(q[1]*q[2] - q[0]*q[3])
    R[2] = 2*(q[1]*q[3] + q[0]*q[2])
    R[3] = 2*(q[1]*q[2] + q[0]*q[3])
    R[4] = q[0]**2 - q[1]**2 + q[2]**2 - q[3]**2
    R[5] = 2*(q[2]*q[3] - q[0]*q[1])
    R[6] = 2*(q[1]*q[3] - q[0]*q[2])
    R[7] = 2*(q[2]*q[3] + q[0]*q[1])
    R[8] = q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2
        
    if R.shape[1] == 1:
        return np.reshape(R, (3,3))
    else:
        return R.T
    
def q_inv(q):
    ''' 
    Quaternion inversion 
    '''
    
    q = np.atleast_2d(q)
    if q.shape[1]==3:
        return -q
    else:
        qLength = np.sum(q**2, 1)
        qConj = q * np.r_[1, -1,-1,-1]
        return (qConj.T / qLength).T

def q_mult(p,q):
    '''
    Quaternion multiplication.
    '''

    flag3D = False
    p = np.atleast_2d(p)
    q = np.atleast_2d(q)
    if p.shape[1]==3 & q.shape[1]==3:
        flag3D = True

    if len(p) != len(q):
        assert (len(p)==1 or len(q)==1), \
            'Both arguments in the quaternion multiplication must have the same number of rows, unless one has only one row.'

    p = unit_q(p).T
    q = unit_q(q).T
    
    if np.prod(np.shape(p)) > np.prod(np.shape(q)):
        r=np.zeros(np.shape(p))
    else:
        r=np.zeros(np.shape(q))

    r[0] = p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3]
    r[1] = p[1]*q[0] + p[0]*q[1] + p[2]*q[3] - p[3]*q[2]
    r[2] = p[2]*q[0] + p[0]*q[2] + p[3]*q[1] - p[1]*q[3]
    r[3] = p[3]*q[0] + p[0]*q[3] + p[1]*q[2] - p[2]*q[1]

    if flag3D:
        # for rotations > 180 deg
        r[:,r[0]<0] = -r[:,r[0]<0]
        r = r[1:]

    r = r.T
    return r

def unit_q(inData):
    ''' 
    Turn a quaternion vector into a unit quaternion.
    '''
    inData = np.atleast_2d(inData)
    (m,n) = inData.shape
    if (n!=3)&(n!=4):
        raise ValueError('Quaternion must have 3 or 4 columns')
    if n == 3:
        qLength = 1-np.sum(inData**2,1)
        numLimit = 1e-12
        # Check for numerical problems
        if np.min(qLength) < -numLimit:
            raise ValueError('Quaternion is too long!')
        else:
            # Correct for numerical problems
            qLength[qLength<0] = 0
        outData = np.hstack((np.c_[np.sqrt(qLength)], inData))
        
    else:
        outData = inData
        
    return outData

def rotate_vector(vector, q):
    '''
    Rotates a vector according to the given quaternions.
    '''

    vector = np.atleast_2d(vector)
    qvector = np.hstack((np.zeros((vector.shape[0],1)), vector))
    vRotated = q_mult(q, q_mult(qvector, q_inv(q)))
    vRotated = vRotated[:,1:]

    if min(vRotated.shape)==1:
        vRotated = vRotated.ravel()

    return vRotated

def calc_quat(omega, q0, dt, CStype):
    '''
    Take an angular velocity (in rad/s), and convert it into the
    corresponding orientation quaternion.

    Parameters
    ----------
    omega : angular velocity [rad/s].
    q0 : vector-part of quaternion (!!)
    dt : time steps (in [s])
    CStype: coordinate_system, space-fixed ("sf") or body_fixed ("bf")
    '''
    
    omega_2d = np.atleast_2d(omega).copy()
    if isinstance(dt, int) or isinstance(dt, float):
        dt = np.ones(len(omega_2d))*dt
    
    # The following is (approximately) the quaternion-equivalent of the trapezoidal integration (cumtrapz)
    if omega_2d.shape[1]>1:
        omega_2d[:-1] = 0.5*(omega_2d[:-1] + omega_2d[1:])

    omega_t = np.sqrt(np.sum(omega_2d**2, 1))
    omega_nonZero = omega_t>0

    # initialize the quaternion
    q_delta = np.zeros(omega_2d.shape)
    q_pos = np.zeros((len(omega_2d)+1,4))
    q_pos[0,:] = unit_q(q0)

    # magnitude of position steps
    dq_total = np.sin(omega_t[omega_nonZero]*dt[omega_nonZero]/2)

    q_delta[omega_nonZero,:] = omega_2d[omega_nonZero,:] * np.tile(dq_total/omega_t[omega_nonZero], (3,1)).T

    for ii in range(len(omega_2d)):
        q1 = unit_q(q_delta[ii,:])
        q2 = q_pos[ii,:]
        if CStype == 'sf':            
            qm = q_mult(q1,q2)
        elif CStype == 'bf':
            qm = q_mult(q2,q1)
        else:
            print('Wrong coordinate system!')
        q_pos[ii+1,:] = qm

    return q_pos

class Mahony:
    
    def __init__(self, rate=200.0, Kp=1.0, Ki=0, Quaternion=np.r_[1,0,0,0], nav='ENU'):
        
        self.rate = rate
        self.SamplePeriod = 1/self.rate
        self.Kp = Kp
        self.Ki = Ki
        self.Quaternion = np.asarray(Quaternion, dtype='float64')
        self._eInt = [0, 0, 0]  # integral error
        self.nav = nav

    def Update(self, Gyroscope, Accelerometer, Magnetometer=None):
        
        q = self.Quaternion
        Accelerometer = normalize(Accelerometer)
        
        v = np.array([
            2*(q[1]*q[3] - q[0]*q[2]),
            2*(q[0]*q[1] + q[2]*q[3]),
            q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2])
        e = np.cross(Accelerometer, v)
        
        if Magnetometer is not None:
            Magnetometer = normalize(Magnetometer)
            h = rotate_vector(Magnetometer, q)
            if self.nav == 'NWU':
                b = np.hstack((0, np.sqrt(h[0]**2+h[1]**2), 0, h[2]))
                w = np.array([
                    2*b[1]*(0.5 - q[2]**2 - q[3]**2) + 2*b[3]*(q[1]*q[3] - q[0]*q[2]),
                    2*b[1]*(q[1]*q[2] - q[0]*q[3]) + 2*b[3]*(q[0]*q[1] + q[2]*q[3]),
                    2*b[1]*(q[0]*q[2] + q[1]*q[3]) + 2*b[3]*(0.5 - q[1]**2 - q[2]**2)]) 
            
            elif self.nav == 'ENU':
                b = np.hstack((0, 0, np.sqrt(h[0]**2+h[1]**2), h[2]))
                w = np.array([
                    2*b[2]*(q[1]*q[2] + q[0]*q[3]) + 2*b[3]*(q[1]*q[3] - q[0]*q[2]),
                    2*b[2]*(0.5 - q[1]**2 - q[3]**2) + 2*b[3]*(q[0]*q[1] + q[2]*q[3]),
                    2*b[2]*(q[2]*q[3] - q[0]*q[1]) + 2*b[3]*(0.5 - q[1]**2 - q[2]**2)]) 

            e += np.cross(Magnetometer, w) 

        if self.Ki > 0:
            self._eInt += e * self.SamplePeriod  
        else:
            self._eInt = np.array([0, 0, 0], dtype=np.float)

        Gyroscope += self.Kp * e + self.Ki * self._eInt;            

        qDot = 0.5 * q_mult(q, np.hstack([0, Gyroscope])).flatten()
        q += qDot * self.SamplePeriod

        self.Quaternion = normalize(q)

class Madgwick:

    def __init__(self, rate=200.0, Beta=1.0, Quaternion=np.r_[1,0,0,0], nav='ENU'):
        
        self.rate = rate
        self.SamplePeriod = 1/self.rate
        self.Beta = Beta
        self.Quaternion = np.asarray(Quaternion, dtype='float64')
        self.nav = nav

    def Update(self, Gyroscope, Accelerometer, Magnetometer):

        q = self.Quaternion
        Accelerometer = normalize(Accelerometer)
        Magnetometer = normalize(Magnetometer)
        
        h = rotate_vector(Magnetometer, q)
        
        if self.nav == 'NWU':
            b = np.hstack((0, np.sqrt(h[0]**2+h[1]**2), 0, h[2]))

            F = [2*(q[1]*q[3] - q[0]*q[2])   - Accelerometer[0],
                 2*(q[0]*q[1] + q[2]*q[3])   - Accelerometer[1],
                 2*(0.5 - q[1]**2 - q[2]**2) - Accelerometer[2],
                 2*b[1]*(0.5 - q[2]**2 - q[3]**2) + 2*b[3]*(q[1]*q[3] - q[0]*q[2])   - Magnetometer[0],
                 2*b[1]*(q[1]*q[2] - q[0]*q[3])   + 2*b[3]*(q[0]*q[1] + q[2]*q[3])   - Magnetometer[1],
                 2*b[1]*(q[0]*q[2] + q[1]*q[3])   + 2*b[3]*(0.5 - q[1]**2 - q[2]**2) - Magnetometer[2]]

            J = np.array([
                [-2*q[2],                 	2*q[3],                    -2*q[0],                         2*q[1]],
                [ 2*q[1],                 	2*q[0],                	    2*q[3],                         2*q[2]],
                [0,                        -4*q[1],                    -4*q[2],                         0],
                [-2*b[3]*q[2],              2*b[3]*q[3],               -4*b[1]*q[2]-2*b[3]*q[0],       -4*b[1]*q[3]+2*b[3]*q[1]],
                [-2*b[1]*q[3]+2*b[3]*q[1],	2*b[1]*q[2]+2*b[3]*q[0],    2*b[1]*q[1]+2*b[3]*q[3],       -2*b[1]*q[0]+2*b[3]*q[2]],
                [ 2*b[1]*q[2],              2*b[1]*q[3]-4*b[3]*q[1],    2*b[1]*q[0]-4*b[3]*q[2],        2*b[1]*q[1]]])
        
        elif self.nav == 'ENU': 
            b = np.hstack((0, 0, np.sqrt(h[0]**2+h[1]**2), h[2]))

            F = [2*(q[1]*q[3] - q[0]*q[2])   - Accelerometer[0],
                 2*(q[0]*q[1] + q[2]*q[3])   - Accelerometer[1],
                 2*(0.5 - q[1]**2 - q[2]**2) - Accelerometer[2],
                 2*b[2]*(q[1]*q[2] + q[0]*q[3])   + 2*b[3]*(q[1]*q[3] - q[0]*q[2])   - Magnetometer[0],
                 2*b[2]*(0.5 - q[1]**2 - q[3]**2) + 2*b[3]*(q[0]*q[1] + q[2]*q[3])   - Magnetometer[1],
                 2*b[2]*(q[2]*q[3] - q[0]*q[1])   + 2*b[3]*(0.5 - q[1]**2 - q[2]**2) - Magnetometer[2]]

            J = np.array([
                [-2*q[2],                 	2*q[3],                    -2*q[0],                         2*q[1]],
                [ 2*q[1],                 	2*q[0],                	    2*q[3],                         2*q[2]],
                [0,                        -4*q[1],                    -4*q[2],                         0],
                [ 2*b[2]*q[3]-2*b[3]*q[2],  2*b[2]*q[2]+2*b[3]*q[3],    2*b[2]*q[1]-2*b[3]*q[0],        2*b[2]*q[0]+2*b[3]*q[1]],
                [ 2*b[3]*q[1],	           -4*b[2]*q[1]+2*b[3]*q[0],                2*b[3]*q[3],       -4*b[2]*q[3]+2*b[3]*q[2]],
                [ -2*b[2]*q[1],            -2*b[2]*q[0]-4*b[3]*q[1],    2*b[2]*q[3]-4*b[3]*q[2],        2*b[2]*q[2]]])
        else:
            raise SystemError("Unsupported navigation reference system!")
        
        step = J.T.dot(F)
        step = normalize(step)

        qDot = 0.5 * q_mult(q, np.hstack([0, Gyroscope])) - self.Beta * step

        q = q + qDot * self.SamplePeriod
        self.Quaternion = normalize(q).flatten()


def tupleset(t, i, value):
    l = list(t)
    l[i] = value
    return tuple(l)

def cumtrapz(y, x=None, dx=1.0, axis=-1, initial=None):
    """
    Cumulatively integrate y(x) using the composite trapezoidal rule.
    Parameters
    ----------
    y : array_like
        Values to integrate.
    x : array_like, optional
        The coordinate to integrate along.  If None (default), use spacing `dx`
        between consecutive elements in `y`.
    dx : int, optional
        Spacing between elements of `y`.  Only used if `x` is None.
    axis : int, optional
        Specifies the axis to cumulate.  Default is -1 (last axis).
    initial : scalar, optional
        If given, uses this value as the first value in the returned result.
        Typically this value should be 0.  Default is None, which means no
        value at ``x[0]`` is returned and `res` has one element less than `y`
        along the axis of integration.
    Returns
    -------
    res : ndarray
        The result of cumulative integration of `y` along `axis`.
        If `initial` is None, the shape is such that the axis of integration
        has one less value than `y`.  If `initial` is given, the shape is equal
        to that of `y`.
    """
    y = np.asarray(y)
    if x is None:
        d = dx
    else:
        x = np.asarray(x)
        if x.ndim == 1:
            d = np.diff(x)
            # reshape to correct shape
            shape = [1] * y.ndim
            shape[axis] = -1
            d = d.reshape(shape)
        elif len(x.shape) != len(y.shape):
            raise ValueError("If given, shape of x must be 1-d or the "
                    "same as y.")
        else:
            d = np.diff(x, axis=axis)

        if d.shape[axis] != y.shape[axis] - 1:
            raise ValueError("If given, length of x along axis must be the "
                             "same as y.")

    nd = len(y.shape)
    slice1 = tupleset((slice(None),)*nd, axis, slice(1, None))
    slice2 = tupleset((slice(None),)*nd, axis, slice(None, -1))
    res = np.add.accumulate(d * (y[slice1] + y[slice2]) / 2.0, axis)

    if initial is not None:
        if not np.isscalar(initial):
            raise ValueError("`initial` parameter should be a scalar.")

        shape = list(res.shape)
        shape[axis] = 1
        res = np.concatenate([np.ones(shape, dtype=res.dtype) * initial, res],
                             axis=axis)

    return res

def calc_orient(q_ref=np.r_[1,0,0,0],
               omega=np.zeros((5,3)),
               dt=np.ones(5)*0.005,
               method='Integrate', # "Mahony", "Madgwick"
               acc=np.column_stack((np.zeros((5,2)), 9.7949*np.ones(5))),
               mag=None,
               nav='ENU'):
    ''' 
    Reconstruct orientation from angular velocity.
    '''

    omega = np.atleast_2d(omega)
    acc = np.atleast_2d(acc)
    if method == 'Integrate':
        q = calc_quat(omega, q_ref, dt, 'bf')
        #q = q_mult(q_inv(q[0]), q)

    elif method == 'Mahony':
        AHRS = Mahony(rate=1/dt.mean(), Kp=0.4, Quaternion=q_ref, nav=nav)
        q = np.zeros((len(omega)+1, 4))
        q[0] = q_ref
        for tt in range(len(omega)):
            if mag is None:
                AHRS.Update(omega[tt], acc[tt])
            else:
                AHRS.Update(omega[tt], acc[tt], mag[tt])
            q[tt+1] = AHRS.Quaternion
            
    elif method == 'Madgwick':
        AHRS = Madgwick(rate=1/dt.mean(), Beta=0.5, Quaternion=q_ref, nav=nav)
        q = np.zeros((len(omega)+1, 4))
        q[0] = q_ref
        for tt in range(len(omega)):
            AHRS.Update(omega[tt], acc[tt], mag[tt])
            q[tt+1] = AHRS.Quaternion
    else:
        raise SystemError('Unknown orientation type: {0}'.format(method))
    q[q[:,0]<0,:] = -q[q[:,0]<0,:]
    return q

def calib_acc(acc):
    # theta_pr = [0.001768725422353, 0.007761852978931, -0.007242978857302, 
    #             1.013061190182213, 0.994198454621163, 0.990576049692495,
    #             -0.156737625071990, -0.028140711397211, -0.011445790962552]
    # theta_pr = [0.001301486931713, 0.007784874049646, -0.007313710436288,
    #             1.013909312525567, 0.994272191256321, 0.990420800583987,
    #             -0.164607908168934, -0.029163656520914, -0.012234801349046]
    theta_pr = [-0.002070500810200, 0.008528112599103, -0.010975473501493,
                0.998328157672050, 0.995815096816491, 0.991661580176438, 
                -0.009248369054021, 0.015077977918707,-0.019095604088764]
    misalignmentMatrix = np.asarray([1, -theta_pr[0], theta_pr[1], 0, 1, -theta_pr[2], 0, 0, 1]).reshape(3,3)
    scalingMatrix = np.diag([theta_pr[3], theta_pr[4], theta_pr[5]])
    biasVector = np.asarray([[theta_pr[6]], [theta_pr[7]], [theta_pr[8]]])
    acc_cab = np.dot(np.dot(misalignmentMatrix,scalingMatrix), acc.T) - biasVector
    
    return acc_cab.T

def qs_check(acc, Tinit, freq):
    total_sample = len(acc)
    var_3D = np.sum(np.var(acc[:int(freq*Tinit)], axis=0)**2)
    w_d = 101
    half_w_d = int(w_d/2)
    
    normal_acc = np.zeros((total_sample, 3))
    for i in range(half_w_d, total_sample-half_w_d):
        normal_acc[i,:] = np.var(acc[i-half_w_d:i+half_w_d], axis=0)
    s_square = np.sum(normal_acc**2, axis=1)
    #plt.plot(s_square)
    s_filter = np.asarray([False]*total_sample)
    s_filter[half_w_d:total_sample-half_w_d] = np.where(s_square[half_w_d:total_sample-half_w_d]<=var_3D, True, False)
    return s_filter
    
    #plt.show()

def calc_posvel(quat=np.r_[1,0,0,0],
               init_pos=np.zeros(3),
               init_vel=np.zeros(3),
               acc_measured=np.column_stack((np.zeros((5,2)), 9.7949*np.ones(5))),
               dt=np.ones(5)*0.005,
               high_pass = None,
               low_pass = None,
               threshold=None,
               bias=False,
               calibrate_acc=False,
               gravity_off=False):
    ''' 
    Reconstruct position from linear acceleration.
    Assumes a start in a stationary position!!!
    '''

    acc_measured = np.atleast_2d(acc_measured)
    #print("before: g={0} m/s^2".format(np.mean(np.linalg.norm(acc_measured, axis=1))))
    if calibrate_acc:
        acc_measured = calib_acc(acc_measured)
    #print("after: g={0} m/s^2".format(np.mean(np.linalg.norm(acc_measured, axis=1))))
    if isinstance(dt, int) or isinstance(dt, float):
        dt = np.ones(len(acc_measured))*dt
    
    # g_v = np.r_[0, 0, g]
    g_v = np.r_[0,0,9.7949]
    
    if gravity_off:
        acc_sensor = acc_measured
    else:
        acc_sensor = acc_measured - rotate_vector(g_v, q_inv(quat))
    
    if bias:
        fixed_filter = qs_check(acc_measured, 1, freq=1./dt.mean())
        acc_bias = np.mean(acc_sensor[fixed_filter], axis=0)   
        print("Acc Bias: ", acc_bias)
        acc_sensor = acc_sensor - acc_bias
        
    acc_space = rotate_vector(acc_sensor, quat)
    
    if high_pass:
        bh, ah = signal.butter(1, high_pass*2*dt.mean(), 'highpass')
        acc_space = signal.filtfilt(bh, ah, acc_space.T).T
    if low_pass:
        bl, al = signal.butter(1, low_pass*2*dt.mean(), 'lowpass')
        acc_space = signal.filtfilt(bl, al, acc_space.T).T
        
    if threshold:
        acc_space[np.sum(acc_space**2,axis=1)<threshold**2] = 0.
    
    # acc_sensor = rotate_vector(acc_measured, quat)
    # if gravity_off:
    #     acc_space = acc_sensor
    # else:
    #     acc_space = acc_sensor - g_v
    
    acc_space = np.atleast_2d(acc_space)

    vel = np.nan*np.ones_like(acc_space)
    pos = np.nan*np.ones_like(acc_space)
    if acc_space.shape[0] == 1:
        vel[0,:] = init_vel + acc_space[0]*dt
        pos[0,:] = init_pos + init_vel*dt + 0.5*acc_space[0]*dt**2
        return (pos,vel,acc_space)

    for ii in range(acc_space.shape[1]):
        vel[:,ii] = cumtrapz(acc_space[:,ii], dx=dt[:-1], initial=init_vel[ii])
        if high_pass:
            vel[:,ii] = signal.filtfilt(bh, ah, vel[:,ii])
        if low_pass:
            vel[:,ii] = signal.filtfilt(bl, al, vel[:,ii])
        pos[:,ii] = cumtrapz(vel[:,ii], dx=dt[:-1], initial=init_pos[ii])

    return (pos, vel, acc_space)