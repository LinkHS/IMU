import utilities
import numpy as np
import sys
import os

if __name__ == '__main__':
    in_file = sys.argv[1]
    imu_data = utilities.get_data(in_file)
    #fname = os.path.split(in_file)[0] + '/' + os.path.split(in_file)[1].split('.')[0] + '.csv'
    #print(fname)
    #imu_data.to_csv(fname)
    
    cab = imu_data.loc[:,['linear_acceleration_x','linear_acceleration_y','linear_acceleration_z',
                          'angular_velocity_x','angular_velocity_y','angular_velocity_z']].values
    nrows = len(imu_data)
    cab_output = np.column_stack([np.arange(nrows)*0.005, cab, np.zeros((nrows, 3))])
    fnametxt = os.path.split(in_file)[0] + '/' + os.path.split(in_file)[1].split('.')[0]
    np.savetxt(fnametxt+"Dalpha.txt", cab_output[:,:4])
    np.savetxt(fnametxt+"Domega.txt", cab_output[:,[0,4,5,6]])
    