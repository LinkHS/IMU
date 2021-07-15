import rosbag
import pandas as pd
import sys
import csv

"""
bag_file = sys.argv[1]
bag = rosbag.Bag(bag_file)

csv_output = []
for topic, msg, t in bag.read_messages(topics='/imu/data'):
    header = msg.header
    header_seq = header.seq 
    stamp_sec = header.stamp.secs
    stamp_nsec = header.stamp.nsecs
    csv_output.append([stamp_sec, stamp_nsec, msg.orientation.w, msg.orientation.x, msg.orientation.y,\
        msg.orientation.z, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

csv_output = pd.DataFrame(csv_output, columns = ['secs', 'nsecs',
                                      'orientation_w', 'orientation_x', 'orientation_y', 'orientation_z',
                                      'angular_velocity_x','angular_velocity_y', 'angular_velocity_z', 
                                      'linear_acceleration_x', 'linear_acceleration_y','linear_acceleration_z'])
csv_output.to_csv(sys.argv[1][:-3]+'csv')
"""                      

class IMURecorder:
    def __init__(self, csv_fn=None):
        if csv_fn is None:
            csv_fn = sys.argv[1][:-3]+'csv'

        fieldnames = ['secs', 'nsecs', 'orientation_w', 'orientation_x', 'orientation_y', 'orientation_z',
        'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z', 'linear_acceleration_x',
        'linear_acceleration_y','linear_acceleration_z']
        self.csv_file = open(csv_fn, mode='w')
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames)
        self.csv_writer.writerow({'secs': 'secs',
                                  'nsecs': 'nsecs',
                                  'orientation_w': 'orientation_w',
                                  'orientation_x': 'orientation_x',
                                  'orientation_y': 'orientation_y',
                                  'orientation_z': 'orientation_z',
                                  'angular_velocity_x': 'angular_velocity_x',
                                  'angular_velocity_y': 'angular_velocity_y',
                                  'angular_velocity_z': 'angular_velocity_z',
                                  'linear_acceleration_x': 'linear_acceleration_x',
                                  'linear_acceleration_y': 'linear_acceleration_y',
                                  'linear_acceleration_z': 'linear_acceleration_z',
                                  })

    def write_row(self, imudata):
        sq = imudata.orientation
        sw = imudata.angular_velocity
        sa = imudata.linear_acceleration

        #q = [sq.w, sq.x, sq.y, sq.z]
        #w = [sw.x, sw.y, sw.z]
        #a = [sa.x, sa.y, sa.z]
        self.csv_writer.writerow({'secs': imudata.header.stamp.secs,
                                  'nsecs': imudata.header.stamp.nsecs,
                                  'orientation_w': sq.w,
                                  'orientation_x': sq.x,
                                  'orientation_y': sq.y,
                                  'orientation_z': sq.z,
                                  'angular_velocity_x': sw.x,
                                  'angular_velocity_y': sw.y,
                                  'angular_velocity_z': sw.z,
                                  'linear_acceleration_x': sa.x,
                                  'linear_acceleration_y': sa.y,
                                  'linear_acceleration_z': sa.z,
                                  })

    def close_file(self):
        self.csv_file.close()

    def __exit__(self):
        self.csv_file.close()

if __name__ == "__main__":
    csv_fn = None
    if len(sys.argv) == 3:
        csv_fn = sys.argv[2]

    imu_recorder = IMURecorder(csv_fn)
    bag_file = sys.argv[1]
    bag = rosbag.Bag(bag_file)
    #for topic, msg, t in bag.read_messages(topics='/imu/data'):
    for topic, msg, t in bag.read_messages(topics='/IMU2/imu'):
        imu_recorder.write_row(msg)
    imu_recorder.close_file()
