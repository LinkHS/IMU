import csv
import datetime
import time
import rospy
import sys

from sensor_msgs.msg import Imu


class IMURecorder:
    def __init__(self, csv_fn=None):
        if csv_fn is None:
            ctime = datetime.datetime.fromtimestamp(time.time())
            csv_fn = ctime.strftime('%Y%m%d_%H%M%S') + '.csv'

        fieldnames = ['seq', 'orientation_w', 'orientation_x', 'orientation_y', 'orientation_z',
        'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z', 'linear_acceleration_x',
        'linear_acceleration_y','linear_acceleration_z','secs', 'nsecs']
        self.csv_file = open(csv_fn, mode='w')
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames)
        self.csv_writer.writerow({'seq': 'seq',
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
                                  'secs': 'secs',
                                  'nsecs': 'nsecs'})

        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("imu", Imu, self.imu_callback)

    def imu_callback(self, imudata):
        sq = imudata.orientation
        sw = imudata.angular_velocity
        sa = imudata.linear_acceleration

        #q = [sq.w, sq.x, sq.y, sq.z]
        #w = [sw.x, sw.y, sw.z]
        #a = [sa.x, sa.y, sa.z]
        self.csv_writer.writerow({'seq': imudata.header.seq,
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
                                  'secs': imudata.header.stamp.secs,
                                  'nsecs': imudata.header.stamp.nsecs})

    def __exit__(self):
        self.csv_file.close()


def read(csv_fn):
    csv_reader = csv.reader(open(csv_fn))
    csv_reader.__next__()  # skip title
    line0 = csv_reader.__next__()
    print('orientation', line0[1][1:-1].split(', '))  # orientation


if __name__ == "__main__":
    csv_fn = None
    if len(sys.argv) == 2:
        csv_fn = sys.argv[1]

    imu_recorder = IMURecorder(csv_fn)
    rospy.spin()
