import csv
import datetime
import time
import rospy
from sensor_msgs.msg import Imu


class IMURecorder:
    def __init__(self, csv_fn=None):
        if csv_fn is None:
            ctime = datetime.datetime.fromtimestamp(time.time())
            csv_fn = ctime.strftime('%Y%m%d_%H%M%S') + '.csv'

        fieldnames = ['seq', 'orientation', 'angular velocity', 'linear acceleration',
                      'secs', 'nsecs']
        self.csv_file = open(csv_fn, mode='w')
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames)
        self.csv_writer.writerow({'seq': 'seq',
                                  'orientation': 'orientation',
                                  'angular velocity': 'angular velocity',
                                  'linear acceleration': 'linear acceleration',
                                  'secs': 'secs',
                                  'nsecs': 'nsecs'})

        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("imu", Imu, self.imu_callback)

    def imu_callback(self, imudata):
        sq = imudata.orientation
        sw = imudata.angular_velocity
        sa = imudata.linear_acceleration

        q = (sq.x, sq.y, sq.z, sq.w)
        w = (sw.x, sw.y, sw.z)
        a = (sa.x, sa.y, sa.z)
        self.csv_writer.writerow({'seq': imudata.header.seq,
                                  'orientation': q,
                                  'angular velocity': w,
                                  'linear acceleration': a,
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
    imu_recorder = IMURecorder()
    rospy.spin()
