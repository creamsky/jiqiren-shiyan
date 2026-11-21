"""Require fresh, advancing headers from the new sensor pipeline before ready."""
import sys
import threading
import time

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2


def main():
    rospy.init_node('verify_prepared_sensors', anonymous=True)
    after = float(sys.argv[1])
    topics = {'/imu': Imu, '/velodyne_points': PointCloud2,
              '/Odometry': Odometry, '/cloud_registered': PointCloud2}
    counts = dict.fromkeys(topics, 0)
    stamps = dict.fromkeys(topics, 0.)
    lock = threading.Lock()

    def receive(msg, topic):
        stamp = msg.header.stamp.to_sec()
        now = rospy.Time.now().to_sec()
        if stamp < after or not -.10 <= now-stamp <= .50:
            return
        if isinstance(msg, PointCloud2) and msg.width*msg.height == 0:
            return
        with lock:
            if stamp > stamps[topic]:
                stamps[topic] = stamp
                counts[topic] += 1

    subscribers = [rospy.Subscriber(topic, kind, receive, callback_args=topic,
                                    queue_size=1, buff_size=2**22) for topic, kind in topics.items()]
    deadline = time.monotonic()+45
    while not rospy.is_shutdown() and time.monotonic()<deadline:
        with lock:
            now = rospy.Time.now().to_sec()
            if all(counts[t]>=3 and now-stamps[t]<=.50 for t in topics):
                print('Fresh post-restart streams:', counts)
                return 0
        time.sleep(.05)
    print('Fresh stream counts:', counts, 'last stamps:', stamps, file=sys.stderr)
    return 1


if __name__ == '__main__':
    sys.exit(main())
