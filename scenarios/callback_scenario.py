import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, HomePosition, VehicleStatus, VehicleOdometry
import time
import sys
import math


def xy2latlon(dx, dy, dz=None):
    new_lat = 1. * 47.333439 + (dy / 6378000) * (180 / math.pi)
    new_lon = 1. * 8.547097 + (dx / 6378000) * (180 / math.pi) / math.cos(47.333439 * math.pi / 180)
    if dz is None:
        return new_lat, new_lon
    else:
        return new_lat, new_lon, dz


class Vehicle():
    def __init__(self, node, id, waypoints):
        self.id = id
        self.node = node
        self.command = self.node.create_publisher(VehicleCommand, '/vehicle%d/in/VehicleCommand' % id, 10)
        self.home_sub = self.node.create_subscription(HomePosition, '/vehicle%d/out/HomePosition' % id, self.home_callback, 10)
        self.status_sub = self.node.create_subscription(VehicleStatus, '/vehicle%d/out/VehicleStatus' % id, self.status_callback, 10)
        self.odo_sub = self.node.create_subscription(VehicleOdometry, '/vehicle%d/out/VehicleOdometry' % id, self.odo_callback, 10)

    def home_callback(self, msg):
        print(msg)

    def status_callback(self, msg):
        print(msg)

    def odo_callback(self, msg):
        print(msg)
        # pass

def main(args=None):
    start_time = time.time()
    rclpy.init(args=args)

    node = Node('px4_command_publisher')

    vehicle1 = Vehicle(node, 1, [xy2latlon(150, 0, 5),
                                 xy2latlon(150, 90, 5),
                                 xy2latlon(240, 90, 5)])

    exit_value = 0
    # rclpy.spin(scenario_test)
    while rclpy.ok():
        rclpy.spin_once(node)
        # time.sleep(0.1)
        cur_time = time.time()

        if cur_time - start_time > 300:
            print("Scenario test time out, " + str(cur_time - start_time))
            break
    rclpy.shutdown()

    if exit_value == 0:
        print("SCENARIO PASS")
        exit(0)
    else:
        print("SCENARIO FAIL")
        exit(1)


if __name__ == '__main__':
    print(main())
