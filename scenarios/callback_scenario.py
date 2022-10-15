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

        self.move(100., 100., 10.)

    def home_callback(self, msg):
        print(msg)

    def status_callback(self, msg):
        print(msg)

    def odo_callback(self, msg):
        print(msg)
        # pass

    def arm(self):
        print("send ARM command")
        arm_cmd = VehicleCommand()
        arm_cmd.target_system = self.id
        arm_cmd.command = 400
        arm_cmd.param1 = 1.0
        arm_cmd.confirmation = 0
        arm_cmd.from_external = True
        self.command.publish(arm_cmd)

    def move(self, x, y, z):
        print("send MOVE command")
        arm_cmd = VehicleCommand()
        arm_cmd.target_system = self.id
        arm_cmd.command = 81
        arm_cmd.param1 = 1.0
        arm_cmd.param2 = 1.0
        arm_cmd.param4 = 0.
        arm_cmd.param5 = x
        arm_cmd.param6 = y
        arm_cmd.param7 = z
        arm_cmd.confirmation = True
        arm_cmd.from_external = True
        self.command.publish(arm_cmd)

    def set_position(self, waypoint):
        print("send moving command")
        move_cmd = VehicleCommand()
        move_cmd.target_system = self.id
        move_cmd.command = 192
        move_cmd.param1 = -1.0
        move_cmd.param2 = 1.0
        move_cmd.param3 = 0.0
        move_cmd.param4 = float('nan')
        move_cmd.param5 = waypoint[0]
        move_cmd.param6 = waypoint[1]
        move_cmd.param7 = waypoint[2]
        move_cmd.confirmation = True
        move_cmd.from_external = True
        self.command.publish(move_cmd)

def main(args=None):
    start_time = time.time()
    rclpy.init(args=args)

    node = Node('px4_command_publisher')

    vehicle1 = Vehicle(node, 1, [xy2latlon(150., 0., 5.),
                                 xy2latlon(150., 90., 5.),
                                 xy2latlon(240., 90., 5.)])

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
