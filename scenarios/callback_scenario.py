import numpy as np
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, HomePosition, VehicleStatus, VehicleOdometry, VehicleGpsPosition
import time
import sys
import math
import logging
logging.basicConfig(level=logging.DEBUG)


def xy2latlon(dx, dy, dz=None):
    new_lat = 1. * 47.333439 + (dy / 6378000) * (180 / math.pi)
    new_lon = 1. * 8.547097 + (dx / 6378000) * (180 / math.pi) / math.cos(47.333439 * math.pi / 180)
    if dz is None:
        return new_lat, new_lon
    else:
        return new_lat, new_lon, dz


class Vehicle():
    def __init__(self, node, id, flight_alt=5.):
        self.id = id
        self.node = node
        self.pos = None
        self.home = None
        self.logger = logging.getLogger('agent%d' % id)
        self.ready = False
        self.nav_state = None
        self.arming_state = None

        self.flight_alt = 5.
        self.command = self.node.create_publisher(VehicleCommand, '/vehicle%d/in/VehicleCommand' % id, 10)
        self.home_sub = self.node.create_subscription(HomePosition, '/vehicle%d/out/HomePosition' % id, self.on_home_callback, 10)
        self.status_sub = self.node.create_subscription(VehicleStatus, '/vehicle%d/out/VehicleStatus' % id, self.on_status_callback, 10)
        self.gps_sub = self.node.create_subscription(VehicleGpsPosition, '/vehicle%d/out/VehicleGpsPosition' % id, self.on_gps_callback, 10)

        self.init_pos = []

    def on_ready(self):
        pass

    def on_home_callback(self, msg):
        # print(msg)
        pass

    def on_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.logger.debug('nav_state: %d, arming_state: %d' % (self.nav_state, self.arming_state))

    def on_gps_callback(self, msg):
        coor = [msg.lat * 1e-7, msg.lon * 1e-7, msg.alt * 1e-3]
        if not self.home:
            self.init_pos.append(coor)
            if len(self.init_pos) >= 10:
                self.home = np.average(self.init_pos, axis=0).tolist()

        self.pos = coor
        self.logger.debug('Received GPS coordinate: [%.4f, %.4f, %.4f]' % tuple(coor))
        # pass

    def arm(self):
        self.logger.info("send ARM command")
        arm_cmd = VehicleCommand()
        arm_cmd.target_system = self.id
        arm_cmd.command = 400
        arm_cmd.param1 = 1.0
        arm_cmd.confirmation = 0
        arm_cmd.from_external = True
        self.command.publish(arm_cmd)

    def disarm(self):
        self.logger.info("send DISARM command")
        disarm_cmd = VehicleCommand()
        disarm_cmd.target_system = self.id
        disarm_cmd.command = 400
        disarm_cmd.param1 = 0.0
        disarm_cmd.confirmation = 0
        disarm_cmd.from_external = True
        self.command.publish(disarm_cmd)

    def takeoff(self, lat, lon, alt, takeoff_value):
        self.logger.info("send Takeoff command")
        takeoff_cmd = VehicleCommand()
        takeoff_cmd.target_system = self.id
        takeoff_cmd.command = 22
        takeoff_cmd.param1 = -1.0
        takeoff_cmd.param2 = 0.0
        takeoff_cmd.param3 = 0.0
        takeoff_cmd.param4 = 0.0
        # lat: 47.39775103965341
        # lon: 8.545607598150605
        # alt: 488.1470947265625
        takeoff_cmd.param5 = lat
        takeoff_cmd.param6 = lon
        takeoff_cmd.param7 = alt + takeoff_value
        takeoff_cmd.confirmation = True
        takeoff_cmd.from_external = True
        self.command.publish(takeoff_cmd)

    def set_position(self, waypoint):
        self.logger.info("send moving command")
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
    vehicle2 = Vehicle(node, 2, [xy2latlon(150., 0., 5.),
                                 xy2latlon(150., 90., 5.),
                                 xy2latlon(240., 90., 5.)])
    vehicle3 = Vehicle(node, 3, [xy2latlon(150., 0., 5.),
                                 xy2latlon(150., 90., 5.),
                                 xy2latlon(240., 90., 5.)])
    vehicle4 = Vehicle(node, 4, [xy2latlon(150., 0., 5.),
                                 xy2latlon(150., 90., 5.),
                                 xy2latlon(240., 90., 5.)])
    vehicle5 = Vehicle(node, 5, [xy2latlon(150., 0., 5.),
                                 xy2latlon(150., 90., 5.),
                                 xy2latlon(240., 90., 5.)])
    vehicle6 = Vehicle(node, 6, [xy2latlon(150., 0., 5.),
                                 xy2latlon(150., 90., 5.),
                                 xy2latlon(240., 90., 5.)])
    vehicle7 = Vehicle(node, 7, [xy2latlon(150., 0., 5.),
                                 xy2latlon(150., 90., 5.),
                                 xy2latlon(240., 90., 5.)])
    vehicle8 = Vehicle(node, 8, [xy2latlon(150., 0., 5.),
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
