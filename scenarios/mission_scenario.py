import numpy as np
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleStatus, VehicleOdometry, VehicleGpsPosition
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
    MODE_INIT = 0
    MODE_TAKEOFF = 1

    def __init__(self, node, id, flight_alt:float =10.):
        self.id = id
        self.node = node
        self.pos = None
        self.home = None
        self.logger = logging.getLogger('agent%d' % id)
        self.ready = False
        self.nav_state = None
        self.arming_state = None
        self.vehicle_type = None
        self.mode = self.MODE_TAKEOFF
        self.waypoint = None

        self.flight_alt = flight_alt
        self.command = self.node.create_publisher(VehicleCommand, '/vehicle%d/in/VehicleCommand' % id, 10)
        self.command_sub = self.node.create_subscription(VehicleCommandAck, '/vehicle%d/out/VehicleCommandAck' % id, self.on_command_callback, 10)
        self.status_sub = self.node.create_subscription(VehicleStatus, '/vehicle%d/out/VehicleStatus' % id, self.on_status_callback, 10)
        self.gps_sub = self.node.create_subscription(VehicleGpsPosition, '/vehicle%d/out/VehicleGpsPosition' % id, self.on_gps_callback, 10)

        self.set_mode(0)

        self.init_pos = []

    def on_home_callback(self, msg):
        self.logger.info(msg)
        pass

    def on_command_callback(self, msg):
        self.logger.info(msg)

    def on_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.vehicle_type = msg.vehicle_type

        if self.mode == self.MODE_TAKEOFF:
            if self.nav_state != 17:
                self.takeoff()
            elif self.arming_state < 2:
                self.arm()

        self.logger.debug('nav_state: %d, arming_state: %d, mode: %d' % (self.nav_state, self.arming_state, self.mode))

    def on_gps_callback(self, msg):
        coor = [msg.lat * 1e-7, msg.lon * 1e-7, msg.alt * 1e-3]

        self.pos = coor
        self.logger.debug('Received GPS coordinate: [%.4f, %.4f, %.4f]' % tuple(self.pos))

    def arm(self):
        self.logger.info("send ARM command")
        arm_cmd = VehicleCommand()
        arm_cmd.target_system = self.id
        arm_cmd.command = 400
        arm_cmd.param1 = 1.0
        arm_cmd.confirmation = True
        arm_cmd.from_external = True
        self.command.publish(arm_cmd)

    def disarm(self):
        self.logger.info("send DISARM command")
        disarm_cmd = VehicleCommand()
        disarm_cmd.target_system = self.id
        disarm_cmd.command = 400
        disarm_cmd.param1 = 0.0
        disarm_cmd.confirmation = True
        disarm_cmd.from_external = True
        self.command.publish(disarm_cmd)

    def takeoff(self):

        self.logger.info("send Takeoff command")
        takeoff_cmd = VehicleCommand()
        takeoff_cmd.target_system = self.id
        takeoff_cmd.command = 22
        takeoff_cmd.param1 = -1.0
        # takeoff_cmd.param5 = pos[0]
        # takeoff_cmd.param6 = pos[1]
        takeoff_cmd.param7 = float(self.flight_alt)
        takeoff_cmd.confirmation = True
        takeoff_cmd.from_external = True
        self.command.publish(takeoff_cmd)

    def land(self):
        self.logger.info("send landing command")
        landing_cmd = VehicleCommand()
        landing_cmd.target_system = self.id
        landing_cmd.command = 21
        landing_cmd.from_external = True
        self.command.publish(landing_cmd)

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

    def set_mode(self, mode=192):
        self.logger.info("send SET MODE command")
        msg = VehicleCommand()
        msg.target_system = self.id
        msg.command = 176
        msg.param1 = float(mode)
        msg.confirmation = True
        msg.from_external = True
        self.command.publish(msg)

    def mission_move(self, pos):
        self.logger.info("send MOVE mission")
        msg = VehicleCommand()
        msg.target_system = self.id
        msg.command = 16
        # mission.param1 = float(hold_time)  # Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)
        # mission.param2 = float(radius)     # Acceptance radius in meters (if the sphere with this radius is hit, the MISSION counts as reached)
        msg.param3 = 0.0          # 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
        # mission.param4 = 0.0          # Desired yaw angle at MISSION (rotary wing)
        msg.param5 = float(pos[0])     # Latitude
        msg.param6 = float(pos[1])     # Longitude
        msg.param7 = float(pos[2])     # Altitude
        msg.confirmation = True
        msg.from_external = True
        self.command.publish(msg)

    def start_mission(self):
        self.logger.info("send START MISSION")
        start_mission = VehicleCommand()
        start_mission.command = 300
        start_mission.confirmation = True
        start_mission.from_external = True
        self.command.publish(start_mission)

def main(args=None):
    start_time = time.time()
    rclpy.init(args=args)

    node = Node('px4_command_publisher')

    vehicles = [Vehicle(node, i, 10) for i in range(1, 2)]

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
