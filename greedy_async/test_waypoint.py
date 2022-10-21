import numpy as np
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, VehicleCommandAck, HomePosition, VehicleStatus, VehicleOdometry, VehicleGpsPosition
import time
import sys
import math
import logging

MIN_DIST = 3e-5

from scipy.spatial import KDTree

logging.basicConfig(level=logging.INFO)

def xy2latlon(dx, dy, dz=None):
    new_lat = 1. * 43.81678595809796 + ((dy * 2 - 100) / 6378000) * (180 / math.pi)
    new_lon = 1. * 28.581366799068558 + ((dx * 2 - 100) / 6378000) * (180 / math.pi) / math.cos(43.81678595809796 * math.pi / 180)
    if dz is None:
        return [new_lat, new_lon]
    else:
        return [new_lat, new_lon, float(dz)]


class Vehicle():
    MODE_INIT = 0
    MODE_STANDBY = 1
    MODE_PREFLIGHT = 2
    MODE_TAKEOFF = 3
    MODE_READY = 4
    MODE_MOVING = 5
    MODE_MISSION = 6

    TYPE_AIR = 1
    TYPE_LAND = 2
    TYPE_SEA = 3

    def __init__(self, node, id, type, tasks, flight_alt=None):
        self.id = id
        self.type = type
        self.tasks = tasks
        self.node = node
        if flight_alt is None:
            flight_alt = 4. + id if type == self.TYPE_AIR else 0.
        self.flight_alt = flight_alt

        self.location = None
        self.home = None
        self.logger = logging.getLogger('agent%d' % id)
        self.ready = False
        self.nav_state = None
        self.arming_state = None
        self.vehicle_type = None
        self.mode = self.MODE_INIT
        self.waypoints = []
        self.target = None

        self.curr_task = None
        self.in_mission = False

        self.flight_alt = flight_alt
        self.command = self.node.create_publisher(VehicleCommand, '/vehicle%d/in/VehicleCommand' % id, 10)
        self.command_sub = self.node.create_subscription(VehicleCommandAck, '/vehicle%d/out/VehicleCommandAck' % id, self.on_command_callback, 10)
        self.status_sub = self.node.create_subscription(VehicleStatus, '/vehicle%d/out/VehicleStatus' % id, self.on_status_callback, 10)
        self.gps_sub = self.node.create_subscription(VehicleGpsPosition, '/vehicle%d/out/VehicleGpsPosition' % id, self.on_gps_callback, 10)

        self.init_pos = []

    def on_home_callback(self, msg):
        pass

    def on_command_callback(self, msg):
        self.logger.debug(msg)

    def on_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.vehicle_type = msg.vehicle_type

        if self.mode > self.MODE_STANDBY and self.arming_state == 1:
            self.mode = self.MODE_STANDBY
        if self.mode == self.MODE_STANDBY:
            if self.arming_state == 2:
                if self.type == self.TYPE_AIR:
                    self.mode = self.MODE_PREFLIGHT
                else:
                    self.mode = self.MODE_READY
            else:
                self.arm()
                time.sleep(1)
        elif self.mode == self.MODE_PREFLIGHT:
            if self.nav_state == 17:
                self.mode = self.MODE_TAKEOFF
            elif self.nav_state == 4 and abs(self.location[2] - self.flight_alt) < 0.5:
                self.mode = self.MODE_READY
            else:
                self.takeoff()
        elif self.mode == self.MODE_TAKEOFF:
            if self.nav_state == 4:
                self.mode = self.MODE_READY
            elif self.nav_state == 5:
                self.mode = self.MODE_STANDBY
        elif self.mode == self.MODE_READY:
            if len(self.waypoints) > 0:
                _, pot_lon = xy2latlon(55, 0)
                _, neg_lon = xy2latlon(45, 0)
                if self.location[1] >= pot_lon and self.waypoints[0][1] < neg_lon:
                    self.waypoints = [xy2latlon(50, 85.35534)] + self.waypoints
                elif self.location[1] <= neg_lon and self.waypoints[0][1] > pot_lon:
                    self.waypoints = [xy2latlon(50, 78.28427)] + self.waypoints

                print(self.waypoints)
                self.target = list(self.waypoints[0]) + [self.flight_alt]
                del self.waypoints[0]
                self.set_position(self.target)
                print(self.target)
                self.mode = self.MODE_MOVING
            elif self.in_mission:
                if self.curr_task:
                    self.curr_task.done = True
                new_task = self.find_task()
                if new_task is not None:
                    self.curr_task = new_task
                    self.waypoints += new_task.locations
                    new_task.assigned = True
                else:
                    self.curr_task = None

        self.logger.info('nav_state: %d, arming_state: %d, mode: %d' % (self.nav_state, self.arming_state, self.mode))

    def on_gps_callback(self, msg):
        coor = [msg.lat * 1e-7, msg.lon * 1e-7, msg.alt * 1e-3]
        if self.home is None:
            self.init_pos.append(coor)
            if len(self.init_pos) >= 10:
                self.home = np.average(self.init_pos, axis=0).tolist()
                del self.init_pos
                self.mode = self.MODE_STANDBY

        self.location = coor
        if self.mode == self.MODE_MOVING:
            dist = np.sqrt((self.target[0] - self.location[0]) ** 2 + (self.target[1] - self.location[1]) ** 2)
            self.logger.debug('Distance to waypoint: %.4f' % dist)
            if dist < MIN_DIST:
                self.mode = self.MODE_READY
                self.target = None

        self.logger.debug('Received GPS coordinate: [%.4f, %.4f, %.4f]' % tuple(self.location))

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

    def takeoff(self, pos=None, flight_alt=None):
        if not pos:
            pos = self.location
        if not flight_alt:
            flight_alt = self.flight_alt
        self.logger.info("send Takeoff command")
        takeoff_cmd = VehicleCommand()
        takeoff_cmd.target_system = self.id
        takeoff_cmd.command = 22
        takeoff_cmd.param1 = 0.0
        takeoff_cmd.param5 = pos[0]
        takeoff_cmd.param6 = pos[1]
        takeoff_cmd.param7 = flight_alt
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
        if self.type == self.TYPE_AIR:
            move_cmd.param7 = waypoint[2]
        move_cmd.confirmation = True
        move_cmd.from_external = True
        self.command.publish(move_cmd)

    def find_task(self):
        available_tasks = [task for task in self.tasks.values() if (not task.done) and (not task.assigned) and (self.type in task.types)]
        if available_tasks:
            tree = KDTree([[task.locations[0][0], task.locations[0][1]] for task in available_tasks])
            _, idx = tree.query([self.location[0], self.location[1]])
            return available_tasks[idx]
        return None


    # def set_mode(self, mode=216):
    #     self.logger.info("send SET MODE command")
    #     msg = VehicleCommand()
    #     msg.target_system = self.id
    #     msg.command = 176
    #     msg.param1 = float(mode)
    #     msg.confirmation = True
    #     msg.from_external = True
    #     self.command.publish(msg)


class Task:
    def __init__(self, id, locations, types):
        self.id = id
        self.locations = locations
        self.types = types
        self.assigned = False
        self.done = False


def find_task(tasks_location, location):
    tree = KDTree(tasks_location)
    _, idx = tree.query([location[0], location[1]])
    return idx


def main(args=None):
    from data import bases, agents, search_nodes, enemy_nodes

    start_time = time.time()

    rclpy.init(args=args)
    px4_node = Node('px4_command_publisher')

    vehicle = Vehicle(px4_node, 1, Vehicle.TYPE_SEA, None)

    exit_value = 0
    assignment_ready = True

    while rclpy.ok():
        rclpy.spin_once(px4_node)
        cur_time = time.time()

        if assignment_ready and vehicle.mode == Vehicle.MODE_READY:
            vehicle.waypoints.append(xy2latlon(20, 60))
            assignment_ready = False

    rclpy.shutdown()

    if exit_value == 0:
        print("SCENARIO PASS")
        exit(0)
    else:
        print("SCENARIO FAIL")
        exit(1)


if __name__ == '__main__':
    print(main())
