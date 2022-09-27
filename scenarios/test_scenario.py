import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleGpsPosition
import time
import sys
import math


def xy2latlon(dx, dy):
    new_lat = 1. * 47.333439 + (dy / 6378000) * (180 / math.pi)
    new_lon = 1. * 8.547097 + (dx / 6378000) * (180 / math.pi) / math.cos(47.333439 * math.pi / 180)
    return new_lat, new_lon


class Vehicle(Node):
    def __init__(self, node, id, waypoints):
        super().__init__('px4_vehicle%d' % id)
        self.id = id
        self.node = node
        self.publisher = self.node.create_publisher(VehicleCommand, '/vehicle%d/VehicleCommand_PubSubTopic' % id, 10)
        self.vehicle_local_pose_sup = self.node.create_subscription(VehicleLocalPosition,
                                                                    '/vehicle%d/VehicleLocalPosition_PubSubTopic' % id,
                                                                    self.listener_callback, 10)
        self.vehicle_status_sup = self.node.create_subscription(VehicleStatus,
                                                                '/vehicle%d/VehicleStatus_PubSubTopic' % id,
                                                                self.vehicle_status_listener_callback, 10)
        self.vehicle_gps_sup = self.node.create_subscription(VehicleGpsPosition,
                                                             '/vehicle%d/VehicleGpsPosition_PubSubTopic' % id,
                                                             self.vehicle_gps_listener_callback, 10)

        self.init_pos = [0., 0., 0.]
        self.gps_pos = [0., 0., 0.]

        self.arming_state = 0
        self.moving_value = 0
        self.takeoff_value = 5
        self.mode = 0
        # armed = 0, takeoff 1, move position 2, success 99
        self.tgt_success = 0
        self.success_cnt = 0

        self.waypoints = waypoints
        self.pos_thres = 0.1
        self.exit_value = -1

    def vehicle_gps_listener_callback(self, msg):
        self.gps_pos = [msg.lat * 1e-7, msg.lon * 1e-7, msg.alt * 1e-3]
        if self.arming_state == 2:
            check_pos = self.check_target_position(self.waypoints[0], self.gps_pos)
            if check_pos == True:
                self.tgt_success = 1

    def listener_callback(self, msg):
        self.init_pos = [msg.ref_lat, msg.ref_lon, msg.ref_alt]

    def vehicle_status_listener_callback(self, msg):
        self.fail_safe = msg.failsafe
        self.arming_state = msg.arming_state
        # 1 -> standby
        # 2 -> armed
        self.nav_state = msg.nav_state
        self.takeoff_time = msg.takeoff_time
        self.armed_time = msg.armed_time
        # 18 -> standby
        # 17 -> take-off
        # 4 -> moving
        if self.exit_value == -1:
            if self.nav_state != 0 and self.arming_state == 1 and self.mode == 0 and self.armed_time <= 0:
                self.arming()
                self.mode = 1
            elif self.nav_state != 0 and self.arming_state == 2 and self.mode == 1 and self.takeoff_time <= 0:
                self.takeoff(self.init_pos[0], self.init_pos[1], self.init_pos[2], self.takeoff_value)
                print("first takeoff")
                self.mode = 2
            elif self.nav_state == 17 and self.arming_state == 2 and self.mode == 2 and self.gps_pos[
                2] >= self.takeoff_value:
                self.set_position(self.waypoints[0])
                self.mode = 3
            elif self.arming_state == 2 and self.mode == 3 and self.tgt_success == 1:
                print("Reach the target position")
                if len(self.waypoints) == 0:
                    self.landing()
                    self.mode = 4
                else:
                    self.tgt_success = 0
                    del self.waypoints[0]
                    self.set_position(self.waypoints[0])

            elif self.mode == 4 and self.arming_state == 1 and self.armed_time <= 0:
                self.disarming()
                self.exit_value = 0
            if self.fail_safe == True:
                print("Fail safe detected!")
                self.exit_value = 1

    def check_target_position(self, tgt_pos, cur_pos, thres=0.0001):
        dist = math.dist([tgt_pos[0], tgt_pos[1]], [cur_pos[0], cur_pos[1]])
        if self.mode == 2 or self.mode == 3:
            print("distance of taget-current position: " + str(dist) + 'coordinate: %.8f, %.8f, %.8f' % tuple(cur_pos))
        if dist <= thres:
            return True
        else:
            return False

    def arming(self):
        print("send ARM command")
        arm_cmd = VehicleCommand()
        arm_cmd.target_system = self.id
        arm_cmd.command = 400
        arm_cmd.param1 = 1.0
        arm_cmd.confirmation = 0
        arm_cmd.from_external = True
        self.publisher.publish(arm_cmd)

    def disarming(self):
        print("send DISARM command")
        disarm_cmd = VehicleCommand()
        disarm_cmd.target_system = self.id
        disarm_cmd.command = 400
        disarm_cmd.param1 = 0.0
        disarm_cmd.confirmation = 0
        disarm_cmd.from_external = True
        self.publisher.publish(disarm_cmd)

    def takeoff(self, lat, lon, alt, takeoff_value):
        print("send Takeoff command")
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
        self.publisher.publish(takeoff_cmd)

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
        self.publisher.publish(move_cmd)

    def landing(self):
        print("send landing command")
        landing_cmd = VehicleCommand()
        landing_cmd.target_system = self.id
        landing_cmd.command = 21
        landing_cmd.from_external = True
        self.publisher.publish(landing_cmd)


def main(args=None):
    start_time = time.time()
    rclpy.init(args=args)

    node = Node()

    lat1, lon1 = xy2latlon(150, 0)
    lat2, lon2 = xy2latlon(150, 90)
    lat3, lon3 = xy2latlon(240, 90)

    vehicle1 = Vehicle(node, 1, [[lat1, lon1, 5.0],
                                 [lat2, lon2, 5.0],
                                 [lat3, lon3, 5.0]])

    exit_value = 0
    # rclpy.spin(scenario_test)
    while rclpy.ok():
        rclpy.spin_once(vehicle1)
        # time.sleep(0.1)
        cur_time = time.time()
        if vehicle1.exit_value != -1:
            break
        if cur_time - start_time > 300:
            print("Scenario test time out, " + str(cur_time - start_time))
            break
    exit_value = vehicle1.exit_value
    vehicle1.destroy_node()
    rclpy.shutdown()

    if exit_value == 0:
        print("SCENARIO PASS")
        exit(0)
    else:
        print("SCENARIO FAIL")
        exit(1)


if __name__ == '__main__':
    print(main())
