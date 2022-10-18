import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleGpsPosition
import time
import sys
import math

class ScenarioTest(Node):
    def __init__(self):
        super().__init__('px4_command_publisher')
        self.publisher1_ = self.create_publisher(VehicleCommand, '/vehicle1/in/VehicleCommand', 10)
        # self.vehicle_local_pose_sup = self.create_subscription(VehicleLocalPosition, '/vehicle1/VehicleLocalPosition_PubSubTopic', self.listener_callback, 10)
        #self.subscriber1_
        self.vehicle_status_sup = self.create_subscription(VehicleStatus, '/vehicle1/out/VehicleStatus', self.vehicle_status_listener_callback, 10)
        self.vehicle_gps_sup = self.create_subscription(VehicleGpsPosition, '/vehicle1/out/VehicleGpsPosition', self.vehicle_gps_listener_callback, 10)
        self.vehicle1_init_lat = 0.0
        self.vehicle1_init_lon = 0.0
        self.vehicle1_init_alt = 0.0

        self.vehicle1_gps_lat = 0.0
        self.vehicle1_gps_lon = 0.0
        self.vehicle1_gps_alt = 0.0

        self.arming_state = 0
        self.moving_value = 0
        self.takeoff_value = 5
        self.mode = 0
        # armed = 0, takeoff 1, move position 2, success 99
        self.tgt_success = 0
        self.success_cnt = 0
        self.tgt_pos = [47.3334583, 8.5493813, 5.000]
        self.cur_pos = [0.0, 0.0, 0.0]
        self.pos_thres = 0.1
        self.exit_value = -1

    def vehicle_gps_listener_callback(self, msg):
        self.vehicle1_gps_lat = msg.lat * 1e-7
        self.vehicle1_gps_lon = msg.lon * 1e-7
        self.vehicle1_gps_alt = msg.alt * 1e-3
        self.cur_pos = [self.vehicle1_gps_lat, self.vehicle1_gps_lon, self.vehicle1_gps_alt]
        if self.arming_state == 2:
            check_pos = self.check_target_position(self.tgt_pos, self.cur_pos)
            if check_pos == True:
                self.tgt_success = 1

    def listener_callback(self, msg):
        self.vehicle1_init_lat = msg.ref_lat
        self.vehicle1_init_lon = msg.ref_lon
        self.vehicle1_init_alt = msg.ref_alt


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
        if self.exit_value == -1 :
            if self.nav_state != 0 and self.arming_state == 1 and self.mode == 0 and self.armed_time <= 0:
                self.arming()
                self.mode = 1
            elif self.nav_state != 0 and self.arming_state == 2 and self.mode == 1 and self.takeoff_time <= 0:
                self.takeoff(self.vehicle1_gps_lat, self.vehicle1_gps_lon, self.vehicle1_init_alt, self.takeoff_value)
                print("first takeoff")
                self.mode = 2
            elif self.nav_state == 17 and self.arming_state == 2 and self.mode == 2 and self.vehicle1_gps_alt >= self.takeoff_value:
                self.set_position(self.tgt_pos[0], self.tgt_pos[1], self.tgt_pos[2])
                self.mode = 3
            elif self.arming_state == 2 and self.mode == 3 and self.tgt_success == 1:
                print("Reach the target position")
                self.landing()
                self.mode = 4
            elif self.mode == 4 and self.arming_state == 1 and self.armed_time <= 0:
                self.disarming()
                self.exit_value = 0
            if self.fail_safe == True:
                print("Fail safe detected!")
                self.exit_value = 1

    def check_target_position(self, tgt_pos, cur_pos, thres=0.01):
        dist = math.dist(tgt_pos, cur_pos)
        if self.mode == 2 or self.mode == 3:
            print("distance of taget-current position: " + str(dist))
        if dist <= thres:
            return True
        else:
            return False

    def arming(self):
        print("send ARM command")
        arm_cmd = VehicleCommand()
        arm_cmd.target_system = 1
        arm_cmd.command = 400
        arm_cmd.param1 = 1.0
        arm_cmd.confirmation = 0
        arm_cmd.from_external = True
        self.publisher1_.publish(arm_cmd)

    def disarming(self):
        print("send DISARM command")
        disarm_cmd = VehicleCommand()
        disarm_cmd.target_system = 1
        disarm_cmd.command = 400
        disarm_cmd.param1 = 0.0
        disarm_cmd.confirmation = 0
        disarm_cmd.from_external = True
        self.publisher1_.publish(disarm_cmd)

    def takeoff(self, lat, lon, alt, takeoff_value):
        print("send Takeoff command")
        takeoff_cmd = VehicleCommand()
        takeoff_cmd.target_system = 1
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
        self.publisher1_.publish(takeoff_cmd)

    def set_position(self, lat, lon, alt):
        print("send moving command")
        move_cmd = VehicleCommand()
        move_cmd.target_system = 1
        move_cmd.command = 192
        move_cmd.param1 = -1.0
        move_cmd.param2 = 1.0
        move_cmd.param3 = 0.0
        move_cmd.param4 = float('nan')
        move_cmd.param5 = lat
        move_cmd.param6 = lon
        move_cmd.param7 = alt
        move_cmd.confirmation = True
        move_cmd.from_external = True
        self.publisher1_.publish(move_cmd)

    def landing(self):
        print("send landing command")
        landing_cmd = VehicleCommand()
        landing_cmd.target_system = 1
        landing_cmd.command = 21
        landing_cmd.from_external = True
        self.publisher1_.publish(landing_cmd)

def main(args=None):
    start_time = time.time()
    rclpy.init(args=args)
    scenario_test = ScenarioTest()
    exit_value = 0
    # rclpy.spin(scenario_test)
    while rclpy.ok():
        rclpy.spin_once(scenario_test)
        # time.sleep(0.1)
        cur_time = time.time()
        if scenario_test.exit_value != -1:
            break
        if cur_time - start_time > 300:
            print("Scenario test time out, " + str(cur_time - start_time))
            break
    exit_value = scenario_test.exit_value
    scenario_test.destroy_node()
    rclpy.shutdown()

    if exit_value == 0:
        print("SCENARIO PASS")
        exit(0)
    elif exit_value == 1:
        print("SCENARIO FAIL")
        exit(1)
    elif exit_value == -1:
        print("SCENARIO FAIL")
        exit(2)

if __name__ == '__main__':
    print(main())