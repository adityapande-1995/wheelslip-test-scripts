#!python3
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from subprocess import Popen, PIPE
import os, signal
import time
import rclpy
from rclpy.node import Node
from threading import Thread
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

class RunSim:
    def __init__(self, init_timeout = 3.0, sim_timeout = 5.0):
        self.init_timeout = init_timeout
        self.sim_timeout = sim_timeout
        rclpy.init()
        self.node_running_flag = False
        self.ros_node = Node('record_data_node')

    def start_sim(self):
        self.gz_process = Popen(['gzserver', 'slip_trial.world', '--verbose'])
        print('Launched sim')

    def start_node(self):
        self.node_running_flag = True
        self.ros_spin_thread = Thread(target=self.spin_function)
        self.ros_spin_thread.start()

    def start_subs(self):
        self.sub_1_first_flag = True
        self.sub_2_first_flag = True

        self.sub_1_first_msg = None
        self.sub_2_first_msg = None
        self.sub_1_last_msg = None
        self.sub_2_last_msg = None

        self.subs_1 = self.ros_node.create_subscription(JointState, 'joint_states', self.subs_1_cb, 10)
        self.subs_2 = self.ros_node.create_subscription(Odometry, 'odom', self.subs_2_cb, 10)

    def subs_1_cb(self, data):
        if self.sub_1_first_flag:
            self.sub_1_first_msg = data
            self.sub_1_first_flag = False
        else:
            self.sub_1_last_msg = data

    def subs_2_cb(self, data):
        if self.sub_2_first_flag:
            self.sub_2_first_msg = data
            self.sub_2_first_flag = False
        else:
            self.sub_2_last_msg = data

    def spin_function(self):
        while self.node_running_flag:
            rclpy.spin_once(self.ros_node, timeout_sec=0.1)

    def shutdown_sim(self):
        print('Trying to kill sim process')
        os.kill(self.gz_process.pid, signal.SIGINT)
        time.sleep(2.0)
        if not self.gz_process.poll():
            print('Sim shutdown successful')

    def shutdown_node(self):
        self.node_running_flag = False
        self.ros_spin_thread.join()
        self.ros_node.destroy_node()
        rclpy.shutdown()

    def __enter__(self):
        self.start_sim()
        self.start_node()
        # Let the sim run for some time
        time.sleep(self.init_timeout)
        self.start_subs()
        # Record data for 1 sec
        time.sleep(self.sim_timeout)

    def __exit__(self,exep_type, exep_value, trace):
        if exep_type is not None:
            raise Exception('Exception occured, value: ', exep_value)
        self.shutdown_node()
        self.shutdown_sim()

    def get_velocities(self):
        # wheel odom
        t1 = self.sub_1_first_msg.header.stamp.sec + self.sub_1_first_msg.header.stamp.nanosec / 10**9
        t2 = self.sub_1_last_msg.header.stamp.sec + self.sub_1_last_msg.header.stamp.nanosec / 10**9

        p1 = np.array(self.sub_1_first_msg.position)
        p2 = np.array(self.sub_1_last_msg.position)
        wheel_speed = np.mean((p2 - p1) / (t2 - t1)) * 0.15

        # Ground truth
        t1 = self.sub_2_first_msg.header.stamp.sec + self.sub_2_first_msg.header.stamp.nanosec / 10**9
        t2 = self.sub_2_last_msg.header.stamp.sec + self.sub_2_last_msg.header.stamp.nanosec / 10**9

        x1, y1 = self.sub_2_first_msg.pose.pose.position.x, self.sub_2_first_msg.pose.pose.position.y
        x2, y2 = self.sub_2_last_msg.pose.pose.position.x, self.sub_2_last_msg.pose.pose.position.y

        dist = ( (x2 - x1)**2 + (y2 - y1)**2 )**0.5
        actual_speed = dist * np.sign(x2 - x1)/ (t2 - t1)

        return wheel_speed , actual_speed

def create_world_file(slip_lat = 0.0, slip_long = 0.0, incline_deg = 0.0):
    lines = open('slip_base.world', 'r').readlines()

    # Replace gravity
    g = "\t<gravity>" + str(round(-9.8 * np.sin(incline_deg * np.pi/180), 3)) + " 0 " \
        + str(round(-9.8 * np.cos(incline_deg * np.pi/180),3)) + "</gravity>"

    lines[ 5 - 1] = g


    # Change slip comliance values
    slip_lat_string = '\t\t<slip_compliance_lateral>' + str(slip_lat) + '</slip_compliance_lateral>\n'
    slip_long_string = '\t\t<slip_compliance_longitudinal>' + str(slip_long) + '</slip_compliance_longitudinal>\n'

    for l in [21, 26, 31]:
        lines[l - 1] = slip_lat_string
        lines[l] = slip_long_string

    # Write the file again
    with open('slip_trial.world', 'w') as out:
        out.writelines(lines)

# --------------- Main ---------------------

# Run several simulations
slip_values = np.array([0.0, 1.0, 2.0, 3.0])
incline_list = np.linspace(-40,40,60)
total_iter = len(incline_list) * len(slip_values)
counter = 0

for slip in slip_values:
    y = []
    x = []
    for theta in incline_list:
        counter += 1
        create_world_file(slip, slip, theta)
        a = RunSim(init_timeout=3.0, sim_timeout=2.0)
        with a:
            print('\n ** Iteration ', counter, ' of ', total_iter)

        wheel_speed, actual_speed = a.get_velocities()
        y_data = (actual_speed - wheel_speed)
        x.append(theta) ; y.append(y_data)

    # Plot stuff
    plt.plot(x,y, label='slip_compliance = ' + str(slip))

plt.legend(loc='best')
plt.title('actual - wheel speed vs incline angle')
plt.grid()
plt.savefig('slip_vs_incline.png')
