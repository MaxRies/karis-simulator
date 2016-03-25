__author__ = 'Maximilian Ries'

import rospy
from std_msgs.msg import Int8, Int16
from geometry_msgs.msg import PoseStamped
from karislib.msg import LiftCommand, NaviStatus, MultiNavRobotInformation, LiftStatus
import argparse
import random

'''
Sonstige Parameter
'''
karis_max_speed = 2.0         # max speed in m/s
karis_feinpos_speed = 0.1   # Speed during fine positioning
loop_frequency = 100        # frequency of main loop. battery constants rely on this one.
feinpos_time = 30           # estimated time for fine positioning. every approach is randomized

'''
Battery drain factors.
Battery is drained once per main loop.
The main loop runs with a frequency of 10 Hz.
Karis is supposed to be operational for 2 hours with a single charge.
2 h = 7200 s
7200 s * 10 Hz = 72000 loop iterations
1000 / 72000 loops = 0.01388889 Percent*10/Loop
'''
drive_drain = 0.5/loop_frequency      # Battery drain while driving
waiting_drain = 0.1/loop_frequency   # Battery drain while waiting
feinpos_drain = 0.03/loop_frequency    # Battery drain while positioning
hub_drain = 0.04/loop_frequency        # Battery drain while using the lift



class Karis:
    state = 'driving'
    battery_charge = 1000
    position_x = 0.0
    position_y = 0.0
    position_1_x = 10.0
    position_1_y = 10.0
    position_1_reached = False
    position_2_reached = False
    name = 'karis1'         # Name format: karis[n]


    def __init__(self, name, initial_charge, start_x, start_y):
        self.battery_charge = initial_charge
        self.name = name
        self.position_x = start_x
        self.position_y = start_y
        rospy.init_node(self.name, anonymous = False)
        self.subscribe_to_topics()
        self.setup_publishers()
    '''
    Stuff that interacts with ROS message system
    '''
    def subscribe_to_topics(self):
        rospy.Subscriber('/'+self.name+'/move_base_simple/goal', PoseStamped, self.move_base_callback)
        rospy.Subscriber('/'+self.name+'/position_command', Int8, self.feinpos_callback)
        rospy.Subscriber('/'+self.name+'/fp_reset_command', Int8, self.feinpos_reset_callback)
        rospy.Subscriber('/'+self.name+'/karis_lift_command', LiftCommand, self.liftcommand_callback)

    def setup_publishers(self):
        self.status_pub = rospy.Publisher('/'+self.name+'/karis_navigation_status', NaviStatus, queue_size=10)
        self.battery_pub = rospy.Publisher('/'+self.name+'/karis_battery_status', Int16, queue_size=10)
        self.robot_information_pub = rospy.Publisher('/karis_robot_information', MultiNavRobotInformation, queue_size=10)
        self.feinpos_pub = rospy.Publisher('/'+self.name+'/status_fp_2_administartion', Int8, queue_size=10)
        self.lift_pub = rospy.Publisher('/'+self.name+'/karis_lift_status', LiftStatus, queue_size=10)

    def publish_updates(self):
        self.battery_pub.publish(self.battery_charge)

    '''
    Callback functions for ROS message system
    '''
    def move_base_callback(self, data):
        self.position_1_x = data.pose.position.x
        self.position_1_y = data.pose.position.y
        if self.position_1_x != self.position_x or self.position_1_y != self.position_y:
            print('New goal: x: {0} y: {1}'.format(self.position_1_x, self.position_1_y))
            self.position_1_reached = False
            self.state = 'driving'

    def feinpos_callback(self, data):
        # TODO Hier noch weitermachen. Warten, dann melden das Feinpositioniert wurde (siehe PDF von patric)
        if data == 1 and self.position_1_reached == True:
            self.state = 'feinpos'
            self.position_1_reached = False
            self.feinpos.count = 0  # Reset counter for feinpositioning function
            self.feinpos.time = feinpos_time + random.randint(-20,20)    #Time to position is randomized
        elif data == 1 and self.position_2_reached == True:
            self.state = 'feinpos'
            self.feinpos.count = 0
            self.feinpos.time = feinpos_time + random.randint(-20,20)


    def feinpos_reset_callback(self, data):
        pass

    def liftcommand_callback(self, data):
        pass
    '''
    Drain the battery according to the state our karis is in
    '''
    def drain_battery(self):
        if self.state == 'driving':
            self.battery_charge = self.battery_charge - drive_drain
        elif self.state == 'waiting':
            self.battery_charge = self.battery_charge - waiting_drain
        elif self.state == 'feinpos':
            self.battery_charge = self.battery_charge - feinpos_drain
        elif self.state == 'hub':
            self.battery_charge = self.battery_charge - hub_drain

    '''
    Karis moves in an orthogonal grid to its final position.
    It first drives to the desired x coordinate, then to the desired y coordinate.
    A tolerance interval of +- 0.1 m around the desired goal position is accepted.
    '''
    def drive_to_position_1(self):
        if self.position_1_reached == False:
            if self.position_x < self.position_1_x - 0.1:
                print('PENISPENISPENISPENIS')
                self.position_x = self.position_x + karis_max_speed/loop_frequency
            elif self.position_x > self.position_1_x + 0.1:
                self.position_x = self.position_x - karis_max_speed/loop_frequency
            else:
                if self.position_y < self.position_1_y - 0.1:
                    self.position_y = self.position_y + karis_max_speed/loop_frequency
                elif self.position_y > self.position_1_y + 0.1:
                    self.position_y = self.position_y - karis_max_speed/loop_frequency
                else:
                    self.position_1_reached = True
                    self.state = 'waiting'
        else:
            pass

    # Fine positioning: wait for a predefined amount of time, then send position_2 reached command.
    def feinpos(self):
        self.feinpos.count = self.feinpos.count + 1
        if self.feinpos.count >= self.feinpos.time * loop_frequency:
            # Feinpositionierung beendet
            self.position_2_reached = True
            self.feinpos_pub.publish(1)

    def check_state(self):
        if self.state == 'driving':
            self.drive_to_position_1()
        elif self.state == 'waiting':
            pass
            #self.wait()
        elif self.state == 'feinpos':
            self.feinpos()
        elif self.state == 'hub':
            pass
            #self.hub()
        else:
            print(self.name + ' is in an unknown state: ' + self.state + '. Please kill it now.')




if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('name', help='name of simulated karis')
    parser.add_argument('initial_charge', help='initial battery charge in permille', type=int)
    parser.add_argument('start_x', help='x coordinate of starting position', type=float)
    parser.add_argument('start_y', help='y coordinate of starting position', type=float)
    args = parser.parse_args()
    agent = Karis(args.name, args.initial_charge, args.start_x, args.start_y)
    r = rospy.Rate(loop_frequency)
    while not rospy.is_shutdown():
        agent.check_state()
        agent.drain_battery()
        agent.publish_updates()
        print ('name: {0} \t state: {1} \t charge: {2} \t pos_x: {3} \t pos_y: {4}'.format(agent.name,
                                                                                           agent.state,
                                                                                           agent.battery_charge,
                                                                                           agent.position_x,
                                                                                           agent.position_y))
        r.sleep()