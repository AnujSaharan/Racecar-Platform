#!/usr/bin/env python
import rospy
import sys, select, termios, tty
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class Laser():

    def __init__(self):
        rospy.init_node('laser_navigate', anonymous=True)
        self.sub1 = rospy.Subscriber('/scan', LaserScan, self.laser_info)
        self.pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped,queue_size=10)
        self.rate = rospy.Rate(10)
        rospy.loginfo(" laser node, Press Ctrl+C to Exit")

        self.scan_flag=False
        self.reverse=False

        self.filtered_alpha=0.8
        self.flitered_speed=0
        self.filtered_steering=0

        while not rospy.is_shutdown():
            pass


    def laser_info(self, scan):
        # print 'scan len:',len(scan.ranges)
        self.scan_flag=True
        data_arr1=[]
        for i in range(len(scan.ranges)):
            if (scan.ranges[i]>scan.range_min) and (scan.ranges[i]<scan.range_max):
                ang= scan.angle_min+i*scan.angle_increment
                if (ang>-3.14/2.) and (ang<3.14/2):
                    data_arr1.append([scan.ranges[i],(scan.angle_min+i*scan.angle_increment)*180/3.14])

        if len(data_arr1):
            data_arr1.sort()
            data_arr2=[]
            for i in range(len(data_arr1)):
                if abs(data_arr1[0][1]-data_arr1[i][1])<10:
                    data_arr2.append(data_arr1[i])
            # print data_arr2
            dist_arr=[x[0] for x in data_arr2]
            th_arr=[x[1] for x in data_arr2]
            min_dist=sum(dist_arr)/float(len(dist_arr))
            min_th=sum(th_arr)/float(len(th_arr))
            self.min_obs=[min_dist,min_th]
        else:
            self.min_obs=[1,1,1]
        print 'min_obs:',self.min_obs
        self.scan_ranges=scan.ranges
        # self.rate.sleep()
        self.navigate()
        # print 'min dist,angle:',self.min_obs



    def navigate(self):
        msg=AckermannDriveStamped();
        msg.header.stamp=rospy.Time.now();
        msg.header.frame_id="base_link";

        max_turn=0.5

        if self.scan_flag:
            self.scan_flag=False
            throttle_dir=0
            steer_dir=0

            # print 'min_obs,reverse:',[self.min_obs,self.reverse]
            if (self.min_obs[0]>0.6) and  (not self.reverse):
                max_speed=1
                throttle_dir=1
                steer_dir=0
                print 'forward no steer'
            if (self.min_obs[0]<0.6) and (self.min_obs[0]>0.15) and  (not self.reverse):
                max_speed=0.5
                throttle_dir=1
                steer_dir=-self.min_obs[1]/abs(self.min_obs[1])
                print 'forward steer'
            if (self.min_obs[0]<0.2) and (self.min_obs[1]>-45) and (self.min_obs[1]<45) and not self.reverse:
                self.reverse=True
                # steer_dir=self.min_obs[1]/abs(self.min_obs[1])
                # steer_dir=-1
                print '********* going to reverse mode ***********'
            if self.reverse:
                print 'reverse'
                max_speed=0.5
                throttle_dir=-1
                steer_dir=-1
                scan_mid=int(len(self.scan_ranges)/2)
                avg_len=5
                rev_stop_dist=sum(self.scan_ranges[scan_mid-avg_len:scan_mid+avg_len])/(2.*avg_len)
                if rev_stop_dist>1.5:
                    self.reverse=False

            self.filtered_steering = self.filtered_alpha*self.filtered_steering + (1-self.filtered_alpha)*steer_dir*max_turn
            self.flitered_speed = self.filtered_alpha*self.flitered_speed + (1-self.filtered_alpha)*throttle_dir*max_speed

            msg.drive.speed=self.flitered_speed
            msg.drive.acceleration=1
            msg.drive.jerk=1
            msg.drive.steering_angle=self.filtered_steering
            msg.drive.steering_angle_velocity=1

        else:
            throttle_dir=1
            steer_dir=0
            msg.drive.speed=self.filtered_speed
            msg.drive.acceleration=1
            msg.drive.jerk=1
            msg.drive.steering_angle=self.filtered_steering
            msg.drive.steering_angle_velocity=1


        self.pub.publish(msg)
        # self.rate.sleep()

if __name__ == '__main__':

    laser = Laser()
