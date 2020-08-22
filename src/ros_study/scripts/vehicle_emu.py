#!/usr/bin/env python

import rospy, tf2_ros, threading, utm
from math import sqrt, sin, cos, pi
from numpy import random, rad2deg
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Vector3, Point, PoseStamped
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped, Quaternion
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path, Odometry
from tf.transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

GPS_ERR_V = [ -5,  -4,  -3,  -2,  -1,   0,   1,   2,   3,   4,   5]
#GPS_ERR_P = [.00, .00, .00, .00, .00, 1.0, .00, .00, .00, .00, .00]
#GPS_ERR_P = [.00, .00, .02, .03, .05, .80, .05, .03, .02, .00, .00]
GPS_ERR_P = [.01, .03, .05, .08, .13, .40, .13, .08, .05, .03, .01]

COVARIANCE_NO_ESTIMATION = [
    -1.0, 0.0,  0.0,
    0.0,  0.0,  0.0,
    0.0,  0.0,  0.0 ]

COVARIANCE_ORIENTATION_DEFAULT = [
    0.0,  0.0,  0.0,
    0.0,  0.0,  0.0,
    0.0,  0.0,  0.0 ]


def cmd_vel_cb(twist):

    global v_linear, v_angular

    with lock:
        v_linear = twist.linear.x
        v_angular = twist.angular.z


def update():

    global pos_x, pos_y, orientation, v_linear, v_angular

    rate = rospy.Rate(20)
    t0 = None

    while not rospy.is_shutdown():

        with lock:

            # Current time
            t = rospy.Time.now().to_sec()

            try:
                # Calculate elapsed time
                dt = t - t0

                # Calculate movement
                pos_x += v_linear * dt * cos(orientation)
                pos_y += v_linear * dt * sin(orientation)
                orientation += v_angular * dt

                # Limit rotation
                if orientation > pi:
                    orientation -= 2 * pi
                elif orientation < -pi:
                    orientation += 2 * pi

            except TypeError:
                pass
            finally:
                t0 = t

        rate.sleep()


def do_fix():

    global path, gps_err_x, gps_err_y

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():

        gps_err_x, gps_err_y = random.choice(GPS_ERR_V, p = GPS_ERR_P, size=(2))
        gps_err = sqrt(gps_err_x ** 2 + gps_err_y ** 2)

        with lock:
            now = rospy.Time.now()

            x, y = pos_x + gps_err_x, pos_y + gps_err_y

            m = NavSatFix()
            m.header.stamp = now
            m.header.frame_id = base_frame
            m.status.status = 0
            m.status.service = 1
            m.latitude, m.longitude = utm.to_latlon(x, y, 30, northern=True)
            m.altitude = 0
            m.position_covariance = [
                (2 * gps_err) ** 2, 0.0, 0.0,
                0.0, (2 * gps_err) ** 2, 0.0,
                0.0, 0.0, 10.0 ]
            m.position_covariance_type = 3

            n = PoseStamped()
            n.header.stamp = now
            n.header.frame_id = map_frame
            n.pose.position = Point(x - pos_x0, y - pos_y0, 0)
            q = quaternion_from_euler(0, 0, orientation)
            n.pose.orientation = Quaternion(*q)

            path.poses.append(n)
            path.header.stamp = now

        pub_fix.publish(m)
        pub_path.publish(path)
        rate.sleep()


def do_imu():

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        with lock:
            m = Imu()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = base_frame
            q = quaternion_from_euler(0, 0, orientation)
            m.orientation = Quaternion(*q)
            m.orientation_covariance = COVARIANCE_ORIENTATION_DEFAULT
            #m.angular_velocity = (0, 0, 0)
            m.angular_velocity_covariance = COVARIANCE_NO_ESTIMATION
            #m.linear_acceleration = (0, 0, 0)
            m.linear_acceleration_covariance = COVARIANCE_NO_ESTIMATION

            n = Vector3()
            n.x = rad2deg(orientation) + 90
            n.y = 0
            n.z = 0

        pub_imu.publish(m)
        rate.sleep()


def do_odom():

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        with lock:
            now = rospy.Time.now()

            mt = TwistWithCovarianceStamped()
            mt.header.stamp = now
            mt.header.frame_id = base_frame
            mt.twist.twist.linear.x = v_linear
            mt.twist.twist.angular.z = v_angular
            mt.twist.covariance = [
                0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                0.0,  0.0,  0.0, 0.0, 0.0, 0.0 ]

            if publish_odom:
                mo = Odometry()
                mo.header.stamp = now
                mo.header.frame_id = odom_frame
                mo.pose.pose.position.x = pos_x - pos_x0
                mo.pose.pose.position.y = pos_y - pos_y0
                q = quaternion_from_euler(0, 0, orientation)
                mo.pose.pose.orientation = Quaternion(*q)
                mo.child_frame_id = base_frame
                mo.twist.twist.linear.x = v_linear
                mo.twist.twist.angular.z = v_angular

            if publish_tf:

                t = TransformStamped()
                t.header.stamp = now
                t.header.frame_id = odom_frame
                t.child_frame_id = base_frame
                t.transform.translation.x = pos_x - pos_x0
                t.transform.translation.y = pos_y - pos_y0
                t.transform.rotation = Quaternion(*q)
                tfbr.sendTransform(t)

                t = TransformStamped()
                t.header.stamp = now
                t.header.frame_id = map_frame
                t.child_frame_id = odom_frame
                t.transform.translation.x = gps_err_x
                t.transform.translation.y = gps_err_y
                q = quaternion_from_euler(0, 0, 0)
                t.transform.rotation = Quaternion(*q)
                tfbr.sendTransform(t)


        pub_twist.publish(mt)
        if publish_odom:
            pub_odom.publish(mo)

        rate.sleep()


if __name__ == '__main__':

    # Starting position
    pos_x0, pos_y0 = 500000.0, 5000000.0
    pos_x, pos_y, orientation = pos_x0, pos_y0, 0.0
    v_linear, v_angular = 0.0, 0.0

    map_frame = 'map'
    odom_frame = 'odom'
    base_frame = 'base_link'

    rospy.init_node('vehicle_emu')

    # Get parameters
    publish_tf = rospy.get_param('~publish_tf', True)
    publish_odom = rospy.get_param('~publish_odom', True)

    # Transform broadcaster
    if publish_tf:
        tfbr = TransformBroadcaster()

    # Path
    path = Path()
    path.header.frame_id = map_frame
    path.header.stamp = rospy.Time.now()

    # Topics
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_cb)
    pub_fix = rospy.Publisher('fix', NavSatFix, queue_size=1)
    pub_path = rospy.Publisher('path', Path, queue_size=1, latch=True)
    pub_imu = rospy.Publisher('imu', Imu, queue_size=1)
    pub_twist = rospy.Publisher('twist', TwistWithCovarianceStamped, queue_size=1)
    if publish_odom:
        pub_odom = rospy.Publisher('odom', Odometry, queue_size=1)

    # Threads
    lock = threading.Lock()
    thr_update = threading.Thread(target=update)
    thr_fix = threading.Thread(target=do_fix)
    thr_imu = threading.Thread(target=do_imu)
    thr_odom = threading.Thread(target=do_odom)
    thr_update.start()
    thr_fix.start()
    thr_imu.start()
    thr_odom.start()

    # Loop
    rospy.spin()
