#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np 
from sensor_msgs.msg import Imu

def main():
    rospy.init_node('realsense_publisher', anonymous=True)
    bridge = CvBridge()

    # Configure RealSense camera
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.accel)
    config.enable_stream(rs.stream.gyro)

    # Start streaming
    pipeline.start(config)

    # Create ROS publishers
    depth_pub = rospy.Publisher('/camera/depth/image_raw', Image, queue_size=10)
    color_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
    camera_info_pub = rospy.Publisher('/camera/color/camera_info', CameraInfo, queue_size=10)
    imu_pub = rospy.Publisher('/camera/imu', Imu, queue_size=10)
    rate = rospy.Rate(30)  # Publish at 30 Hz

    while not rospy.is_shutdown():
        # Wait for the next set of frames
        frames = pipeline.wait_for_frames()
        time=rospy.Time.now()
        # Get depth and color frames
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        # Convert frames to ROS messages
        depth_image = bridge.cv2_to_imgmsg(np.array(depth_frame.get_data()), encoding="passthrough")
        color_image = bridge.cv2_to_imgmsg(np.array(color_frame.get_data()), encoding="bgr8")
        depth_image.header.stamp=time
        depth_image.header.frame_id="camera_depth_frame"
        color_image.header.frame_id="camera_color_frame"
        color_image.encoding="bgr8"
        color_image.header.stamp=time
        # Get IMU frames
        accel_frame = frames.first_or_default(rs.stream.accel)
        gyro_frame = frames.first_or_default(rs.stream.gyro)


        # Create CameraInfo message
        camera_info = CameraInfo()
        camera_info.header.stamp =time

        camera_info.width = color_frame.width
        camera_info.height = color_frame.height

        accel_data = accel_frame.as_motion_frame().get_motion_data()
        gyro_data = gyro_frame.as_motion_frame().get_motion_data()

        # # Create Imu message
        imu_msg = Imu()
        imu_msg.header.stamp =time
        imu_msg.header.frame_id="camera_imu_optical_frame"
        imu_msg.angular_velocity.x = gyro_data.x
        imu_msg.angular_velocity.y = gyro_data.y
        imu_msg.angular_velocity.z = gyro_data.z
        imu_msg.linear_acceleration.x = accel_data.x
        imu_msg.linear_acceleration.y = accel_data.y
        imu_msg.linear_acceleration.z = accel_data.z

        # # Publish message
        imu_pub.publish(imu_msg)
        # color_pub.header.timestamp = rospy.r
        # Publish messages
        depth_pub.publish(depth_image)
        color_pub.publish(color_image)
        camera_info_pub.publish(camera_info)

        rate.sleep()

    # Stop streaming
    pipeline.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
