#!/usr/bin/env python3
#Rospython
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Imu
#Camera
import depthai

#get mono data simmilar to https://docs.luxonis.com/projects/api/en/latest/samples/MonoCamera/mono_preview/
def publish_infra_and_imu():
    rospy.init_node('oak_d_publisher',anonymous=True)

    mono1_pub=rospy.Publisher('/oak_d/mono1', Image, queue_size=10)
    mono2_pub=rospy.Publisher('/oak_d/mono2', Image, queue_size=10)
    imu_pub=rospy.Publisher('/oak_d/imu', Imu, queue_size=10)
    depth_pub=rospy.Publisher('/oak_d/depth',Image,queue_size=10)

    pipeline=depthai.Pipeline()
    mono_left=pipeline.create(depthai.node.MonoCamera)
    mono_right=pipeline.create(depthai.node.MonoCamera)
    stereo=pipeline.create(depthai.node.StereoDepth)

    xoutleft=pipeline.create(depthai.node.XLinkOut)
    xoutright=pipeline.create(depthai.node.XLinkOut)
    xoutstereo=pipeline.create(depthai.node.XLinkOut)

    xoutleft.setStreamName('left')
    xoutright.setStreamName('right')
    xoutstereo.setStreamName('depth')

    mono_left.setCamera("left")
    mono_left.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_720_P)
    mono_left.setFps(30)
    mono_right.setCamera("right")
    mono_right.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_720_P)
    mono_right.setFps(30)

    #todo: get correct connfigs
    stereo.setDefaultProfilePreset(depthai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setRectifyEdgeFillColor(0)

    mono_left.out.link(xoutleft.input)
    mono_right.out.link(xoutright.input)
    stereo.depth.link(xoutstereo.input)
    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)
    

    imu1= pipeline.create(depthai.node.IMU)
    xoutImu=pipeline.create(depthai.node.XLinkOut)
    xoutImu.setStreamName("imu")


    imu1.enableIMUSensor(depthai.IMUSensor.ACCELEROMETER_RAW,200)
    imu1.enableIMUSensor(depthai.IMUSensor.GYROSCOPE_RAW,200)
    
    imu1.setBatchReportThreshold(1)
    imu1.setMaxBatchReports(1)

    imu1.out.link(xoutImu.input)

    with depthai.Device(pipeline) as device:

        qleft = device.getOutputQueue(name="left", maxSize=4, blocking=False)
        qright = device.getOutputQueue(name="right", maxSize=4, blocking=False)
        qimu=device.getOutputQueue(name="imu",maxSize=10,blocking=False)
        qstereo=device.getOutputQueue(name="depth",maxSize=4, blocking=False)
        bridge_left=CvBridge()
        bridge_right=CvBridge()
        bridge_stereo=CvBridge()
        while not rospy.is_shutdown():
            #publish imu stream

            if qimu.has():
                imu_data=qimu.tryGet()
                imu_packets=imu_data.packets
                #todo Timestamps if batch report active
                #ros_t0=rospy.Time.now()
                #imu_t0=imu_packets[0].acceleroMeter.getTimestampDevice()
                
                for imu_packet in imu_packets:
                    accelValues=imu_packet.acceleroMeter
                    gyroValues=imu_packet.gyroscope
                    #print(accelValues.x)
                    #print(gyroValues.x)
                    imu_ros=Imu()
                    imu_ros.header.stamp=rospy.Time.now()
                    imu_ros.angular_velocity.x=gyroValues.x
                    imu_ros.angular_velocity.y=gyroValues.y
                    imu_ros.angular_velocity.z=gyroValues.z
                    imu_ros.linear_acceleration.x=accelValues.x
                    imu_ros.linear_acceleration.y=accelValues.y
                    imu_ros.linear_acceleration.z=accelValues.z
                    imu_pub.publish(imu_ros) 

            #publish image stream
            if qleft.has() and qright.has():
                left= qleft.tryGet()
                right=qright.tryGet()

                left_image=bridge_left.cv2_to_imgmsg(left.getCvFrame(),encoding="passthrough")
                left_image.header.stamp=rospy.Time.now()
                right_image=bridge_right.cv2_to_imgmsg(right.getCvFrame(),encoding="passthrough")
                #same timestamp is necessary for VIO
                right_image.header.stamp=left_image.header.stamp

                #publish
                mono1_pub.publish(left_image)
                mono2_pub.publish(right_image)
            if qstereo.has():
                raw_depth=qstereo.tryGet()
                stereo_depth=bridge_stereo.cv2_to_imgmsg(raw_depth.getCvFrame(),encoding="passthrough")
                stereo_depth.header.stamp=rospy.Time.now()
                depth_pub.publish(stereo_depth)



            
if __name__ == '__main__':
    try:
        publish_infra_and_imu()
    except rospy.ROSInterruptException:
        pass