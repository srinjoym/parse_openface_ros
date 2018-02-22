#!/usr/bin/env python
import rospy
import sys
import atexit
import rosbag
from datetime import datetime
from std_srvs.srv import Empty
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32MultiArray, Bool, String
import os
import subprocess
import signal
import psutil
import math
from openface_ros.msg import Faces
from hlpr_feature_extraction.msg import PcFeatureArray
from image_geometry import PinholeCameraModel
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, Vector3Stamped
from rail_object_detector.msg import Detections
import tf
import numpy as np
from object_detector.msg import GazeTopic

class ParseOpenFace:
    def __init__(self):
        print "in init"

        self.image_geo = None
        rospy.Subscriber("faces", Faces, self.callback)
        rospy.Subscriber("beliefs/features", PcFeatureArray, self.hlpr_callback)
        self.camera_info_sub = rospy.Subscriber("kinect/qhd/camera_info", CameraInfo, self.camera_info_callback)
        self.yolo_sub = rospy.Subscriber("detector_node/detections", Detections, self.yolo_callback)
        self.pub = rospy.Publisher("openface_gaze", Float32MultiArray)
        self.obj_bridge_pub = rospy.Publisher("object_detector_bridge", GazeTopic)
        self.camera_init_done = False
        self.gaze_x, self.gaze_y, self.gaze_z = None, None, None
        self.objects = None
        self.mutual = False
        print "finished init"

    def camera_info_callback(self,msg):
      self.image_geo = PinholeCameraModel()
      self.image_geo.fromCameraInfo(msg)
      self.camera_init_done = True
      self.camera_info_sub.unregister()

    def hlpr_callback(self,msg):
      objects = msg.objects
      min_diff = 100000000000000
      min_item = None

      if len(objects)<1:
        return

      for item in objects:
        center = item.bb_center
        l1 = np.array([self.head_pose.x, self.head_pose.y, self.head_pose.z])
        l2 = np.array([center.x, center.y, center.z ])
        p = self.average
        diff = np.linalg.norm(np.cross(l2-l1, l1-p))/np.linalg.norm(l2-l1)

        # diff += (self.gaze_x - center.x)**2 + (self.gaze_y - center.y)**2 + (self.gaze_z - center.z)**2
        if diff<min_diff:
          min_diff = diff
          min_item = item

      min_center = min_item.bb_center
      x, y = self.image_geo.project3dToPixel((min_center.x, min_center.y, min_center.z))
      min_diff = 100000000000000
      yolo_obj = None
      if not self.objects:
        print "No Yolo Callback"
        return

      for yolo_object in self.objects:
        diff = 0
        diff += (yolo_object.centroid_x-x)**2 + (yolo_object.centroid_y-y)**2
        if diff<min_diff:
          min_diff = diff
          yolo_obj = yolo_object

      mutual_bool = Bool(data=self.mutual)
      nearest_object_msg = String(data=yolo_obj.label)

      msg_topic = GazeTopic(nearest_object = nearest_object_msg, mutual = mutual_bool )
      self.obj_bridge_pub.publish(msg_topic)


    def yolo_callback(self,msg):
      self.objects = msg.objects 

    def callback(self,msg):
        # print msg

        if not self.camera_init_done:
          return

        if len(msg.faces)>0:
            scaling_factor = 700

            left, right = msg.faces[0].left_gaze,msg.faces[0].right_gaze 

            self.average = [(left.x+right.x), (left.y+right.y), (left.z+right.z)]
            if(self.average[0]<0.01 and self.average[1]<0.01):
              self.mutual = True

            self.average = map(lambda x: x*scaling_factor, self.average)
            self.head_pose = msg.faces[0].head_pose.position

            self.gaze_x, self.gaze_y, self.gaze_z = self.head_pose.x+self.average[0], self.head_pose.y+self.average[1], self.head_pose.z+self.average[2]

            x, y = self.image_geo.project3dToPixel((self.gaze_x, self.gaze_y, self.gaze_z))
            msg = Float32MultiArray(data=(x,y))
            self.pub.publish(msg)





def main():
  obj = ParseOpenFace()


if __name__ == '__main__':
  rospy.init_node('parse_openface_ros', anonymous=True)
  main()
  while not rospy.is_shutdown():
    rospy.spin()
