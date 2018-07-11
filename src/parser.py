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
from parse_openface_ros.msg import Lines

class ParseOpenFace:
    def __init__(self):
        print "in init"

        self.image_geo = None
        rospy.Subscriber("faces", Faces, self.openface_callback)
        rospy.Subscriber("beliefs/features", PcFeatureArray, self.hlpr_callback)
        self.camera_info_sub = rospy.Subscriber("kinect/qhd/camera_info", CameraInfo, self.camera_info_callback)
        self.yolo_sub = rospy.Subscriber("detector_node/detections", Detections, self.yolo_callback)
        self.line_pub = rospy.Publisher("gaze_gui_line", Lines)
        self.obj_bridge_pub = rospy.Publisher("object_detector_bridge", GazeTopic)
        self.camera_init_done = False
        self.gaze_x, self.gaze_y, self.gaze_z = None, None, None
        self.hlpr_objects, self.yolo_objects = None, None
        self.mutual = False
        print "finished init"

    def camera_info_callback(self,msg):
      self.image_geo = PinholeCameraModel()
      self.image_geo.fromCameraInfo(msg)
      self.camera_init_done = True
      self.camera_info_sub.unregister()

    def hlpr_callback(self,msg):
      self.hlpr_objects = msg.objects

    def yolo_callback(self,msg):
      self.yolo_objects = msg.objects 

    def openface_callback(self,msg):
        if len(msg.faces)>0:
            left, right = msg.faces[0].left_gaze,msg.faces[0].right_gaze 

            sum_gaze = [(left.x+right.x), (left.y+right.y), (left.z+right.z)]
            norm_factor = math.sqrt(sum(map(lambda x:x*x,sum_gaze)))
            self.avg_gaze = map(lambda x: x/norm_factor, sum_gaze)

            self.avg_gaze = map(lambda x: x*1000, self.avg_gaze) #####SCALING FACTOR

            # print self.avg_gaze

            if(self.avg_gaze[0]>0.065):
              self.mutual = True
            else:
              self.mutual = False

            # print self.mutual

            head_msg = msg.faces[0].head_pose.position
            self.head_pose = [head_msg.x, head_msg.y, head_msg.z]
            self.head_gaze_pose = np.add(self.head_pose, self.avg_gaze).tolist()
            # print self.head_pose
            self.find_nearest_object()
            self.publish_nearest_obj()

            
    def find_nearest_object(self):
      if not (self.camera_init_done or self.hlpr_objects or self.yolo_objects):
        return
      # scaling_factor = 1500
      # self.avg_gaze = map(lambda x: x*scaling_factor, self.avg_gaze)
      

    def publish_nearest_obj(self):
      if not (self.hlpr_objects or self.yolo_objects or len(self.hlpr_objects)>0):
        return
      # min_center = self.find_closest_hlpr_cluster().bb_center
      # print min_center  # DEBUG
      # obj_x, obj_y = self.image_geo.project3dToPixel((min_center.x, min_center.y, min_center.z))
      yolo_object = self.find_closest_yolo_obj()

      gaze_x, gaze_y = self.image_geo.project3dToPixel(self.head_gaze_pose)
      head_x, head_y = self.image_geo.project3dToPixel(self.head_pose)
      prediction = [gaze_x, gaze_y, head_x, head_y]

      self.publish_object_bridge(yolo_object, prediction)
      
    def publish_object_bridge(self, yolo_obj, prediction):
      mutual_bool = Bool(data=self.mutual)
      nearest_object_msg = String(data=yolo_obj.label)
      coordinate_array = Float32MultiArray(data=prediction)

      msg_topic = GazeTopic(nearest_object = nearest_object_msg, mutual = mutual_bool, coordinates=coordinate_array)
      self.obj_bridge_pub.publish(msg_topic)
      # print yolo_obj.label  # DEBUG

    # def find_closest_hlpr_cluster(self):
    #   min_diff = float('inf')
    #   min_item = None
    #   for item in self.hlpr_objects:  # Find closest hlpr object to gaze vector
    #     center = np.array([item.bb_center.x,item.bb_center.y,item.bb_center.z])
    #     l1 = np.array(self.head_pose)
    #     l2 = center
    #     p = np.array(self.avg_gaze)
    #     # diff = np.linalg.norm(np.cross(l2-l1, p))/np.linalg.norm(p)
    #     diff = np.linalg.norm(np.cross(l2-l1, p-l1))/np.linalg.norm(l2-l1)
    #     # diff = (self.avg_gaze[0] - center[0])**2 + (self.avg_gaze[1] - center[1])**2 + (self.avg_gaze[2] - center[2])**2
    #     if diff<min_diff:
    #       min_diff = diff
    #       min_item = item
    #       min_l1, min_l2, min_p = l1, l2, p

    #   head_x, head_y = self.image_geo.project3dToPixel(self.head_pose)
    #   print "min l2 {0}".format(min_l2)
    #   print "min l1 {0}".format(min_l1)
    #   print "min diff {0}".format(min_l2-min_l1)
    #   l1_x, l1_y = self.image_geo.project3dToPixel(min_l2-min_l1)  
    #   line1 = Float32MultiArray(data=[head_x,head_y,l1_x+head_x, l1_y+head_y])
    #   l2_x, l2_y = self.image_geo.project3dToPixel(min_l1 - 1000*min_p)
    #   line2 = Float32MultiArray(data=[head_x,head_y,l2_x, l2_y])
    #   self.line_pub.publish([line1, line2])
      
    #   return min_item

    def find_closest_yolo_obj(self):
      min_diff = float('inf')
      yolo_obj = None

      head_x, head_y = self.image_geo.project3dToPixel(self.head_pose)
      head_gaze_x, head_gaze_y = self.image_geo.project3dToPixel(self.head_gaze_pose)
      for yolo_object in self.yolo_objects:
        if yolo_object.label != "spoon":
          head = np.array([head_x, head_y])
          head_gaze = np.array([head_gaze_x, head_gaze_y])
          obj = np.array([yolo_object.centroid_x, yolo_object.centroid_y])
        
          # diff = np.linalg.norm(np.cross(l2-l1, p))/np.linalg.norm(p)
          # diff = np.linalg.norm(np.cross(l2-l1, l1-p))/np.linalg.norm(l2-l1)
          diff = np.linalg.norm(np.cross(head_gaze-head, head-obj))/np.linalg.norm(head_gaze-head)
          # diff = (self.avg_gaze[0] - center[0])**2 + (self.avg_gaze[1] - center[1])**2 + (self.avg_gaze[2] - center[2])**2

          if diff<min_diff:
            min_diff = diff
            yolo_obj = yolo_object
            # min_l1, min_l2, min_p = l1, l2, p

      # print "min l2 {0}".format(min_l2)
      # print "min l1 {0}".format(min_l1)
      # print "min diff {0}".format(min_l2-min_l1)
      line1 = Float32MultiArray(data=[0,0,0, 0])
      line2 = Float32MultiArray(data=[head_x,head_y,head_x+ 100*(head_gaze_x-head_x), head_y+ 100*(head_gaze_y-head_y)])
      self.line_pub.publish([line1, line2])

      return yolo_obj

def main():
  obj = ParseOpenFace()


if __name__ == '__main__':
  rospy.init_node('parse_openface_ros', anonymous=True)
  main()
  while not rospy.is_shutdown():
    rospy.spin()
