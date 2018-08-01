#!/usr/bin/env python
import rospy
import sys
import atexit
import rosbag
from datetime import datetime
from std_srvs.srv import Empty
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32MultiArray, Bool, String, Float32
import os
import subprocess
import signal
import psutil
import math
from openface_ros.msg import Faces
#from hlpr_feature_extraction.msg import PcFeatureArray
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
        #rospy.Subscriber("beliefs/features", PcFeatureArray, self.hlpr_callback)
        self.camera_info_sub = rospy.Subscriber("kinect/qhd/camera_info", CameraInfo, self.camera_info_callback)
        #self.yolo_sub = rospy.Subscriber("detector_node/detections", Detections, self.yolo_callback)
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

    # rotate vector v1 by quaternion q1 
    def qv_mult_ros(self, q1, v1):
      v1 = tf.transformations.unit_vector(v1)
      q2 = list(v1)
      q2.append(0.0)
      return tf.transformations.quaternion_multiply(
          tf.transformations.quaternion_multiply(q1, q2), 
          tf.transformations.quaternion_conjugate(q1)
          )[:3]

    def q_mult(self, q1, q2):
      #print('q1')
      #print(q1)
      w1, x1, y1, z1 = q1
      w2, x2, y2, z2 = q2
      w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
      x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
      y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
      z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
      return w, x, y, z

    def q_conjugate(self, q):
      w, x, y, z = q
      return (w, -x, -y, -z)

    def qv_mult(self, q1, v1):
      q2 = (0.0,) + v1
      return self.q_mult(self.q_mult(q1, q2), self.q_conjugate(q1))[1:]

    def normalize_q(self, v, tolerance=0.00001):
      mag2 = sum(n * n for n in v)
      if abs(mag2 - 1.0) > tolerance:
          mag = math.sqrt(mag2)
          v = tuple(n / mag for n in v)
      return v

    def axisangle_to_q(self, v, theta):
      v = self.normalize_q(v)
      x, y, z = v
      theta /= 2
      w = math.cos(theta)
      x = x * math.sin(theta)
      y = y * math.sin(theta)
      z = z * math.sin(theta)
      return w, x, y, z

    def q_to_axisangle(self, q):
      w, v = q[0], q[1:]
      theta = math.acos(w) * 2.0
      return self.normalize_q(v), theta

    def normalize(self,vector):
      norm_factor = math.sqrt(sum(map(lambda x:x*x,vector)))
      norm_vector = map(lambda x: x/norm_factor, vector)
      return norm_vector


    def openface_callback(self,msg):
      if len(msg.faces)>0:
        left, right = msg.faces[0].left_gaze,msg.faces[0].right_gaze 
        print('left, right: ',left,right)
        head_msg = msg.faces[0].head_pose.position
        print('head_msg: ', head_msg)
        head_orient = msg.faces[0].head_pose.orientation
        self.mutual_value = 0

        #(msg.faces[0].landmarks_3d[36])
        left_eye_corner1 = msg.faces[0].landmarks_3d[36] 
        left_eye_corner2 = msg.faces[0].landmarks_3d[39]
        left_eye_center = [(left_eye_corner1.x + left_eye_corner2.x)/2.0, (left_eye_corner1.y + left_eye_corner2.y)/2.0, (left_eye_corner1.z + left_eye_corner2.z)/2.0]
        left_eye_center = self.normalize(left_eye_center)

        right_eye_corner1 = msg.faces[0].landmarks_3d[42] 
        right_eye_corner2 = msg.faces[0].landmarks_3d[45]
        right_eye_center = [(right_eye_corner1.x + right_eye_corner2.x)/2.0, (right_eye_corner1.y + right_eye_corner2.y)/2.0, (right_eye_corner1.z + right_eye_corner2.z)/2.0]
        right_eye_center = self.normalize(right_eye_center)

        # normalized averaged eye gaze vector
        sum_gaze = [(left.x+right.x)/2.0, (left.y+right.y)/2.0, (left.z+right.z)/2.0]
        self.avg_gaze_norm = self.normalize(sum_gaze)
        self.avg_gaze = map(lambda x: x*1000, self.avg_gaze_norm) #####SCALING FACTOR

        #normalize head pose vector
        self.head_pose = [head_msg.x, head_msg.y, head_msg.z]
        head_pose_norm = self.normalize(self.head_pose)

        # normalized gaze vector with respect to camera 
        gaze_camera = [(head_pose_norm[0] + sum_gaze[0]), (head_pose_norm[1] + sum_gaze[1]), (head_pose_norm[2] + sum_gaze[2])]
        gaze_camera_norm = self.normalize(gaze_camera)

        # normalized head to camera vector
        head_camera = [head_pose_norm[0], head_pose_norm[1], head_pose_norm[2]]
        head_camera_norm = self.normalize(head_camera)
    
        # get left eye gaze vector oriented in camera frame 
        #rotation = self.q_conjugate((head_orient.w,head_orient.x,head_orient.y,head_orient.z))
        #rotation = (head_orient.w,-head_orient.x,-head_orient.y,-head_orient.z)
        #rotation = self.normalize_q(rotation)

        print('head_orientation: ',head_orient)
        v, theta = self.q_to_axisangle((head_orient.w,head_orient.x,head_orient.y,head_orient.z))
        print('quaternion axis, angle: ', v, math.degrees(theta))
        rotation = self.axisangle_to_q(v,-theta)

        left_camera_frame = self.qv_mult(rotation,(left.x,left.y,left.z))
        left_camera_frame = self.normalize(left_camera_frame)
        print('left eye camera frame: ', left_camera_frame)

        # get right eye gaze vector oriented in camera frame
        right_camera_frame = self.qv_mult(rotation,(right.x,right.y,right.z))
        right_camera_frame = self.normalize(right_camera_frame)
        print('right eye camera frame: ', right_camera_frame)

        # average and normalize eye vectors
        gaze_combined = [(left_camera_frame[0]+right_camera_frame[0])/2.0, \
                        (left_camera_frame[1]+right_camera_frame[1])/2.0, \
                        (left_camera_frame[2]+right_camera_frame[2])/2.0]
        gaze_camera_frame = self.normalize(gaze_combined)


        dot_product = (head_camera_norm[0]*gaze_camera_frame[0]) + (head_camera_norm[1]*gaze_camera_frame[1]) + (head_camera_norm[2]*gaze_camera_frame[2])
        #print(math.degrees(math.acos(dot_product)))

        dot_product_left = (left_eye_center[0]*left_camera_frame[0]) + (left_eye_center[1]*left_camera_frame[1]) + (left_eye_center[2]*left_camera_frame[2])
        #print(math.degrees(math.acos(dot_product_left)))
        dot_product_right = (right_eye_center[0]*right_camera_frame[0]) + (right_eye_center[1]*right_camera_frame[1]) + (right_eye_center[2]*right_camera_frame[2])
        #print(math.degrees(math.acos(dot_product_right)))
        avg_angle = (math.degrees(math.acos(dot_product_left)) + math.degrees(math.acos(dot_product_right)))/2.0
        print(45-avg_angle)

        if (45-avg_angle)<-20:
          self.mutual = True    # angle between head vector and gazze vector is less than 45 degrees
        else: 
          self.mutual = False

        print self.mutual
        self.mutual_value = avg_angle

        #print('debug')
        #print(self.head_pose)
        #print(self.avg_gaze)
        self.head_gaze_pose = np.add(self.head_pose, self.avg_gaze).tolist()
        self.find_nearest_object()
        self.publish_nearest_obj()

            
    def find_nearest_object(self):
      if not (self.camera_init_done or self.hlpr_objects or self.yolo_objects):
        return
      # scaling_factor = 1500
      # self.avg_gaze = map(lambda x: x*scaling_factor, self.avg_gaze)
      

    def publish_nearest_obj(self):
      #if not (self.hlpr_objects or self.yolo_objects or len(self.hlpr_objects)>0):
      #  return
      # min_center = self.find_closest_hlpr_cluster().bb_center
      # print min_center  # DEBUG
      # obj_x, obj_y = self.image_geo.project3dToPixel((min_center.x, min_center.y, min_center.z))
      #yolo_object = self.find_closest_yolo_obj()
      #print('debug2')
      #print(self.head_gaze_pose)
      gaze_x, gaze_y = self.image_geo.project3dToPixel(self.head_gaze_pose)
      head_x, head_y = self.image_geo.project3dToPixel(self.head_pose)
      prediction = [gaze_x, gaze_y, head_x, head_y]

      self.publish_object_bridge(None, prediction)
      
    def publish_object_bridge(self, yolo_obj, prediction):
      mutual_bool = Bool(data=self.mutual)
      #nearest_object_msg = String(data=yolo_obj.label)
      coordinate_array = Float32MultiArray(data=prediction)
      val = Float32(self.mutual_value)

      msg_topic = GazeTopic(nearest_object = None, mutual = mutual_bool, coordinates=coordinate_array, mutual_value=val)
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
