import numpy as np
import math


class UnitTest:
    def __init__(self, pos, quat, gaze):
        print "in init"
        self.pos = pos
        self.quat = quat
        self.gaze = gaze

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

    def normalize(vector):
    	norm_factor = math.sqrt(sum(map(lambda x:x*x,vector)))
        norm_vector = map(lambda x: x/norm_factor, vector)
    	return norm_vector

	def head_camera_vector(self):	
		#normalize head pose vector
        #self.head_pose = [head_msg.x, head_msg.y, head_msg.z]
        #norm_factor = math.sqrt(sum(map(lambda x:x*x,self.head_pose)))
        #head_pose_norm = map(lambda x: x/norm_factor, self.head_pose)
        head_pose_norm = normalize(self.head_pose)

        # normalized head to camera vector
        head_camera = [-head_pose_norm[0], -head_pose_norm[1], -head_pose_norm[2]]
        #norm_factor = math.sqrt(sum(map(lambda x:x*x,head_camera)))
        #head_camera_norm = map(lambda x: x/norm_factor, head_camera)
        head_camera_norm = normalize(head_camera)
        return head_camera_norm

    def gaze_in_camera_frame(self):
    	#rotation = (-head_orient.w,head_orient.x,head_orient.y,head_orient.z)
    	rotation = (-self.quat[0], self.quat[1], self.quat[2], self.quat[3])
	    gaze_camera_frame = self.qv_mult(rotation,(self.gaze[0],self.gaze[1],left.zself.gaze[2]))
	    return gaze_camera_frame

	def get_mutual_gaze():
		head_camera_norm = self.head_camera_vector(self.pos)
		gaze_camera_frame = self.gaze_in_camera_frame(self.gaze)
		dot_product = (head_camera_norm[0]*gaze_camera_frame[0]) + (head_camera_norm[1]*gaze_camera_frame[1]) + (head_camera_norm[2]*gaze_camera_frame[2])
        print(math.degrees(math.acos(dot_product)))


if __name__ == '__main__':
	
	head_pos = [0,0,1]
	head_quat = [0,0,0,1] #rotate 180 degrees by z axis
	head_gaze = [0,0,1]	# in the head frame

	test = UnitTest(head_pos, head_quat, head_gaze)
	test.get_mutual_gaze()
