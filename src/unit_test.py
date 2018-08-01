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

    def head_camera_vector(self):
        head_pose_norm = self.normalize(self.pos)

        # normalized head to camera vector
        head_camera = [-head_pose_norm[0], -head_pose_norm[1], -head_pose_norm[2]]
        head_camera_norm = self.normalize(head_camera)
        return head_camera_norm

    def gaze_in_camera_frame(self):
        #rotation = (-head_orient.w,head_orient.x,head_orient.y,head_orient.z)
        #self.quat = self.normalize_q(self.quat)
        #rotation = (self.quat[0], -self.quat[1], -self.quat[2], -self.quat[3])
        v, theta = self.q_to_axisangle(self.quat)
        rotation = self.axisangle_to_q(v,-theta)
        #print('rotation')
        #print(self.quat)
        #print(rotation)
        gaze_camera_frame = self.qv_mult(rotation,(self.gaze[0],self.gaze[1],self.gaze[2]))
        gaze_camera_frame = self.normalize(gaze_camera_frame)
        return gaze_camera_frame

    def get_mutual_gaze(self):
        head_camera_norm = self.head_camera_vector()
        gaze_camera_frame = self.gaze_in_camera_frame()
        print(head_camera_norm)
        print(gaze_camera_frame)
        dot_product = (head_camera_norm[0]*gaze_camera_frame[0]) + (head_camera_norm[1]*gaze_camera_frame[1]) + (head_camera_norm[2]*gaze_camera_frame[2])
        print(math.degrees(math.acos(dot_product)))


if __name__ == '__main__':
    
    head_pos = [0,0,1]
    head_quat = [0,0,1,0] #rotate 180 degrees by z axis
    head_gaze = [0,0,1] # in the head frame
    print('test1')
    test1 = UnitTest(head_pos, head_quat, head_gaze)
    test1.get_mutual_gaze()


    head_pos = [0,1,1]
    head_quat = [0,0,1,0] #rotate 180 degrees along y axis
    head_gaze = [0,0,1] # in the head frame
    print('\ntest2')
    test2 = UnitTest(head_pos, head_quat, head_gaze)
    test2.get_mutual_gaze()

    head_45 = test2.axisangle_to_q((1,0,0),-np.pi/4)
    head_45_inv = test2.q_mult(head_45,[0,0,1,0])
    rot_vec = test2.qv_mult(head_45_inv,(0,0,1))
    print('\nrot_vec')
    print(rot_vec)
    # head rotated 45 degrees down
    head_quat = [head_45_inv[0], head_45_inv[1], head_45_inv[2], head_45_inv[3]]

    print('\ntest3')
    test3 = UnitTest(head_pos, head_quat, head_gaze)
    test3.get_mutual_gaze()
