import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from pyproj import Transformer
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from gpsx.msg import Gpsx


class Gps_compute(Node):
    def __init__(self):
        super().__init__('gps_compute')
        input_topic = self.declare_parameter('input_topic', '/gps_value').value
        output_topic = self.declare_parameter('output_topic', '/warthog_gps_pose').value

        self.subscription1 = self.create_subscription(
            Gpsx,
            "topic_emlid_E845",
            self.emlid_E845_callback,
            10)

        self.subscription2 = self.create_subscription(
            Gpsx,
            "topic_emlid_6802",
            self.emlid_6802_callback,
            10)

        self.subscription3 = self.create_subscription(
            Gpsx,
            "topic_emlid_C959",
            self.emlid_C959_callback,
            10)

        self.subscription1  # prevent unused variable warning
        self.subscription2  # prevent unused variable warning
        self.subscription3  # prevent unused variable warning

        self.tf_broadcaster = TransformBroadcaster(self)
        self.publisher = self.create_publisher(
            PoseStamped,
            output_topic,
            10)
        tf_sub = self.create_subscription(
            TFMessage,
            "/tf",
            self.tf_callback,
            10
        )
        self.Q1, self.Q2, self.Q3 = None, None, None
        self.tf_acquired = False
        self.timer = self.create_timer(0.2, self.timer_callback)

    def emlid_E845_callback(self,msg):
        self.emlid_E845 = msg

    def emlid_6802_callback(self,msg):
        self.emlid_6802 = msg

    def emlid_C959_callback(self,msg):
        self.emlid_C959 = msg

    def timer_callback(self):
        #convertion2949 = Transformer.from_crs("EPSG:4326", "EPSG:2949", always_xy=True)
        # x[0], y[0] = convertion2949.transform(gps_values[0], gps_values[1])
        # x[1], y[1] = convertion2949.transform(gps_values[3], gps_values[4])
        # x[2], y[2] = convertion2949.transform(gps_values[6], gps_values[7])
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'world'
        

        #TODO: find a way to get the timestamp from the gps message
        # self.pose.header.stamp.sec = msg.header.stamp.sec
        # self.pose.header.stamp.nanosec = msg.header.stamp.nanosec

        x = [0, 0, 0]
        y = [0, 0, 0]
        z = [0, 0, 0]
        convertion2955 = Transformer.from_crs("EPSG:4326", "EPSG:2955", always_xy=True)

        try:
            gps_emlid = [self.emlid_E845, self.emlid_6802, self.emlid_C959] 
            gps_values = []
            for i in gps_emlid:
                gps_values.append(i.longitude)
                gps_values.append(i.latitude)
                gps_values.append(i.altitude)
            
            for i in range(3):
                x[i], y[i], z[i] = convertion2955.transform(gps_values[i*3], gps_values[i*3+1], gps_values[i*3+2])
            position = np.array([x, y, z])
            # self.get_logger().info(f"Received GPS values: {position}")
            self.compute_calibration(position)
            # self.get_logger().info(f"Calibration computed!")
            self.publish_transform(self.T_world_to_base_link)
            self.publish_pose(self.T_world_to_base_link)
            # self.get_logger().info(f"Transofrmation matrix: {self.T_world_to_base_link}")

            self.get_logger().info("Publishing calibration transform...")
            self.publish_transform(self.T_world_to_base_link)

        except:
            self.get_logger().info('-------------------')
            self.get_logger().info('no info from a gps')
            self.get_logger().info('-------------------')

    def tf_callback(self, tf_msg):
        for tf in tf_msg.transforms:
            if tf.child_frame_id == "EmlidE845" and tf.header.frame_id == "base_link":
                self.Q1 = np.array([[tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]])
                self.get_logger().info(f"EmlidE845 transformation acquired: {self.P[0]}")
            if tf.child_frame_id == "Emlid6802" and tf.header.frame_id == "base_link":
                self.Q2 = np.array([[tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]])
                self.get_logger().info(f"Emlid6802 transformation acquired: {self.P[1]}")
            if tf.child_frame_id == "EmlidC959" and tf.header.frame_id == "base_link":
                self.Q3 = np.array([[tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]])
                self.get_logger().info(f"EmlidC959 transformation acquired: {self.P[2]}")
        if self.Q1 is not None and self.Q2 is not None and self.Q3 is not None:
            self.tf_acquired = True
            self.destroy_subscription(self.tf_sub)
    
    def compute_calibration(self, P):
        while self.tf_acquired == False:
            self.get_logger().info("Waiting for tf transform...")
            time.sleep(0.5)
        Q = np.array([self.Q1, self.Q2, self.Q3]).T
        Q = np.vstack((Q, np.ones((1, Q.shape[1]))))  
        
        Q = np.array([[2.980048, -0.249529, -0.680838], [3.753882, -0.211299, -0.584590], [3.611486,  0.286080, -0.561145], [1,1,1]]) #Q est une matrice qui provient de valeur mesure au lab 
        P = np.vstack((P, np.ones((1, P.shape[1]))))#P est une matrice qui provient de valeur mesuré par le robot 
        if P is not None and Q is not None:
            self.T_world_to_base_link = self.minimization(P, Q)
            # self.get_logger().info(f"GPS values: {P}")
        else:
            self.get_logger().info("Bad gps signal!")
                   
    def minimization(self, P, Q):
        errors_before = Q - P
        mu_p = np.mean(P[0:3, :], axis=1)
        mu_q = np.mean(Q[0:3, :], axis=1)
        P_mu = np.ones((3, P.shape[1]))
        Q_mu = np.ones((3, Q.shape[1]))
        for i in range(0, P_mu.shape[1]):
            P_mu[0:3, i] = P[0:3, i] - mu_p
        for i in range(0, Q_mu.shape[1]):
            Q_mu[0:3, i] = Q[0:3, i] - mu_q
        H = P_mu @ Q_mu.T
        U, s, V = np.linalg.svd(H)
        M = np.eye(3)
        M[2, 2] = np.linalg.det(V.T @ U.T)
        R = V.T @ M @ U.T
        t = mu_q - R @ mu_p
        T = np.eye(4)
        T[0:3, 0:3] = R
        T[0:3, 3] = t
        return T
    
    def publish_transform(self, T):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'world'
        t.transform.translation.x = T[0, 3]
        t.transform.translation.y = T[1, 3]
        t.transform.translation.z = T[2, 3]
        q = np.zeros(4)
        q[0] = np.sqrt(1 + T[0, 0] + T[1, 1] + T[2, 2]) / 2
        q[1] = (T[2, 1] - T[1, 2]) / (4 * q[0])
        q[2] = (T[0, 2] - T[2, 0]) / (4 * q[0])
        q[3] = (T[1, 0] - T[0, 1]) / (4 * q[0])
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]
        self.tf_broadcaster.sendTransform(t)
    
    
    def publish_pose(self, T):
        position = T[0:3, 3]
        self.pose.pose.position.x = position[0]
        self.pose.pose.position.y = position[1]
        self.pose.pose.position.z = position[2]
        self.get_logger().info(f"Publishing Pose:, Time: {self.pose.header.stamp.sec}.{self.pose.header.stamp.nanosec:.4f}, X: {self.pose.pose.position.x:.4f}, Y: {self.pose.pose.position.y:.4f}, Z: {self.pose.pose.position.z:.4f}")
        self.publisher.publish(self.pose)

def main(args=None):
    rclpy.init(args=args)
    gps_compute = Gps_compute()
    rclpy.spin(gps_compute)
    gps_compute.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()