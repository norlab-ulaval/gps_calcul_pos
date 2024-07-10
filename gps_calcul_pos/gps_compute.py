import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from pyproj import Transformer
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


class Gps_compute(Node):

    def __init__(self):
        super().__init__('gps_compute')
        input_topic = self.declare_parameter('input_topic', '/gps_value').value
        output_topic = self.declare_parameter('output_topic', '/warthog_gps_pose').value
        self.subscription = self.create_subscription(
            Float32MultiArray,
            input_topic,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
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

    def listener_callback(self, msg):
        #convertion2949 = Transformer.from_crs("EPSG:4326", "EPSG:2949", always_xy=True)
        # x[0], y[0] = convertion2949.transform(gps_values[0], gps_values[1])
        # x[1], y[1] = convertion2949.transform(gps_values[3], gps_values[4])
        # x[2], y[2] = convertion2949.transform(gps_values[6], gps_values[7])
        x = [0, 0 ,0]
        y = [0, 0 ,0]
        z = [0, 0 ,0]
        gps_values = list(msg.data)
        convertion2955 = Transformer.from_crs("EPSG:4326", "EPSG:2955", always_xy=True)

        x[0], y[0], z[0] = convertion2955.transform(gps_values[0], gps_values[1], gps_values[2])
        x[1], y[1], z[1] = convertion2955.transform(gps_values[3], gps_values[4], gps_values[5])
        x[2], y[2], z[2] = convertion2955.transform(gps_values[6], gps_values[7], gps_values[8])
        position = np.array([x, y, z])
        self.compute_calibration(position)
        if self.compute_calibration:
            self.publish_pose(position)
            
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
        # while self.tf_acquired == False:
        #     self.get_logger().info("Waiting for tf transform...")
        #     time.sleep(0.5)
        P = np.array([position, [1,1,1]])#P est une matrice qui provient de valeur mesuré par le robot 
        Q = np.array([[2.980048, -0.249529, -0.680838], [3.753882, -0.211299, -0.584590], [3.611486,  0.286080, -0.561145], [1,1,1]]) #Q est une matrice qui provient de valeur mesure au lab 
        # Q = np.array([self.Q1, self.Q2, self.Q3]).T
        # Q = np.vstack((Q, np.ones((1, Q.shape[1]))))

        if P is not None and Q is not None:
            self.T_world_to_base_link = self.minimization(P, Q)
            self.get_logger().info(f"calibration computed: {self.T_world_to_base_link}")
        else:
            self.get_logger().info("bad gps signal")
                   
    def minimization(self, P, Q):
        errors_before = Q - P # Errors at the beginning
        mu_p = np.mean(P[0:3, :], axis=1) #Centroide of each pointcloud
        mu_q = np.mean(Q[0:3, :], axis=1)
        P_mu = np.ones((3, P.shape[1])) # Centered each pointclouds
        Q_mu = np.ones((3, Q.shape[1]))
        for i in range(0, P_mu.shape[1]):
            P_mu[0:3, i] = P[0:3, i] - mu_p
        for i in range(0, Q_mu.shape[1]):
            Q_mu[0:3, i] = Q[0:3, i] - mu_q
        H = P_mu @ Q_mu.T # Cross covariance matrix
        U, s, V = np.linalg.svd(H) # Use SVD decomposition
        M = np.eye(3) # Compute rotation
        M[2, 2] = np.linalg.det(V.T @ U.T)
        R = V.T @ M @ U.T
        t = mu_q - R @ mu_p # Compute translation
        T = np.eye(4) # Transformation matrix
        T[0:3, 0:3] = R
        T[0:3, 3] = t
        return T
    
    def publish_transform(self, T):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'theodolite'
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
    
    def timer_callback(self):
        self.get_logger().info("Publishing calibration transform...")
        self.publish_transform(self.T_world_to_base_link)
    
    def publish_pose(self, position):
        position = np.array([position[0], position[1], position[2], 1])
        self.pose.pose.position.x = position[0]
        self.pose.pose.position.y = position[1]
        self.pose.pose.position.z = position[2]
        self.get_logger().info(f"Publishing Pose:, Time: {self.pose.header.stamp.sec}.{self.pose.header.stamp.nanosec:.4f}, X: {self.pose.pose.position.x:.4f}, Y: {self.pose.pose.position.y:.4f}, Z: {self.pose.pose.position.z:.4f}")
        self.publisher.publish(self.pose)

def main(args=None):
    rclpy.init(args=args)
    gps_compute = Gps_compute()
    rclpy.spin(gps_compute)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_compute.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()