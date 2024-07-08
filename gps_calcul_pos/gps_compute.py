import rclpy
from rclpy.node import Node
from pyproj import Transformer
from std_msgs.msg import Float32MultiArray
import numpy as np


class Gps_compute(Node):

    def __init__(self):
        super().__init__('gps_compute')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'gps_value',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):

        #convertion2949 = Transformer.from_crs("EPSG:4326", "EPSG:2949", always_xy=True)

        x = [0, 0 ,0]
        y = [0, 0 ,0]
        z = [0, 0 ,0]

        gps_values = list(msg.data)

        # x[0], y[0] = convertion2949.transform(gps_values[0], gps_values[1])
        # x[1], y[1] = convertion2949.transform(gps_values[3], gps_values[4])
        # x[2], y[2] = convertion2949.transform(gps_values[6], gps_values[7])

        convertion2955 = Transformer.from_crs("EPSG:4326", "EPSG:2955", always_xy=True)

        x[0], y[0], z[0] = convertion2955.transform(gps_values[0], gps_values[1], gps_values[2])
        x[1], y[1], z[1] = convertion2955.transform(gps_values[3], gps_values[4], gps_values[5])
        x[2], y[2], z[2] = convertion2955.transform(gps_values[6], gps_values[7], gps_values[8])

        def minimization(P, Q):
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
        
       #P est une matrice qui provient de valeur mesure au lab 
        P = np.array([[2.980048, -0.249529, -0.680838], [3.753882, -0.211299, -0.584590], [3.611486,  0.286080, -0.561145], [1,1,1]])
        Q = np.array([x , y , z, [1,1,1]])

        try:
            print(minimization(P,Q))

        except:
            self.get_logger().info("bad gps signal")

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