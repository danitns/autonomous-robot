#!/home/danitns/Documents/autonomous-robot/pi-env/bin/python3
import pigpio
import time
import struct 

# Import ROS2 libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
from geometry_msgs.msg import Vector3

I2C_ADDR = 0x14  # STM32 I2C address
BUS = 1  

pi = pigpio.pi()
handle = pi.i2c_open(BUS, I2C_ADDR)


class LSM9DS1Node(Node):
    def __init__(self):
        super().__init__("lsm9ds1_node")
        self.imu_publisher = self.create_publisher(Imu, "imu/data", 10)

        # quaternion
        self.q0 = 0
        self.q1 = 0
        self.q2 = 0
        self.q3 = 0

        # acceleration
        self.ax = 0
        self.ay = 0
        self.az = 0

        # gyro
        self.gx = 0
        self.gy = 0
        self.gz = 0

        self.timer = self.create_timer(0.1, self.publish_sensor_data)

    def publish_sensor_data(self):
        # read data
        (count, data) = pi.i2c_read_device(handle, 20) 

        # Unpack the data into corresponding IMU values
        if count > 0:
            self.reconstruct_imu_data(data)
        now = self.get_clock().now()

        # create IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = now.to_msg()
        imu_msg.header.frame_id = "imu_link"
        
        # Set the linear acceleration (ax, ay, az)
        imu_msg.linear_acceleration.x = self.ax
        imu_msg.linear_acceleration.y = self.ay
        imu_msg.linear_acceleration.z = self.az

        # Set the angular velocity (gx, gy, gz)
        imu_msg.angular_velocity.x = self.gx
        imu_msg.angular_velocity.y = self.gy
        imu_msg.angular_velocity.z = self.gz

        # Set the orientation (quaternion)
        imu_msg.orientation.w = self.q0
        imu_msg.orientation.x = self.q1
        imu_msg.orientation.y = self.q2
        imu_msg.orientation.z = self.q3

        imu_msg.orientation_covariance[0] = 0.01 
        imu_msg.orientation_covariance[4] = 0.01
        imu_msg.orientation_covariance[8] = 0.01

        imu_msg.angular_velocity_covariance[0] = 0.01
        imu_msg.angular_velocity_covariance[4] = 0.01
        imu_msg.angular_velocity_covariance[8] = 0.01

        imu_msg.linear_acceleration_covariance[0] = 0.01
        imu_msg.linear_acceleration_covariance[4] = 0.01
        imu_msg.linear_acceleration_covariance[8] = 0.01


        # publish messages
        self.imu_publisher.publish(imu_msg)

    def reconstruct_imu_data(self, data):
        # quaternion
        self.q0 = struct.unpack_from('h', data, 0)[0] / 10000.0
        self.q1 = struct.unpack_from('h', data, 2)[0] / 10000.0
        self.q2 = struct.unpack_from('h', data, 4)[0] / 10000.0
        self.q3 = struct.unpack_from('h', data, 6)[0] / 10000.0

        # acceleration
        self.ax = struct.unpack_from('h', data, 8)[0] / 1000.0
        self.ay = struct.unpack_from('h', data, 10)[0] / 1000.0
        self.az = struct.unpack_from('h', data, 12)[0] / 1000.0

        # gyro
        self.gx = struct.unpack_from('h', data, 14)[0] / 100.0
        self.gy = struct.unpack_from('h', data, 16)[0] / 100.0
        self.gz = struct.unpack_from('h', data, 18)[0] / 100.0



def main(args=None):
    
    rclpy.init(args=args)
    node = LSM9DS1Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()