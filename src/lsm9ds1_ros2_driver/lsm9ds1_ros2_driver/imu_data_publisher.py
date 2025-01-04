#!/home/danitns/Documents/imu_publisher/env_imu/bin/python3

import time
import board
import adafruit_lsm9ds1
from digitalio import DigitalInOut, Direction

# Import ROS2 libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
from geometry_msgs.msg import Vector3

magnometer_correction_x = 0.30030000000000007
magnometer_correction_y = -0.27327999999999997
magnometer_correction_z = 0.07140000000000002

accel_correction_x = 0.14
accel_correction_y = -0.4
accel_correction_z = -0.11

gyro_correction_x = 0.0238237442897226
gyro_correction_y = 0.05062545661566052
gyro_correction_z = -0.106581392688091176

spi = board.SPI()
csag = DigitalInOut(board.D5)
csag.direction = Direction.OUTPUT
csag.value = True
csm = DigitalInOut(board.D6)
csm.direction = Direction.OUTPUT
csm.value = True
sensor = adafruit_lsm9ds1.LSM9DS1_SPI(spi, csag, csm)

class LSM9DS1Node(Node):
    def __init__(self):
        super().__init__("lsm9ds1_node")
        self.imu_publisher = self.create_publisher(Imu, "imu/data_raw", 10)
        self.magnetometer_publisher = self.create_publisher(MagneticField, "imu/mag", 10)
        # self.temp_publisher = self.create_publisher(Temperature, "imu/temp", 10)

        self.timer = self.create_timer(0.1, self.publish_sensor_data)

    def publish_sensor_data(self):
        # read data
        accel_x, accel_y, accel_z = sensor.acceleration
        mag_x, mag_y, mag_z = sensor.magnetic
        gyro_x, gyro_y, gyro_z = sensor.gyro
        now = self.get_clock().now()
        # temp = sensor.temperature

        # apply corrections
        accel_x -= accel_correction_x
        accel_y -= accel_correction_y
        accel_z -= accel_correction_z

        mag_x -= magnometer_correction_x
        mag_y -= magnometer_correction_y
        mag_z -= magnometer_correction_z

        gyro_x -= gyro_correction_x
        gyro_y -= gyro_correction_y
        gyro_z -= gyro_correction_z

        # create IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = now.to_msg()
        imu_msg.header.frame_id = "imu_link"
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y * (-1)
        imu_msg.linear_acceleration.z = accel_z

        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y * (-1)
        imu_msg.angular_velocity.z = gyro_z

        # create magnetometer message
        mag_msg = MagneticField()
        mag_msg.header.stamp = now.to_msg()
        mag_msg.header.frame_id = "imu_link"
        mag_msg.magnetic_field.x = mag_x * (-1)
        mag_msg.magnetic_field.y = mag_y * (-1)
        mag_msg.magnetic_field.z = mag_z

        # create temperature message
        # temp_msg = Temperature()
        # temp_msg.temperature = temp

        # publish messages
        self.imu_publisher.publish(imu_msg)
        self.magnetometer_publisher.publish(mag_msg)
        # self.temp_publisher.publish(temp_msg)


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
