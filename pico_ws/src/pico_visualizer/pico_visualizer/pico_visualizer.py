import serial
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class SerialToImu(Node):
    def __init__(self):
        super().__init__('pico_visualizer')
        self.publisher = self.create_publisher(Imu, '/imu/data', 10)
        self.serial_port = '/dev/ttyUSB0'  # Replace with your serial port
        self.serial_baudrate = 115200  # Replace with your baudrate
        # self.serial = serial.Serial(port="COM3", parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE, timeout=1, baudrate=115200)

    def read_serial_data(self):
        with serial.Serial(self.serial_port, self.serial_baudrate) as ser:
            while rclpy.ok():
                # Read data from serial port
                data = ser.read(14)  # Replace 14 with the number of bytes you expect to receive

                # Parse the data
                linear_acceleration_x, linear_acceleration_y, linear_acceleration_z, \
                quaternion_x, quaternion_y, quaternion_z, quaternion_w = struct.unpack('ffffff', data)

                # Create Imu message
                imu_msg = Imu()
                imu_msg.header.frame_id = 'map'
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.linear_acceleration.x = linear_acceleration_x
                imu_msg.linear_acceleration.y = linear_acceleration_y
                imu_msg.linear_acceleration.z = linear_acceleration_z
                imu_msg.orientation.x = quaternion_x
                imu_msg.orientation.y = quaternion_y
                imu_msg.orientation.z = quaternion_z
                imu_msg.orientation.w = quaternion_w

                # Publish the Imu message
                self.publisher.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    serial_to_imu = SerialToImu()
    serial_to_imu.read_serial_data()
    rclpy.spin(serial_to_imu)
    serial_to_imu.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()