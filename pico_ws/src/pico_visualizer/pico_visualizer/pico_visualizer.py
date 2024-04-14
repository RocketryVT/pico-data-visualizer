import serial
from serial import Serial
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped

class SerialToImu(Node):
    def __init__(self):
        super().__init__('pico_visualizer')
        self.publisher = self.create_publisher(Imu, '/imu/data', 100)
        self.pose_publisher = self.create_publisher(PoseStamped, 'pose', 100)  # Add this line
        self.serial_port = '/dev/ttyACM0'  # Replace with your serial port
        self.serial_baudrate = 115200  # Replace with your baudrate
        # self.serial = serial.Serial(port="COM3", parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE, timeout=1, baudrate=115200)

    def read_serial_data(self):
        with serial.Serial(self.serial_port, self.serial_baudrate) as ser:
            while rclpy.ok():
                # Read data from serial port
                data = ser.readline().decode('utf-8').strip()
                
                # print(data)
                self.get_logger().info(data)

                try:
                    # Parse the data
                    altitude, quaternion_w, quaternion_x, quaternion_y, quaternion_z, linear_acceleration_x, linear_acceleration_y, linear_acceleration_z = map(float, data.split())
                    # Create Imu message
                    imu_msg = Imu()
                    imu_msg.header.frame_id = 'imu_link'
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.linear_acceleration.x = linear_acceleration_x
                    imu_msg.linear_acceleration.y = linear_acceleration_y
                    imu_msg.linear_acceleration.z = linear_acceleration_z
                    imu_msg.orientation.x = quaternion_x
                    imu_msg.orientation.y = quaternion_y
                    imu_msg.orientation.z = quaternion_z
                    imu_msg.orientation.w = quaternion_w
                    
                    pose_msg = PoseStamped()
                    pose_msg.header.frame_id = 'base_link'
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.pose.position.z = altitude  # Set altitude to z value

                    # Publish the Imu message
                    self.publisher.publish(imu_msg)
                    self.pose_publisher.publish(pose_msg)
                except ValueError:
                    self.get_logger().warn(f"Could not parse data: {data}")
                    continue
                # Parse the data
                # quaternion_w, quaternion_x, quaternion_y, quaternion_z, linear_acceleration_x, linear_acceleration_y, linear_acceleration_z = map(float, data.split())

                

def main(args=None):
    rclpy.init(args=args)
    serial_to_imu = SerialToImu()
    serial_to_imu.read_serial_data()
    rclpy.spin(serial_to_imu)
    serial_to_imu.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()