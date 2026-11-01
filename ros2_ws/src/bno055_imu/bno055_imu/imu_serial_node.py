import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Temperature

from std_msgs.msg import UInt8MultiArray

import serial
import time
import math

# Arduino packet definition

# Packet:
#
# IMU,
# seq,
# millis,
# qw,qx,qy,qz,
# gx,gy,gz,
# ax,ay,az,
# lax,lay,laz,
# grx,gry,grz,
# mx,my,mz,
# temp,
# sys,gyro,accel,mag
#

HEADER = "IMU"

# Main ROS2 Node

class IMUSerialNode(Node):

    def __init__(self):

        super().__init__('imu_serial_node')

        self.declare_parameter(
            'port',
            '/dev/ttyACM0'
        )

        self.declare_parameter(
            'baudrate',
            115200
        )

        self.declare_parameter(
            'frame_id',
            'imu_link'
        )

        self.declare_parameter(
            'publish_rate',
            100.0
        )

        self.declare_parameter(
            'reconnect_timeout',
            2.0
        )


        self.port = (
            self.get_parameter('port')
            .get_parameter_value()
            .string_value
        )


        self.baudrate = (
            self.get_parameter('baudrate')
            .get_parameter_value()
            .integer_value
        )


        self.frame_id = (
            self.get_parameter('frame_id')
            .get_parameter_value()
            .string_value
        )


        self.publish_rate = (
            self.get_parameter('publish_rate')
            .get_parameter_value()
            .double_value
        )


        self.reconnect_timeout = (
            self.get_parameter('reconnect_timeout')
            .get_parameter_value()
            .double_value
        )

        # Serial variables

        self.serial_port = None

        self.last_connection_attempt = 0.0

        self.packet_count = 0


        # ROS Publishers

        self.imu_pub = self.create_publisher(
            Imu,
            '/imu/data',
            10
        )


        self.mag_pub = self.create_publisher(
            MagneticField,
            '/imu/mag',
            10
        )


        self.temp_pub = self.create_publisher(
            Temperature,
            '/imu/temperature',
            10
        )


        self.calib_pub = self.create_publisher(
            UInt8MultiArray,
            '/imu/calibration',
            10
        )


        # Covariance values
        # These values are initial estimates.
        # Later these parameters can be tuned using sensor testing. Use the .py file under tools folder.

        self.orientation_covariance = [
            0.01, 0.0, 0.0,    # Measured value  by recording IMU data 9.130453129848692e-31
            0.0, 0.01, 0.0,    # Measured value  by recording IMU data 6.202083033078805e-28
            0.0, 0.0, 0.02    # Measured value  by recording IMU data 5.4473242452181246e-33
        ]


        self.angular_velocity_covariance = [
            0.02, 0.0, 0.0,    # Measured value  by recording IMU data 5.356029426317522e-06
            0.0, 0.02, 0.0,   # Measured value  by recording IMU data 1.0726795166799256e-05
            0.0, 0.0, 0.02    # Measured value  by recording IMU data 4.3564913560017094e-06
        ]


        self.linear_acceleration_covariance = [
            0.04, 0.0, 0.0,    # Measured value  by recording IMU data 0.0003824760423610028
            0.0, 0.04, 0.0,   # Measured value  by recording IMU data 0.00020821564221512031
            0.0, 0.0, 0.04     # Measured value  by recording IMU data 0.0006647305910450847
        ]


        self.magnetic_covariance = [
            0.05, 0.0, 0.0,  # Measured value  by recording IMU data 7.326899289577471e-13
            0.0, 0.05, 0.0,  # Measured value  by recording IMU data 6.382346389821366e-13
            0.0, 0.0, 0.05  # Measured value  by recording IMU data 1.0266705150040357e-12
        ]


        # Timer

        period = 1.0 / self.publish_rate


        self.timer = self.create_timer(
            period,
            self.timer_callback
        )


        self.get_logger().info(
            "BNO055 Serial Driver Started"
        )


        self.get_logger().info(
            f"Port: {self.port}"
        )


        self.get_logger().info(
            f"Baudrate: {self.baudrate}"
        )

    # Main Timer Callback

    def timer_callback(self):
        
        # Main execution loop

        # Read Arduino serial

        line = self.read_serial_line()

        if line is None:

            return

        if line == "IMU_READY":

            self.get_logger().info(
                "Arduino IMU Ready"
            )

            return


        if line.startswith("ERROR"):

            self.get_logger().error(
                line
            )

            return

        # Parse packet

        imu_data = self.parse_packet(line)


        if imu_data is None:

            return

        self.packet_count += 1


        # Publish ROS2 messages

        self.publish_imu(
            imu_data
        )


        self.publish_magnetic_field(
            imu_data
        )


        self.publish_temperature(
            imu_data
        )


        self.publish_calibration(
            imu_data
        )

        # Debug output

        if self.packet_count % 100 == 0:

            self.get_logger().info(
                f"IMU packets received: {self.packet_count}"
            )
    
    
    # Serial Connection


    def connect_serial(self):
        """
        Open serial connection with Arduino
        """

        current_time = time.time()


        # Stop frequent trying
        if (
            current_time - self.last_connection_attempt
            <
            self.reconnect_timeout
        ):
            return False


        self.last_connection_attempt = current_time


        try:

            self.serial_port = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.1
            )


            # Clear old data
            time.sleep(2.0)
            self.serial_port.reset_input_buffer()


            self.get_logger().info(
                f"Connected to Arduino on {self.port}"
            )


            return True


        except serial.SerialException as error:


            self.serial_port = None


            self.get_logger().warn(
                f"Serial connection failed: {error}"
            )


            return False


    def disconnect_serial(self):
        
        # Close serial connection

        if self.serial_port is not None:

            try:

                self.serial_port.close()


            except Exception:

                pass


        self.serial_port = None


    def read_serial_line(self):
        
        # Read one line from Arduino
        
        # No connection
        if self.serial_port is None:

            if not self.connect_serial():

                return None

        try:

            if self.serial_port.in_waiting > 0:

                line = (
                    self.serial_port
                    .readline()
                    .decode('utf-8', errors='ignore')
                    .strip()
                )

                return line

            else:

                return None

        except (
            serial.SerialException,
            UnicodeDecodeError
        ) as error:


            self.get_logger().warn(
                f"Serial read error: {error}"
            )


            self.disconnect_serial()


            return None
        
    
    # Packet Parser

    def parse_packet(self, line):
        
        # Parse Arduino IMU packet that returns dictionary containing IMU data

        try:

            fields = line.split(',')


            # Check packet header

            if fields[0] != HEADER:

                return None

            # Check packet length

            # Expected fields:
            # 0  IMU
            # 1  sequence
            # 2  millis
            # 3-6 quaternion
            # 7-9 gyro
            # 10-12 accel
            # 13-15 linear accel
            # 16-18 gravity
            # 19-21 magnetometer
            # 22 temperature
            # 23-26 calibration

            if len(fields) != 27:

                self.get_logger().warn(
                    f"Invalid packet length: {len(fields)}"
                )

                return None


            # Create dictionary

            imu_data = {}

            imu_data["sequence"] = int(fields[1])

            imu_data["arduino_time"] = int(fields[2])

            # Quaternion

            imu_data["qw"] = float(fields[3])
            imu_data["qx"] = float(fields[4])
            imu_data["qy"] = float(fields[5])
            imu_data["qz"] = float(fields[6])

            # Gyroscope

            imu_data["gx"] = float(fields[7])
            imu_data["gy"] = float(fields[8])
            imu_data["gz"] = float(fields[9])

            # Accelerometer

            imu_data["ax"] = float(fields[10])
            imu_data["ay"] = float(fields[11])
            imu_data["az"] = float(fields[12])

            # Linear acceleration

            imu_data["lax"] = float(fields[13])
            imu_data["lay"] = float(fields[14])
            imu_data["laz"] = float(fields[15])

            # Gravity
        
            imu_data["grx"] = float(fields[16])
            imu_data["gry"] = float(fields[17])
            imu_data["grz"] = float(fields[18])

            # Magnetometer

            imu_data["mx"] = float(fields[19])
            imu_data["my"] = float(fields[20])
            imu_data["mz"] = float(fields[21])

            # Temperature

            imu_data["temperature"] = int(fields[22])

            # Calibration

            imu_data["sys_cal"] = int(fields[23])
            imu_data["gyro_cal"] = int(fields[24])
            imu_data["accel_cal"] = int(fields[25])
            imu_data["mag_cal"] = int(fields[26])

            return imu_data

        except (
            ValueError,
            IndexError
        ) as error:
            

            self.get_logger().warn(
                f"Packet parsing error: {error}"
            )


            return None
        
    # ROS2 Publishers

    def publish_imu(self, data):
        """
        Publish sensor_msgs/msg/Imu
        """


        msg = Imu()

        # Timestamp

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = self.frame_id

        # Orientation Quaternion

        msg.orientation.w = data["qw"]
        msg.orientation.x = data["qx"]
        msg.orientation.y = data["qy"]
        msg.orientation.z = data["qz"]

        msg.orientation_covariance = (
            self.orientation_covariance
        )

        # Angular Velocity (BNO055 outputs rad/s)

        msg.angular_velocity.x = data["gx"]
        msg.angular_velocity.y = data["gy"]
        msg.angular_velocity.z = data["gz"]

        msg.angular_velocity_covariance = (
            self.angular_velocity_covariance
        )

        # Linear Acceleration (Using BNO055 linear acceleration. Gravity removed )

        msg.linear_acceleration.x = data["lax"]
        msg.linear_acceleration.y = data["lay"]
        msg.linear_acceleration.z = data["laz"]

        msg.linear_acceleration_covariance = (
            self.linear_acceleration_covariance
        )

        self.imu_pub.publish(msg)


    def publish_magnetic_field(self, data):
        
        # Publish sensor_msgs/msg/MagneticField

        msg = MagneticField()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = self.frame_id

        # BNO055 magnetometer unit (micro Tesla):
        # ROS MagneticField unit (Tesla):

        msg.magnetic_field.x = (
            data["mx"] * 1e-6
        )

        msg.magnetic_field.y = (
            data["my"] * 1e-6
        )

        msg.magnetic_field.z = (
            data["mz"] * 1e-6
        )

        msg.magnetic_field_covariance = (
            self.magnetic_covariance
        )

        self.mag_pub.publish(msg)
        

    def publish_temperature(self, data):

        # Publish sensor_msgs/msg/Temperature
        
        msg = Temperature()


        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = self.frame_id

        # Celsius

        msg.temperature = (
            float(data["temperature"])
        )

        # Unknown variance

        msg.variance = 0.0

        self.temp_pub.publish(msg)
        

    def publish_calibration(self, data):
        
        # Publish BNO055 calibration status

        msg = UInt8MultiArray()

        msg.data = [

            data["sys_cal"],

            data["gyro_cal"],

            data["accel_cal"],

            data["mag_cal"]

        ]

        self.calib_pub.publish(msg)
        

def main(args=None):

    rclpy.init(args=args)


    node = IMUSerialNode()


    try:

        rclpy.spin(node)


    except KeyboardInterrupt:

        pass



    finally:

        node.disconnect_serial()

        node.destroy_node()

        if rclpy.ok():
            
            rclpy.shutdown()



if __name__ == '__main__':

    main()
        
    
    
    
