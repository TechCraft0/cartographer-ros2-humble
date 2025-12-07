#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped
import math
import re

class GPSSerialNode(Node):
    def __init__(self):
        super().__init__('gps_serial_node')
        
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('frame_id', 'gps')
        
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.frame_id = self.get_parameter('frame_id').value
        
        self.fix_pub = self.create_publisher(NavSatFix, 'fix', 10)
        self.vel_pub = self.create_publisher(TwistStamped, 'vel', 10)
        
        self.valid_fix = False
        
        try:
            self.serial_port = serial.Serial(self.port, self.baud, timeout=2)
            self.get_logger().info(f'GPS serial port opened: {self.port} at {self.baud} baud')
            self.timer = self.create_timer(0.01, self.read_gps)
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
    
    def check_nmea_checksum(self, sentence):
        parts = sentence.split('*')
        if len(parts) != 2:
            return False
        checksum = 0
        for c in parts[0][1:]:
            checksum ^= ord(c)
        return ("%02X" % checksum) == parts[1].upper()
    
    def convert_latitude(self, field):
        if not field or len(field) < 4:
            return 0.0
        try:
            return float(field[0:2]) + float(field[2:]) / 60.0
        except:
            return 0.0
    
    def convert_longitude(self, field):
        if not field or len(field) < 5:
            return 0.0
        try:
            return float(field[0:3]) + float(field[3:]) / 60.0
        except:
            return 0.0
    
    def parse_gga(self, fields):
        if len(fields) < 15:
            return None
        
        if not fields[2] or not fields[4]:
            return None
        
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self.frame_id
        
        lat = self.convert_latitude(fields[2])
        if fields[3] == 'S':
            lat = -lat
        lon = self.convert_longitude(fields[4])
        if fields[5] == 'W':
            lon = -lon
        
        fix.latitude = lat
        fix.longitude = lon
        
        if fields[9]:
            try:
                fix.altitude = float(fields[9])
                if fields[11]:
                    fix.altitude += float(fields[11])
            except:
                fix.altitude = 0.0
        
        fix_quality = int(fields[6]) if fields[6] else 0
        if fix_quality > 0:
            fix.status.status = NavSatStatus.STATUS_FIX
            self.valid_fix = True
        else:
            fix.status.status = NavSatStatus.STATUS_NO_FIX
            self.valid_fix = False
        
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        
        hdop = float(fields[8]) if fields[8] else 1.0
        fix.position_covariance[0] = (hdop * 4.0) ** 2
        fix.position_covariance[4] = (hdop * 4.0) ** 2
        fix.position_covariance[8] = (hdop * 8.0) ** 2
        
        return fix
    
    def parse_vtg(self, fields):
        if not self.valid_fix or len(fields) < 9:
            return None
        
        vel = TwistStamped()
        vel.header.stamp = self.get_clock().now().to_msg()
        vel.header.frame_id = self.frame_id
        
        if fields[7]:
            try:
                speed = float(fields[7]) / 3.6
                if fields[1]:
                    course = math.radians(float(fields[1]))
                    vel.twist.linear.x = speed * math.sin(course)
                    vel.twist.linear.y = speed * math.cos(course)
                return vel
            except:
                pass
        return None
    
    def read_gps(self):
        try:
            line = self.serial_port.readline().decode('ascii', errors='ignore').strip()
            if not line or not line.startswith('$'):
                return
            
            if not self.check_nmea_checksum(line):
                self.get_logger().debug(f'Checksum failed: {line}', throttle_duration_sec=5.0)
                return
            
            fields = [f.strip(',') for f in line.split(',')]
            if len(fields) < 1:
                return
            
            fields[-1] = fields[-1].split('*')[0]
            sentence_type = fields[0]
            
            if sentence_type in ['$GNGGA', '$GPGGA']:
                self.get_logger().info(f'GGA: {line}', throttle_duration_sec=2.0)
                fix = self.parse_gga(fields)
                if fix:
                    self.fix_pub.publish(fix)
                    self.get_logger().info(f'GPS Fix: lat={fix.latitude:.6f}, lon={fix.longitude:.6f}', throttle_duration_sec=1.0)
                else:
                    self.get_logger().info('GGA parsed but no valid fix (indoor/no satellites)', throttle_duration_sec=5.0)
            elif sentence_type in ['$GNVTG', '$GPVTG']:
                vel = self.parse_vtg(fields)
                if vel:
                    self.vel_pub.publish(vel)
        except Exception as e:
            self.get_logger().warn(f'Error: {e}', throttle_duration_sec=5.0)
    
    def destroy_node(self):
        if hasattr(self, 'serial_port'):
            self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GPSSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
