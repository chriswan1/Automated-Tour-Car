from rclpy.node import Node
import rclpy
from std_msgs.msg import String, Float64MultiArray
import utm
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GPSSubscriber(Node):
    def __init__(self):
        super().__init__('gps_subscriber')

        self.subscript = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            'coords',
            10
        )
        self.subscript
        
    def listener_callback(self, msg):
        print(msg)
        #if msg.
        '''
        xy = coords.split(',')
        x = float(xy[0])
        y = float(xy[1])
        #self.get_logger().info('I heard: "%f, %f"', x,y)
        
        utmcoords = utm.from_latlon(x,y)
        x = float(utmcoords[0])-421997.0166
        x = x * 14.69
        y = float(utmcoords[1])-3723007.886
        y = y * 14.711
        #print(x,y)
        arr = Float64MultiArray()
        arr.data=[x,y]
        self.publisher_.publish(arr)
        '''


def main(args=None):
    rclpy.init(args=args)
    gps_messages = GPSSubscriber()
    rclpy.spin(gps_messages)
    rclpy.shutdown()


if __name__ == '__main__':
    main()