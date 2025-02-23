import rclpy
from sensor_msgs.msg import CompressedImage
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from find_object_py.find_orange_ball import capture
from geometry_msgs.msg import Point

class FindObject(Node):
    def __init__(self):
        super().__init__('find_object_py')

        # Create a custom QoS profile
        custom_qos_profile=QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,history=QoSHistoryPolicy.KEEP_LAST,durability=QoSDurabilityPolicy.VOLATILE,depth=1)

        # Create the subscription
        self.subscription = self.create_subscription(
            CompressedImage, 
            '/image_raw/compressed', 
            self._image_callback, 
            custom_qos_profile
        )

        self.publisher_ = self.create_publisher(Point, '/obj_coords', 10)

    def _image_callback(self, frame):
        # self.get_logger().info('Received an image')
        obj_coords = float(capture(frame))

        # if a 0 is sent, assume the object is in the center of the image (for if no image is detected)
        if obj_coords == 0:
            obj_coords = float(0)

        # Convert message to ROS message
        obj_coords_point = Point()
        obj_coords_point.x = obj_coords

        self.publisher_.publish(obj_coords_point)

def main(args=None):
    rclpy.init(args=args)

    find_object = FindObject()

    rclpy.spin(find_object)

    find_object.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
