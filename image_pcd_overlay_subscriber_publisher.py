import sensor_msgs.msg as sensor_msgs
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import numpy as np

# Own files
import params 
from pcd_image_overlay import create_point_cloud_image_overlay

### Class definition
class Img_PCD_Subscriber_Overlay_Publisher():
    def __init__(self):
        super().__init__()
        #print(params.sub_topic_pcd, params.sub_topic_image)
        self.glob_cv_image = None
        self.glob_pcd_file = None
        self.ros_image_point_cloud_overlay = None
        # Subscribe to pcd file
        self.pcd_file = rospy.Subscriber(
            params.sub_topic_pcd,  # Subscribes from pcd_publisher 
            sensor_msgs.PointCloud2,
            self.sub_callback_pcd)
        self.pcd_file  # Prevent unused variable warning

        # Subscribe to image file
        self.img_file = rospy.Subscriber(
            params.sub_topic_image,  # Subscribes from image publisher
            sensor_msgs.Image,
            self.sub_callback_img)
        self.img_file  # Prevent unused variable warning

        # Publish overlay as an image
        self.overlay_publisher = rospy.Publisher(
            params.pub_topic_overlay,
            sensor_msgs.Image,
            queue_size = 1)
        rospy.Timer(rospy.Duration(1.0 / params.overlay_publish_timestep), self.pub_callback)    

        self.bridge = CvBridge()

    # Image subscriber callback function     
    def sub_callback_img(self, Image):
        try: 
            self.glob_cv_image  = self.bridge.imgmsg_to_cv2(Image, "bgr8")
        except CvBridgeError as e:
            print(e)
        # print("Subscribed to image.") 

    # Pointcloud2 file subscriber callback function
    def sub_callback_pcd(self, pointcloud):
        assert isinstance(pointcloud, sensor_msgs.PointCloud2)
        points = point_cloud2.read_points(pointcloud, field_names=("x", "y", "z"))
        self.glob_pcd_file = np.array(list(points))
        # print("Subscribed to pcd.")

    # Overlay publisher callback function
    def pub_callback(self, event):
        # Overlay image and pointcloud
        self.cv_image_point_cloud_overlay = create_point_cloud_image_overlay\
            (self.glob_pcd_file, self.glob_cv_image)
        # Convert overlay to ros msg image format
        try:
            ros_image_point_cloud_overlay = self.bridge.cv2_to_imgmsg(self.cv_image_point_cloud_overlay, "rgb8")
        except CvBridgeError as e:
            print(e) 
        ros_image_point_cloud_overlay.header.frame_id = "camera_front_point_cloud_overlay"
        self.overlay_publisher.publish(ros_image_point_cloud_overlay)

def main(args = None):
    rospy.init(args = args)
    img_pcd_sub_overlay_pub = Img_PCD_Subscriber_Overlay_Publisher()
    rospy.spin(img_pcd_sub_overlay_pub)

if __name__ == '__main__':
    main()
