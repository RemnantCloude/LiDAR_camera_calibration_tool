from image_pcd_overlay_subscriber_publisher import Img_PCD_Subscriber_Overlay_Publisher
import slider

import rospy
import threading

if __name__ == '__main__':
    # Starting the slider in a different thread
    threading.Thread(target = slider.create_slider).start()
    # Running the subscriber publisher node
    rospy.init_node('camera_lidar_calibration', anonymous=True)
    img_pcd_sub_overlay_pub = Img_PCD_Subscriber_Overlay_Publisher()
    rospy.spin()