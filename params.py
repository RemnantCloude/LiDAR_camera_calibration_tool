import numpy as np

### Camera parameters / projection parameters

# Name of the topic the overlay node gets its pcd files from
sub_topic_pcd = '/rslidar_points'

# Name of the topic the overlay node gets its image files from
sub_topic_image = '/lbas_image'

# Name of the topic under which the overlay image is published --> this topic must be received by RVIZ
pub_topic_overlay = '/camera_lidar_overlay'

# Initial Calibration
# Enter the negative vector from LiDAR to camera
translation = (0, 0, 0)   # meter   
rotation = (90, -90, 0) # x, y, z. The order the rotation is done is z,y',x'', degree, clockwise is positive

# Default user input transformation / rotation --> don't change
usr_translation = (0,0,0) 
usr_rotation = (0,0,0)

# Following parameters need to be defined in case of simulator data or camera usage
width_pixels = 1440 # horizontal number of pixels from camera image / simulation image
height_pixels = 1080 # vertical number of pixels from camera image / simulation image

intrinsic = np.array([
      [2240.526817, 0.000000, 638.231713],
      [0.000000, 2232.334100, 572.292534],
      [0.000000, 0.000000, 1.000000]
    ])

distortion = np.array([0, 0, 0, 0, 0])

# Projection matrix
mp = np.array([
      [2199.676025, 0.000000, 630.104930, 0.000000],
      [0.000000, 2211.820557, 571.834934, 0.000000],
      [0.000000, 0.000000, 1.000000, 0.000000],
    ])

# Timestep the overlay is published in!
# --> must be longer than the processing time of the computer
overlay_publish_timestep = 1 # Seconds

# Quality of overlay image
dpi = 250 # Pixels per inch

# Size of the LiDAR points on the overlay
point_size=0.1

# Slider parameters
# Max / min slider value
max_trans = 0.5 # meters
min_trans = -0.5 # meters
max_deg = 10 # degrees
min_deg = -10 # degrees

resolution_trans = 0.001 # Defines the steprange of the translation sliders in meters
resolution_rot = 0.01 # Defines the steprange of the rotation slider in degrees

width_slider = 800 # Defines the width of the GUI window in pixels