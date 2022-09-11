import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import cv2 as cv
import io # Needed for the buffer functionality
from PIL import Image
from ai import cs # Coordinate transformation package
import copy
# Own files
import transformations
import params

### Convert a matplotlib figure to a jpg image in openCV format and return it
def fig2img(fig):
    buf = io.BytesIO() # The buffer is defined as binary (needed for images)
    plt.savefig(buf, format = 'jpg', dpi = params.dpi) # Save figure as jpg to buffer (Pufferspeicher)
    buf.seek(0) # Needed in order to make cv.imread start reading at beginning of buf
    pil_img = copy.deepcopy(Image.open(buf)).convert('RGB') # Read from buffer as PIL image (read in as openCV image does not work)
    img = np.array(pil_img) # Convert PIL image format to openCV format
    img = img[:, :, ::-1].copy() # Transformations
    buf.close()
    return img

### Create a plot and .jpg with front camera image and pointcloud overlay 
def create_point_cloud_image_overlay(pcd_points, cam_image):

   # Initial Calibration 
   # Transformation of point cloud points into camera coordinate system
   # Adding user input to the initial transformation
    translation_tot = ((params.translation[0]+params.usr_translation[0]),(params.translation[1]+params.usr_translation[1]), (params.translation[2]+params.usr_translation[2]))
    rotation_tot = ((params.rotation[0]+params.usr_rotation[0]), (params.rotation[1]+params.usr_rotation[1]), (params.rotation[2]+params.usr_rotation[2]))
    calib_matrix = transformations.matrix(rotation_tot, translation_tot) 

    # delete x < 0
    mask = np.where(pcd_points[:,0]>0)[0]
    filtered_pcd_points = pcd_points[mask,:]

    try:
        dimensions = filtered_pcd_points.shape # n*3
    except:
        print("No pointcloud has been received. Make sure to run the pcd publisher before starting the main.py file and check for correct topic names.")
        print("The overlay publisher has been stopped.")
        exit()

    filtered_pcd_points = np.insert(filtered_pcd_points, 3, values=1, axis=1) # n*4
    pc2img = ((params.mp@calib_matrix)@(filtered_pcd_points.T)).T # n*3
    pc2img[:,0] = np.divide(pc2img[:,0],pc2img[:,2])
    pc2img[:,1] = np.divide(pc2img[:,1],pc2img[:,2])
    pc2img = pc2img[:,:2]

    mask = np.where((pc2img[:,0]> 0) 
        & (pc2img[:,0]< params.width_pixels) 
        & (pc2img[:,1]> 0) 
        & (pc2img[:,1]< params.height_pixels))[0]
    projected_3D_points_to_2D_project_points = pc2img[mask,:]

    # Cartesian --> spherical
    spherical_pcd_points = np.zeros(dimensions)
    for i in range (0, dimensions[0]):
        spherical_pcd_points[i, 0], spherical_pcd_points[i, 1], spherical_pcd_points[i, 2] = \
            cs.cart2sp(filtered_pcd_points[i,0],filtered_pcd_points[i,1], filtered_pcd_points[i,2])  #xyz --> r theta phi
    spherical_pcd_points = spherical_pcd_points[mask,:]
    
    undistort_img = cam_image.copy()
    cv.undistort(cam_image, params.intrinsic, params.distortion, undistort_img)

    # Create plot with image and overlaying point cloud points
    try:    
        implot = plt.imshow(cam_image)
    except:
        print("No image has been received. Make sure to run the image  before starting the main.py file and check for correct topic names.")
        print("The overlay publisher has been stopped.")
        exit()
        
    scat = plt.scatter(projected_3D_points_to_2D_project_points[:,0], projected_3D_points_to_2D_project_points[:,1],params.point_size, \
        c = spherical_pcd_points[:,0], cmap = cm.gist_ncar)
    plt.axis("off")
    image_pointcloud_overlay_front = fig2img(scat)
    implot.remove()
    scat.remove()

    return image_pointcloud_overlay_front
    


