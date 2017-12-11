import numpy as np
from scipy import ndimage
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def extract_features(img):
    # create an array of zeros same xy size as img, but single channel
    terrain = np.zeros_like(img[:,:,0])
    rocks = np.zeros_like(img[:,:,0])
    obstacles = np.zeros_like(img[:,:,0])
    # require that each pixel be above all three threshold values in rgb
    # above_thresh will now contain a boolean array with "true"
    # where threshold was met
    terrain_rgb = (160, 160, 160)
    terrain_thresh = (img[:,:,0] > terrain_rgb[0]) \
                & (img[:,:,1] > terrain_rgb[1]) \
                & (img[:,:,2] > terrain_rgb[2])
    yellow_rgb = (120, 120, 50)
    yellow_thresh = (img[:,:,0] >= yellow_rgb[0]) \
                & (img[:,:,1] >= yellow_rgb[1]) \
                & (img[:,:,2] < yellow_rgb[2])
#     obstacles_rgb = (100, 100, 100)
#     obstacles_thresh = (img[:,:,0] <= obstacles_rgb[0]) \
#                 & (img[:,:,1] <= obstacles_rgb[1]) \
#                 & (img[:,:,2] <= obstacles_rgb[2])
    # index the array of zeros with the boolean array and set to 1
    terrain[terrain_thresh] = 1
    rocks[yellow_thresh] = 1
#     obstacles[obstacles_thresh] = 1
    # return the binary image
    return terrain, rocks #, obstacles

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle)
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))

    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):

    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))

    return warped, mask

debug = False
display_interval = 10000

def rover_img_to_world(img, xpos, ypos, yaw, world_size, scale):
    # 4) Convert thresholded image pixel values to rover-centric coords
    rover_centric_x, rover_centric_y = rover_coords(img)
    # 5) Convert rover-centric pixel values to world coords
    world_x, world_y = pix_to_world(rover_centric_x, rover_centric_y, xpos, ypos, yaw, world_size, scale)

    return world_x, world_y, rover_centric_x, rover_centric_y

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()

    # Example of how to use the Databucket() object defined above
    # to print the current x, y and yaw values
    # print(data.xpos[data.count], data.ypos[data.count], data.yaw[data.count])
    xpos, ypos = Rover.pos
    yaw = Rover.yaw
    # Define calibration box in source (actual) and destination (desired) coordinates
    # These source and destination points are defined to warp the image
    # to a grid where each 10x10 pixel square represents 1 square meter
    # The destination box will be 2*dst_size on each side
    dst_size = 5
    # Set a bottom offset to account for the fact that the bottom of the image
    # is not the position of the rover but a bit in front of it
    # this is just a rough guess, feel free to change it!
    bottom_offset = 6
    scale = dst_size * 2
    world_size = Rover.worldmap.shape[0]

    # Save velocity history
    Rover.vel_hist.append(Rover.vel)

    # Average velocity
    if Rover.vel_hist:
        Rover.avg_vel = np.mean(Rover.vel_hist)

    # NOTE: camera image is coming to you in Rover.img
    img = Rover.img

    # 1) Define source and destination points for perspective transform
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                  [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    # 2) Apply perspective transform
    warped, mask = perspect_transform(img, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    terrain, rocks = extract_features(warped)

    # Edge detection
    # sx = ndimage.sobel(terrain, axis=1, mode='constant')
    # sy = ndimage.sobel(terrain, axis=0, mode='constant')
    # terrain_edges = np.hypot(sx, sy)

    # Mask removes the blank pixels that are unseen by the rover
    obstacles = np.absolute(np.float32(terrain) - 1) * mask

    # Edge detection
    # sx = ndimage.sobel(obstacles, axis=1, mode='constant')
    # sy = ndimage.sobel(obstacles, axis=0, mode='constant')
    # obstacles_edges = np.hypot(sx, sy)

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = obstacles * 255
    Rover.vision_image[:,:,2] = terrain * 255

    # 5) Convert map image pixel values to rover-centric coords
    # 6) Convert rover-centric pixel values to world coordinates
    world_terrain_x, world_terrain_y, rover_terrain_x, rover_terrain_y = rover_img_to_world(terrain, xpos, ypos, yaw, world_size, scale)

    # For obstacles you can just invert your color selection that you used to detect ground pixels,
    # i.e., if you've decided that everything above the threshold is navigable terrain, then
    # everthing below the threshold must be an obstacle!
    world_obstacle_x, world_obstacle_y, rover_obstacle_x, rover_obstacle_y = rover_img_to_world(obstacles, xpos, ypos, yaw, world_size, scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    if (Rover.roll > 359.0 or Rover.roll < 1.0) and (Rover.pitch > 359.0 or Rover.pitch < 1.0):
        Rover.worldmap[world_obstacle_y, world_obstacle_x, 0] += 1
        Rover.worldmap[world_terrain_y, world_terrain_x, 2] += 10
    #navigable_pix = Rover.worldmap[:, :, 2] > 0
    #Rover.worldmap[navigable_pix, 0] = 0

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    dists, angles = to_polar_coords(rover_terrain_x, rover_terrain_y)
    obst_dists, obst_angles = to_polar_coords(rover_obstacle_x, rover_obstacle_y)
    # Attempt at left wall edge detection
    # obst_edge_dists, obst_edge_angles = to_polar_coords(rover_obstacles_edges_x, rover_obstacles_edges_y)
    # if len(obst_edge_angles) >= 10:
    #     theta_a = obst_edge_angles[5]
    #     theta_b = 180 - theta_a
    #     a = obst_edge_dists[5]
    #     b = obst_edge_dists[10]
    #     c = np.sqrt((a**2) + (b**2) - (2 * a * b * np.cos(theta_b - theta_a)))
    #     theta_c = 1/np.cos(((c**2) + (a**2) - (b**2)) / (2 * c * a)) - theta_b
    #     print("theta_a", theta_a, "theta_b", theta_b, "theta_c", theta_c, "a", a, "b", b, "c", c)
    #
    #     Rover.left_wall_angle = theta_c
    Rover.nav_dists = dists
    Rover.nav_angles = angles

    if rocks.any():
        # For rocks, think about imposing a lower and upper boundary in your color selection to be
        # more specific about choosing colors. You can investigate the colors of the rocks
        # (the RGB pixel values) in an interactive matplotlib window to get a feel for the appropriate
        # threshold range (keep in mind you may want different ranges for each of R, G and B!). Feel
        # free to get creative and even bring in functions from other libraries. Here's an example of
        # color selection using OpenCV.
        world_rocks_x, world_rocks_y, rover_rocks_x, rover_rocks_y = rover_img_to_world(rocks, xpos, ypos, yaw, world_size, scale)
        rock_dists, rock_angles = to_polar_coords(rover_rocks_x, rover_rocks_y)
        rock_idx = np.argmin(rock_dists)
        rock_xcenter = world_rocks_x[rock_idx]
        rock_ycenter = world_rocks_y[rock_idx]
        Rover.rock_dists = rock_dists
        Rover.rock_angles = rock_angles
        Rover.worldmap[rock_ycenter, rock_xcenter, 1] = 255
        Rover.vision_image[:,:,1] = rocks * 255
    else:
        Rover.rock_dists = None
        Rover.rock_angles = None
        Rover.vision_image[:,:,1] = 0

    return Rover
