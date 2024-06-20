import numpy as np
import cv2
import os
import open3d as o3d
from tqdm import tqdm

def project_point_cloud(points, K, Rt, image):
    """
    Project a point cloud onto an image and colorize points by depth.

    Parameters:
    points (np.array): Nx3 array containing 3D points.
    K (np.array): 3x3 camera intrinsic matrix.
    Rt (np.array): 3x4 camera extrinsic matrix (rotation and translation).
    image (np.array): The image onto which points will be projected.

    Returns:
    np.array: Image with projected points.
    """
    # Transform points from world coordinates to camera coordinates
    ones = np.ones((points.shape[0], 1))
    homogeneous_points = np.hstack((points, ones))
    camera_coords = Rt @ homogeneous_points.T

    # Project points onto the image plane
    projected_points = K @ camera_coords[:3, :]
    negative_z = projected_points[2, :] < 0
    projected_points = projected_points[:, ~negative_z]
    projected_points /= projected_points[2, :]  # Divide by the Z component

    # Convert coordinates to pixel indices
    projected_points = projected_points[:2, :].T
    projected_points = projected_points.round().astype(int)

    # Colorize points by depth (Z value)
    depths = camera_coords[2, :]
    depths = depths[~negative_z]
    max_depth = depths.max()
    min_depth = depths.min()
    colors = 255 * (depths - min_depth) / (max_depth - min_depth)
    colors = colors.astype(np.uint8)
    colormap = cv2.applyColorMap(colors, cv2.COLORMAP_JET)

    # Draw points on the image
    for i, (x, y) in enumerate(projected_points):
        if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
            image[y, x] = colormap[i]

    return image

data_dir = "/home/jya9538/Desktop/camera_fix/src/curb2door"
image_dir = os.path.join(data_dir, "images")
pcd_dir = os.path.join(data_dir, "pcd")
output_dir = os.path.join(data_dir, "calib_vis")
image_names = sorted(os.listdir(image_dir))
pcd_names = sorted(os.listdir(pcd_dir))
# print(image_names)
# print(pcd_names)
# assert()
os.makedirs(output_dir, exist_ok=True)

intrinsics = np.array([471, 0, 752, 0, 471, 752, 0, 0, 1]).reshape(3, 3)
extrinsics = np.array([
    [0.99904,-0.0176183,0.040094,-0.00069683],
    [0.0412139,0.0686159,-0.996791,0.367814],
    [0.0148107,0.997488,0.0692762,0.13945],
    [0,0,0,1]
])

for i, image_name in enumerate(tqdm(image_names[5:])):
    image = cv2.imread(os.path.join(image_dir, image_name))
    pcd_path = os.path.join(pcd_dir, pcd_names[i])
    # print(image_name, pcd_path)
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)
    image = project_point_cloud(points, intrinsics, extrinsics, image)
    cv2.imwrite(os.path.join(output_dir, f"{i:04d}.jpg"), image)