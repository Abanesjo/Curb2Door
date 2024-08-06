#!/usr/bin/env python

import matplotlib.pyplot as plt
import os

data_dir = '/mnt/e/work/curb2door/r3live_samples/processed/r3live_run4/'

def read_matched_odometry_file(filepath):
    odometry_data = []
    with open(filepath, 'r') as file:
        for line in file:
            parts = line.strip().split()
            index = int(parts[0])
            x, y, z = map(float, parts[1:4])
            odometry_data.append((x, y, z))
    return odometry_data

def plot_trajectory(odometry_data):
    x_coords = [data[0] for data in odometry_data]
    y_coords = [data[1] for data in odometry_data]
    z_coords = [data[2] for data in odometry_data]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x_coords, y_coords, z_coords, label='Trajectory')

    # Mark the start point with a big red sphere
    ax.scatter(x_coords[0], y_coords[0], z_coords[0], color='red', s=100, label='Start Point')

    # Mark the end point with a big green sphere
    ax.scatter(x_coords[-1], y_coords[-1], z_coords[-1], color='green', s=100, label='End Point')

    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_zlabel('Z Coordinate')
    ax.set_title('Trajectory Plot')
    ax.legend()
    plt.show()

matched_odometry_file_path = os.path.join(data_dir, 'matched_odometry.txt')

if __name__ == '__main__':
    if not os.path.exists(matched_odometry_file_path):
        raise FileNotFoundError('Matched odometry file not found in the specified directory')

    odometry_data = read_matched_odometry_file(matched_odometry_file_path)
    plot_trajectory(odometry_data)
