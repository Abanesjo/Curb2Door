#!/usr/bin/env python
import numpy as np
from tqdm import tqdm
import os
import glob
import shutil

data_dir = '/mnt/e/work/curb2door/r3live_samples/processed/r3live_run4/'
odometry_file_path = os.path.join(data_dir, 'odom.txt')
image_dir = os.path.join(data_dir, 'img')
output_image_dir = os.path.join(data_dir, 'mapped_images')
output_odometry_file = os.path.join(data_dir, 'matched_odometry.txt')

def parse_odometry_file(filepath):
    odometry_data = []
    with open(filepath, 'r') as file:
        print("Parsing Odometry File")
        for line in tqdm(file):
            parts = line.strip().split()
            timestamp = parts[0]
            odometry_data.append((timestamp, list(map(float, parts[1:]))))
    return odometry_data

def find_closest_timestamp(odometry_data, image_timestamp):
    closest_entry = min(odometry_data, key=lambda x: abs(float(x[0]) - float(image_timestamp)))
    return closest_entry

def match_images_to_odometry(image_folder, odometry_data, threshold_ns=1_000_000_000):
    matched_data = []
    image_files = glob.glob(os.path.join(image_folder, '*.png'))
    print("Matching Images to Odometry.")
    for index, image_file in enumerate(tqdm(image_files)):
        image_timestamp = os.path.basename(image_file).replace('.png', '')
        closest_entry = find_closest_timestamp(odometry_data, image_timestamp)
        timestamp_diff = abs(float(closest_entry[0]) - float(image_timestamp))
        if timestamp_diff <= threshold_ns:
            matched_data.append((index, image_file, closest_entry))
    return matched_data

if __name__ == '__main__':
    if not os.path.exists(image_dir) or not os.path.exists(odometry_file_path):
        raise FileNotFoundError('Image or odometry data not found in the specified directory')
    
    odometry_data = parse_odometry_file(odometry_file_path)
    matched_data = match_images_to_odometry(image_dir, odometry_data)

    if not os.path.exists(output_image_dir):
        os.makedirs(output_image_dir)

    with open(output_odometry_file, 'w') as out_file:
        print("Writing Outputs")
        for index, image_file, (odometry_timestamp, odometry_data) in tqdm(matched_data):
            out_file.write(f"{index} {' '.join(map(str, odometry_data))}\n")
            new_image_filename = f"{index:06d}.png"
            shutil.copy(image_file, os.path.join(output_image_dir, new_image_filename))

    print(f"Matched odometry written to {output_odometry_file}")
    print(f"Mapped images copied to {output_image_dir}")