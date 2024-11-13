#!/usr/bin/env python
import cv2
import numpy as np
import cv2.aruco as aruco
import os
 
def generate_aruco_marker(marker_id, marker_size=200, save_path='.'):
    # Modificato per usare getPredefinedDictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    img = aruco.drawMarker(aruco_dict, marker_id, marker_size)
    filename = os.path.join(save_path, f"marker_{marker_id}.png")
    cv2.imwrite(filename, img)
    print(f"Marker ID {marker_id} salvato come {filename}")
 
def generate_multiple_markers(start_id=0, end_id=10, marker_size=200, save_path='./markers'):
    if not os.path.exists(save_path):
        os.makedirs(save_path)
    for marker_id in range(start_id, end_id + 1):
        generate_aruco_marker(marker_id, marker_size, save_path)
 
if __name__ == '__main__':
    start_id = 0
    end_id = 8
    marker_size = 200
    save_path = './markers'
    generate_multiple_markers(start_id, end_id, marker_size, save_path)
