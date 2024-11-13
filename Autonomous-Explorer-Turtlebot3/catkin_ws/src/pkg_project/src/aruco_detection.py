#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2.aruco as aruco
import tf

class MarkerFinder:
    def __init__(self):
        self.camera_matrix = None
        self.dist_coeffs = None
        self.aruco_dict = None
        self.parameters = aruco.DetectorParameters_create()
        self.markers = []
        self.rgb_image = None  # Salva l'immagine RGB

    def marker_param(self, marker_size, aruco_dict_type):
        self.aruco_dict = aruco.Dictionary_get(getattr(aruco, aruco_dict_type))
        self.marker_size = marker_size

    def detect_markers_poses(self, rgb_image):
        self.rgb_image = rgb_image  # Salva l'immagine RGB
        gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        
        self.markers = []

        if ids is not None:
            for i, corner in enumerate(corners):
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corner, self.marker_size, self.camera_matrix, self.dist_coeffs)
                position = tvec[0][0]
                rotation, _ = cv2.Rodrigues(rvec[0][0])
                quaternion = tf.transformations.quaternion_from_matrix(rotation)
                
                marker_info = {
                    'id': int(ids[i]),
                    'position': position,
                    'rotation': quaternion
                }
                self.markers.append(marker_info)
                
                aruco.drawDetectedMarkers(rgb_image, corners, ids)
                aruco.drawAxis(rgb_image, self.camera_matrix, self.dist_coeffs, rvec[0], tvec[0], self.marker_size / 2)

def image_callback(msg, marker_finder):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        marker_finder.detect_markers_poses(cv_image)
        print("Immagine ricevuta.")  # Debug
    except CvBridgeError as e:
        rospy.logerr(f"Errore di CvBridge: {e}")

def show_image(marker_finder):
    while not rospy.is_shutdown():
        if marker_finder.rgb_image is not None:
            cv2.imshow("Robot Camera Feed", marker_finder.rgb_image)
            cv2.waitKey(1)

def main():
    rospy.init_node('marker_detection_node', anonymous=True)
    marker_finder = MarkerFinder()
    marker_size = 0.1
    aruco_dict_type = 'DICT_4X4_50'
    marker_finder.marker_param(marker_size, aruco_dict_type)
    rospy.Subscriber("/camera/rgb/image_raw", Image, lambda msg: image_callback(msg, marker_finder))

    # Avvia il thread per la visualizzazione dell'immagine
    threading.Thread(target=show_image, args=(marker_finder,)).start()

    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
