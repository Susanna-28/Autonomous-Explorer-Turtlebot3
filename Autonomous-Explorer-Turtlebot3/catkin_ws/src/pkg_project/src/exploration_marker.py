#!/usr/bin/env python3
import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from exploration_node_marker import AutonomousExplorer
import numpy as np
import os
import rospkg
import tf
 
class ArUcoDetector:
    def __init__(self):
        # Inizializza il bridge per convertire tra ROS e OpenCV
        self.bridge = CvBridge()
        self.tempo= None
        # Parametri di calibrazione della fotocamera
        self.image_width = 640
        self.image_height = 480
        self.camera_name = "camera"
        self.camera_matrix = np.array([[530.4669406576809, 0.0, 320.5],
                                [0.0, 530.4669406576809, 240.5],
                                [0.0, 0.0, 1.0]], dtype=np.float32)
 
        self.dist_coeffs = np.zeros((5,), dtype=np.float32)  # Assicurati che sia un array numpy di tipo float32
        self.rectification_matrix = np.eye(3)  # Matrice di rettificazione
        self.projection_matrix = np.array([
            [329.2483825683594, 0, 198.4101510452074, 0],
            [0, 329.1044006347656, 155.5057121208347, 0],
            [0, 0, 1, 0]
        ])
 
        # Dizionario delle posizioni attese per ciascun marker
        self.posizioni_attese_marker = {
            0: (-7.2, 3, 0),
            1: (-2.7, 3.75, 0),
            2: (-2.8, 1, 0),
            3: (-2.8,-1, 0),
            4: (-4.35, -3, 0),
            5: (3.95, 0.5, 0),
            6: (1.5, 3.4, 0),
            7: (3.5, -3.1, 0)
        }
 
        self.errore = None
        self.marker_position_map = None 
       
        # TF listener per le trasformazioni
        self.tf_listener = tf.TransformListener()
 
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
 
        # Publisher per il movimento del robot
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
 
        # Subscriber per ottenere la posizione e l'orientamento del robot
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.current_pose = None  # Variabile per memorizzare la posizione corrente del robot
 
        # Dati per i marker ArUco
        self.marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()
 
        # Soglia di distanza per il movimento
        self.distance_threshold = 0.8  # Adatta questo valore in base al tuo robot
 
        # Set per tenere traccia dei marker già salvati
        self.saved_markers = set()
 
        # Percorso della directory per salvare le immagini dei marker
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('pkg_project')  # Ottieni il percorso del pacchetto
        self.image_directory = os.path.join(pkg_path, 'marker_images')
        if not os.path.exists(self.image_directory):
            os.makedirs(self.image_directory)
 
    def odom_callback(self, data):
        # Aggiorna la posizione e l'orientamento corrente del robot
        self.current_pose = data.pose.pose
 
    def callback(self, data):
        try:
            # Converti il messaggio ROS in un'immagine OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
 
            # Rileva i marker ArUco
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, self.marker_dict, parameters=self.parameters)
 
            if ids is not None and len(ids) > 0:
                # Calcola la posa per ogni marker
                for i in range(len(ids)):
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.3, self.camera_matrix, self.dist_coeffs)
 
                    # Assicurati che tvec abbia i dati necessari
                    if tvec.size > 0:
                        # Trasformare la posizione del marker in coordinate della mappa
                        self.marker_position_map = self.transform_marker_to_map(tvec[0][0])  # Usare tvec[0][0] per accedere correttamente
                        if self.marker_position_map:
                            rospy.loginfo(f"Posizione del marker rispetto alla mappa: {self.marker_position_map}")
                            # Calcola l'errore rispetto alla posizione attesa
                            self.errore = self.calcola_errore(int(ids[i]), self.marker_position_map)
                            rospy.loginfo(f"Errore di posizione per marker {int(ids[i])}: {self.errore} metri")

                    # Aggiungi l'ID del marker come testo sull'immagine
                    corner = corners[i][0][0]  # Usa la posizione del primo punto del marker per posizionare il testo
                    marker_id = str(ids[i][0])  # Converte l'ID in una stringa
                    cv2.putText(cv_image, marker_id, (int(corner[0]), int(corner[1] - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                self.tempo = rospy.Time.now()
                rospy.loginfo(f"Marker IDs rilevati: {ids}")
                self.save_marker_data(corners, ids, cv_image)
                aruco.drawDetectedMarkers(cv_image, corners, ids)
                # Mostra l'immagine con i marker rilevati
                cv2.imshow("ArUco Markers", cv_image)
                cv2.waitKey(1)
 
        except Exception as e:
            rospy.logerr(f"Errore nel callback: {e}")
 
 
    def save_marker_data(self, corners, ids, cv_image):
        # Verifica se il marker è già stato salvato
        if int(ids[0]) not in self.saved_markers:
            # Aggiungi il marker al set di marker già salvati
            self.saved_markers.add(int(ids[0]))
 
            # Salva l'immagine del marker nella directory specificata
            image_filename = os.path.join(self.image_directory, f"marker_{ids[0]}.png")
            cv2.imwrite(image_filename, cv_image)
            rospy.loginfo(f"Immagine del marker salvata in: {image_filename}")
 
            # Salva le informazioni sulla posizione del marker e la posizione del robot
            if self.current_pose is not None:
                rospy.loginfo(f"Salvataggio posizione marker ID: {ids[0]}")
                rospy.loginfo(f"Posizione del marker: {self.marker_position_map}")
                rospy.loginfo(f"Tempo di rilevazione: {self.tempo.to_sec()}")
                rospy.loginfo(f"Errore: {self.errore}")
                
                # Salva queste informazioni su un file
                with open(os.path.join(self.image_directory, "marker_positions.txt"), "a") as f:
                    f.write(f"Marker ID: {ids[0]}\n")
                    f.write(f"Posizione del marker: {self.marker_position_map}\n")
                    f.write(f"Immagine salvata in: {image_filename}\n")
                    f.write(f"Tempo: {self.tempo.to_sec()}\n")
                    f.write(f"Errore: {self.errore}\n")
                    f.write("\n")
 
 
    def transform_marker_to_map(self, tvec):
        try:
            # Crea un messaggio PointStamped per la posizione del marker
            point_camera = PointStamped()
            point_camera.header.frame_id = "camera_rgb_optical_frame"
            point_camera.header.stamp = rospy.Time(0)
            point_camera.point.x = tvec[0]  # Usa tvec per la posizione x
            point_camera.point.y = tvec[1]  # Usa tvec per la posizione y
            point_camera.point.z = tvec[2]  # Usa tvec per la posizione y
 
            # Ottieni la trasformazione da camera_rgb_frame a map
            self.tf_listener.waitForTransform("map", "camera_rgb_optical_frame", rospy.Time(0), rospy.Duration(4.0))
            point_map = self.tf_listener.transformPoint("map", point_camera)
 
            return (point_map.point.x, point_map.point.y, point_map.point.z)
 
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"Errore nella trasformazione: {e}")
        return None
   
 
    def calcola_errore(self, ids, marker_position_map):
        # Controlla se l'ID del marker è nel dizionario delle posizioni attese
        if ids in self.posizioni_attese_marker:
            # Ottieni la posizione attesa per il marker
            posizione_attesa = self.posizioni_attese_marker[ids]
           
            # Calcola l'errore come differenza tra posizione attesa e rilevata
            errore_x = marker_position_map[0] - posizione_attesa[0]
            errore_y = marker_position_map[1] - posizione_attesa[1]
            errore=[errore_x,errore_y]
 
            return errore
        else:
            rospy.logwarn(f"Posizione attesa per marker ID {ids} non trovata.")
            return None
 
 
if __name__ == "__main__":
    rospy.init_node('aruco_detector', anonymous=True)
    detector = ArUcoDetector()
    explorer = AutonomousExplorer()  # Inizializza qui per mantenere la logica separata
    rospy.spin()
