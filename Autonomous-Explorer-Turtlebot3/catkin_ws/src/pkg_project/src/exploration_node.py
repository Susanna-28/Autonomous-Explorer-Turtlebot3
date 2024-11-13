#!/usr/bin/env python
import rospy
import actionlib
import random
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
import pandas as pd
from time import sleep, time
import threading

class AutonomousExplorer:
    def __init__(self):
        rospy.init_node('autonomous_explorer')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Attesa di connessione a move_base...")
        self.client.wait_for_server()
        rospy.loginfo("Collegato a move_base.")

        # Limiti della mappa
        self.x_min = -9.8
        self.x_max = 9.8
        self.y_min = -4.8
        self.y_max = 4.8

        self.VISITED_THRESHOLD = 0.2  # Threshold to mark an area as visited (distance)
        self.map_data = None  # Dati della mappa occupata
        self.explored_directions = []  # Track explored directions
        self.total_cells = 80000  # Totale delle celle nella mappa
        self.explored_cells = 0  # Celle esplorate
        self.positions = []  # Lista per salvare tutte le posizioni

        # Tempo iniziale per il salvataggio periodico
        self.last_save_time = time()

        # Subscribe al topic odom (o usa /amcl_pose per pose stimate)
        self.current_pose = None
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Aggiungi il Subscriber al topic /map con self.map_callback
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Publisher for commanding the robot's velocity
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Initialize velocity command
        self.cmd_vel = Twist()

        self.explore()

    def odom_callback(self, msg):
        """Callback per ottenere la posizione corrente del robot"""
        self.current_pose = msg.pose.pose

        # Crea un blocco globale per proteggere l'accesso alla lista delle posizioni
        self.save_lock = threading.Lock()
    
    def save_positions_to_excel(self):
        with self.save_lock:
            # Filtra solo le tuple valide (con due valori)
            self.valid_positions = [pos for pos in self.positions if len(pos) == 2]
            if self.valid_positions:  # Salva solo se ci sono dati validi
                df = pd.DataFrame(self.valid_positions, columns=['x', 'y'])
                try:
                    df.to_excel('/root/ws/progetto/catkin_ws/src/pkg_project/positions_autonomous_exploration.xlsx', index=False)
                    rospy.loginfo("Posizioni salvate in positions_autonomous_exploration.xlsx")
                except Exception as e:
                    rospy.logerr(f"Errore durante il salvataggio del file Excel: {e}")


    # Callback per i dati della mappa occupata
    def map_callback(self, msg):
        # Inizializza o aggiorna il conteggio delle celle
        self.grid_data = msg.data  # Dati della mappa (ogni cella può avere valori come -1, 0, o 100)
        self.explored_cells = sum(1 for cell in self.grid_data if cell == 0)  # Conta le celle libere (valore 0)

    def is_within_map_bounds(self, x, y):
        """Controlla se la posizione è all'interno dei limiti della mappa"""
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max

    def explore(self):
        # Aggiungi un breve ritardo per dare tempo all'aggiornamento della posizione
        rospy.sleep(1)

        while not rospy.is_shutdown():
            # Genera un punto casuale per l'esplorazione
            x = random.uniform(self.x_min, self.x_max)
            y = random.uniform(self.y_min, self.y_max)
            theta = 0.0  # Orientamento neutro

            # Verifica che il goal sia all'interno dei limiti della mappa
            if self.is_within_map_bounds(x, y):
                rospy.loginfo(f"Generato nuovo obiettivo: x={x}, y={y}")
                self.move_to_goal(x, y, theta)
            else:
                rospy.logwarn("Obiettivo fuori dai confini! Selezionando una nuova destinazione...")

            if time() - self.last_save_time > 20:  # Controlla ogni 20 secondi
                self.save_positions_to_excel()
                self.last_save_time = time()

    def stop_robot(self):
        '''
        Stop the robot by setting velocity commands to zero.
        '''
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.vel_pub.publish(self.cmd_vel)
    
    def monitor_robot_position(self):
        if self.current_pose:
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            if not self.is_within_map_bounds(current_x, current_y):
                rospy.logwarn("Robot vicino ai confini! Fermando il movimento...")
                self.client.cancel_goal()

            if current_x is not None and current_y is not None:
                with self.save_lock:
                    self.positions.append((current_x, current_y))
                self.print_exploration_stats()  # Stampa la percentuale di area esplorata
                self.stop_robot_if_needed()  # Ferma il robot se necessario

    def stop_robot_if_needed(self):
        if self.total_cells > 0:
            percentage_explored = (self.explored_cells / self.total_cells) * 100
            if percentage_explored >= 85:
                self.client.cancel_goal()  # Cancella l'obiettivo corrente
                self.stop_robot()  # Imposta la velocità a zero
                rospy.loginfo("Robot fermato perché la percentuale di area esplorata ha superato il 85%.")



    def move_to_goal(self, x, y, theta):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        goal.target_pose.pose.orientation.w = 1.0  # Orientamento neutro
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0

        rospy.loginfo(f"Inviando obiettivo: {goal.target_pose.pose.position}")

        self.client.send_goal(goal)

        # Controllo costante della posizione durante il movimento
        while self.client.get_state() in [actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PENDING]:
            self.monitor_robot_position()
            rospy.sleep(0.1)  # Ritardo per evitare sovraccarico del log

        self.client.wait_for_result()
        rospy.loginfo("Obiettivo raggiunto.")

    def print_exploration_stats(self):
        # Calcola la percentuale di area esplorata
        if self.total_cells > 0:
            self.percentage_explored = (self.explored_cells / self.total_cells) * 100
            rospy.loginfo(f"Percentuale area esplorata: {self.percentage_explored:.2f}% ({self.explored_cells}/{self.total_cells})")

if __name__ == '__main__':
    try:
        AutonomousExplorer()
    except rospy.ROSInterruptException:
        pass
