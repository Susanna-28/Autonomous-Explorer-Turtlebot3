import rospy
import actionlib
import random
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
 
 
class AutonomousExplorer:
    def __init__(self):
 
        # Inizializzazione della classe senza chiamare rospy.init_node()
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Attesa di connessione a move_base...")
        self.client.wait_for_server()
        rospy.loginfo("Collegato a move_base.")
        # Limiti della mappa
        self.x_min = -9.8
        self.x_max = 9.8
        self.y_min = -4.8
        self.y_max = 4.8
 
        # Subscribe al topic odom (o usa /amcl_pose per pose stimate)
        self.current_pose = None
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_vel= None
        self.cmd_vel_pub=rospy.Publisher('/cmd_vel',Twist, queue_size=10)
        self.linearspeed=0.1
        self.angularspeed=0.08
        self.explore()
 
    def odom_callback(self, msg):
        """Callback per ottenere la posizione corrente del robot"""
        self.current_pose = msg.pose.pose
 
    def is_within_map_bounds(self, x, y):
        """Controlla se la posizione è all'interno dei limiti della mappa"""
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max
 
    def explore(self):
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
 
    def monitor_robot_position(self):
        """Monitora costantemente la posizione del robot mentre si muove"""
        if self.current_pose:
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            if not self.is_within_map_bounds(current_x, current_y):
                rospy.logwarn("Robot vicino ai confini! Fermando il movimento...")
                self.client.cancel_goal()
 
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
            # Imposta velocità di movimento del robot
            twist_msg = Twist()
            twist_msg.linear.x = self.linearspeed  # Imposta velocità lineare
            twist_msg.angular.z = self.angularspeed  # Imposta velocità angolare
            self.cmd_vel_pub.publish(twist_msg)  
            # Pubblica il messaggio su /cmd_vel
            rospy.sleep(0.1)
        self.client.wait_for_result()
        rospy.loginfo("Obiettivo raggiunto.")
