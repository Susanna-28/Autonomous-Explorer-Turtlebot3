#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
import math
import pandas as pd
from time import sleep, time
from random import random
import threading
 
# Initialize ROS node
rospy.init_node('turtlebot3_autonomous_navigation')
 
# Publisher for commanding the robot's velocity
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
 
# Initialize velocity command
cmd_vel = Twist()
 
# Constants for robot behavior
REVOLVE_TIME = 4  # Time to complete a full rotation (360 degrees)
STOP_THRESHOLD = 5  # Time threshold for detecting if the robot is stuck
VISITED_THRESHOLD = 0.2  # Threshold to mark an area as visited (distance)
 
# Global variables for sensor data
scan_data = None
odom_data = None
map_data = None  # Dati della mappa occupata
last_movement_time = time()  # Time when the robot last moved
last_position = None  # Previous position of the robot
explored_directions = []  # Track explored directions
total_cells = 80000  # Totale delle celle nella mappa
explored_cells = 0  # Celle esplorate
positions = []  # Lista per salvare tutte le posizioni
# Tempo iniziale per il salvataggio periodico
last_save_time = time()
 
# Callback function for LIDAR data
def lidar_callback(msg):
    global scan_data
    scan_data = msg.ranges
 
# Callback function for odometry data
def odom_callback(msg):
    global odom_data, last_movement_time, last_position
 
    odom_data = msg
    current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
   
    # Verifica che entrambi i valori x e y non siano None e usa il blocco per l'aggiunta
    if current_position[0] is not None and current_position[1] is not None:
        with save_lock:
            positions.append(current_position)
 
    if last_position is None:
        last_position = current_position
    else:
        # Check if the robot has moved significantly
        distance_moved = math.sqrt(
            (current_position[0] - last_position[0])**2 +
            (current_position[1] - last_position[1])**2
        )
        if distance_moved > 0.01:  # Robot has moved
            last_movement_time = time()  # Update the time when the robot last moved
            last_position = current_position
 
# Crea un blocco globale per proteggere l'accesso alla lista delle posizioni
save_lock = threading.Lock()
 
def save_positions_to_excel():
    # Usa il blocco per impedire modifiche a 'positions' durante il salvataggio
    with save_lock:
        # Filtra solo le tuple valide (con due valori)
        valid_positions = [pos for pos in positions if len(pos) == 2]
        if valid_positions:  # Salva solo se ci sono dati validi
            df = pd.DataFrame(valid_positions, columns=['x', 'y'])
            df.to_excel('/root/ws/progetto/catkin_ws/src/pkg_project/positions_lawn_mower.xlsx', index=False)
            rospy.loginfo("Posizioni salvate in positions_lawn_mower.xlsx")
 
# Callback per i dati della mappa occupata
def map_callback(msg):
    global total_cells, explored_cells
    # Inizializza o aggiorna il conteggio delle celle
    grid_data = msg.data  # Dati della mappa (ogni cella può avere valori come -1, 0, o 100)
    explored_cells = sum(1 for cell in grid_data if cell == 0)  # Conta le celle libere (valore 0)
 
rospy.Subscriber('/map', OccupancyGrid, map_callback)
 
# Subscribe to LIDAR and Odometry topics
rospy.Subscriber('/scan', LaserScan, lidar_callback)
rospy.Subscriber('/odom', Odometry, odom_callback)
 
def get_min_distance():
    '''
    Process LIDAR data to find the minimum distance to an obstacle.
    '''

    if scan_data:
        return min(scan_data)  # Return the closest obstacle detected by the LIDAR
    else:
        return float('inf')  # If no data, assume no obstacles
 
def stop_robot():
    '''
    Stop the robot by setting velocity commands to zero.
    '''
    cmd_vel.linear.x = 0.0
    cmd_vel.angular.z = 0.0
    vel_pub.publish(cmd_vel)
 
def move_forward():
    '''
    Move the robot forward with a constant speed.
    '''
    cmd_vel.linear.x = 0.2  # Move forward at a speed of 0.2 m/s
    cmd_vel.angular.z = 0.0
    vel_pub.publish(cmd_vel)
 
def turn_right():
    '''
    Turn the robot to the right (clockwise).
    '''
    cmd_vel.linear.x = 0.0
    cmd_vel.angular.z = -0.5  # Turn right at a rate of 0.5 rad/s
    vel_pub.publish(cmd_vel)
 
def turn_left():
    '''
    Turn the robot to the left (counter-clockwise).
    '''
    cmd_vel.linear.x = 0.0
    cmd_vel.angular.z = 0.5  # Turn left at a rate of 0.5 rad/s
    vel_pub.publish(cmd_vel)
 
def random_spin():
    '''
    Spin the robot randomly to introduce variability in navigation.
    '''
    time_to_spin = max(random(), .2) * REVOLVE_TIME
    turn_right()
    sleep(time_to_spin)
    stop_robot()
 
def has_obstacle_front():
    '''
    Check if there is an obstacle in front of the robot.
    '''
    if scan_data:
        front_range = scan_data[0:10] + scan_data[-10:]  # Check forward angles
        return min(front_range) < VISITED_THRESHOLD  # Adjusted threshold
    return False
 
def has_obstacle_left():
    '''
    Check if there is an obstacle to the left of the robot.
    '''
    if scan_data:
        left_range = scan_data[60:120]  # Check left angles
        return min(left_range) < VISITED_THRESHOLD  # Adjusted threshold
    return False
 
def has_obstacle_right():
    '''
    Check if there is an obstacle to the right of the robot.
    '''
    if scan_data:
        right_range = scan_data[240:300]  # Check right angles
        return min(right_range) < VISITED_THRESHOLD  # Adjusted threshold
    return False
 
def has_free_space():
    '''
    Check if there is free space in front of the robot.
    '''
    if scan_data:
        front_range = scan_data[0:10] + scan_data[-10:]  # Check forward angles
        return min(front_range) > VISITED_THRESHOLD  # Adjusted threshold
    return False
 
def avoid_obstacle():
    '''
    Behavior for avoiding obstacles detected by the LIDAR.
    '''
    obstacle_front = has_obstacle_front()
    obstacle_left = has_obstacle_left()
    obstacle_right = has_obstacle_right()
 
    rospy.loginfo(f"Obstacle Front: {obstacle_front}, Left: {obstacle_left}, Right: {obstacle_right}")
 
    if obstacle_front:
        # Check the explored directions
        if not obstacle_left and 'left' not in explored_directions:
            turn_left()  # Turn left if no obstacles and not explored
            explored_directions.append('left')
        elif not obstacle_right and 'right' not in explored_directions:
            turn_right()  # Turn right if no obstacles and not explored
            explored_directions.append('right')
        else:
            # If both sides are blocked or explored, spin randomly
            random_spin()
    else:
        # If there are no obstacles, move forward
        move_forward()
        explored_directions.clear()  # Reset explored directions when moving forward
 
def is_robot_stuck():
    '''
    Check if the robot has been stuck (no movement) for more than STOP_THRESHOLD seconds.
    '''
    return (time() - last_movement_time) > STOP_THRESHOLD
 
def handle_stuck_robot():
    '''
    Behavior to handle the robot being stuck for too long.
    It makes the robot spin or move.
    '''
    rospy.loginfo("Robot is stuck, trying to recover...")
   
    # Check explored directions and decide a fallback action
    if 'left' in explored_directions and 'right' in explored_directions:
        # If both sides are explored, it can move backward or spin
        turn_right()  # Just an example; you can implement backward movement too
    else:
        random_spin()  # Spin randomly to change direction
 
def print_exploration_stats():
    # Calcola la percentuale di area esplorata
    if total_cells > 0:
        percentage_explored = (explored_cells / total_cells) * 100
        rospy.loginfo(f"Percentuale area esplorata: {percentage_explored:.2f}% ({explored_cells}/{total_cells})")
        

navigation_active = True  # Variabile per attivare/disattivare la navigazione

def stop_robot_if_needed():
    global navigation_active
    if total_cells > 0:
        percentage_explored = (explored_cells / total_cells) * 100
        rospy.loginfo(f"Percentuale esplorata: {percentage_explored}%")  # Log per debug
        if percentage_explored >= 85 and navigation_active:
            stop_robot()
            rospy.loginfo("Robot fermato perché la percentuale di area esplorata ha superato l'85%.")
            navigation_active = False  # Disattiva la navigazione una volta fermato
        elif not navigation_active:
            stop_robot()  # Assicura che il robot continui a rimanere fermo

 
def main():
    '''
    Main loop for autonomous navigation.
    The robot navigates based on LIDAR and odometry readings.
    '''
    rate = rospy.Rate(10)  # 10 Hz loop rate
    global last_save_time

    try:
        while not rospy.is_shutdown():
            stop_robot_if_needed()  # Controlla la percentuale esplorata e ferma il robot se necessario

            if navigation_active:  # Esegui la navigazione solo se attiva
                min_dist = get_min_distance()
                print_exploration_stats()

                if has_obstacle_front():  # Se c'è un ostacolo davanti
                    avoid_obstacle()       # Evita l'ostacolo
                elif has_free_space():     # Se c'è spazio libero
                    move_forward()         # Vai avanti

                # Verifica se il robot è bloccato
                if is_robot_stuck():
                    handle_stuck_robot()

                # Salva periodicamente le posizioni ogni 20 secondi
                if time() - last_save_time > 20:
                    save_positions_to_excel()
                    last_save_time = time()
            else:
                # Pubblica sempre il comando stop se la navigazione non è attiva
                stop_robot()

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
 
if __name__ == "__main__":
 
    main()
