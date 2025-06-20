#!/usr/bin/env python3
import rospy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
import time

# Глобальные переменные
current_state = None
drone_pose = None

def state_callback(msg):
    global current_state
    current_state = msg

def pose_callback(msg):
    global drone_pose
    drone_pose = msg

def main():
    # Инициализация ноды
    rospy.init_node('takeoff_node')
    
    # Подписываемся на состояние дрона
    state_sub = rospy.Subscriber('/mavros/state', State, state_callback)
    pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
    
    # Публикатор целевой позиции
    local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', 
                                     PoseStamped, queue_size=10)
    
    # Сервисные клиенты
    rospy.loginfo("Waiting for services...")
    rospy.wait_for_service('/mavros/cmd/arming')
    rospy.wait_for_service('/mavros/set_mode')
    rospy.wait_for_service('/mavros/cmd/takeoff')
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
    
    # Частота цикла
    rate = rospy.Rate(10)
    
    # Ожидаем соединения с MAVROS
    rospy.loginfo("Waiting for connection...")
    while not rospy.is_shutdown() and (current_state is None or not current_state.connected):
        rate.sleep()
    
    rospy.loginfo("Connected to drone!")
    
    # Создаем сообщение для целевой позиции - ОЧЕНЬ ВАЖНО
    # публиковать setpoint перед переключением в GUIDED
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.position.x = 1
    pose.pose.position.y = -1
    pose.pose.position.z = 1
    
    # КРИТИЧНО: публикуем setpoints некоторое время перед 
    # переключением режима и армированием
#    rospy.loginfo("Publishing initial setpoints...")
#    for i in range(3):  # ≈5 секунд при 20 Гц
#        if rospy.is_shutdown():
#            return
#        local_pos_pub.publish(pose)
#        rate.sleep()
    
    # Переключаемся в режим GUIDED
    rospy.loginfo("Setting GUIDED mode...")
    for i in range(1):  # Несколько попыток
        if current_state.mode != "GUIDED":
            result = set_mode_client(0, "GUIDED")
            if result.mode_sent:
                rospy.loginfo("GUIDED mode sent")
            rospy.sleep(1)
        else:
            break
    
    if current_state.mode != "GUIDED":
        rospy.logwarn("Failed to set GUIDED mode! Current mode: %s", current_state.mode)
        return
    else:
        rospy.loginfo("GUIDED mode confirmed")
    
    # Армируем дрон
    rospy.loginfo("Arming drone...")
    for i in range(1):  # Несколько попыток
        if not current_state.armed:
            result = arming_client(True)
            if result.success:
                rospy.loginfo("Arming command sent")
            rospy.sleep(1)
        else:
            break
            
    if not current_state.armed:
        rospy.logwarn("Failed to arm the drone!")
        return
    else:
        rospy.loginfo("Drone armed confirmed")
    
    # Важно: после армирования нужно немного подождать
    rospy.sleep(2)
    
    # Используем ПЕРВЫЙ метод: takeoff service
    rospy.loginfo("Trying takeoff service...")
    takeoff_result = takeoff_client(0, 0, 0, 0, 1)  
    if takeoff_result.success:
        rospy.loginfo("Takeoff service accepted!")
    else:
        rospy.logwarn("Takeoff service failed, using position setpoint method")
    
    
    rospy.sleep(15)    
    # Продолжаем публиковать целевую высоту независимо от того,
    # сработал ли сервис takeoff
    rospy.loginfo("Publishing setpoint...")
    local_pos_pub.publish(pose)
    
    start_time = time.time()
    while not rospy.is_shutdown(): 
	
        #rospy.loginfo("xyz:", drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z)
        
        # Проверяем, достигли ли высоты
        if drone_pose and abs(drone_pose.pose.position.z - 2.0) < 0.1:
            rospy.loginfo("Target altitude reached!")
            break
            
        # Проверяем таймаут
        if time.time() - start_time > 5:
            rospy.logwarn("Takeoff timeout, but continuing...")
            break
            
        rate.sleep()
    
    # Продолжаем удерживать высоту
    rospy.loginfo("Hovering for 10 seconds...")
    hover_start = time.time()
    while not rospy.is_shutdown() and time.time() - hover_start < 10:
        local_pos_pub.publish(pose)
        rate.sleep()
    
    rospy.loginfo("Takeoff mission completed!")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
