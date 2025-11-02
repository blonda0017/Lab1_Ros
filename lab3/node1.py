#!/usr/bin/env python3 
import rospy #Импорт основной ROS библиотеки для Python
from std_msgs.msg import String #Импорт типа сообщения String из стандартных ROS сообщений. 
                      #Объявление основной функции узла. Содержит всю логику работы ROS узла.
def node1():
    rospy.init_node('node1') #Инициализация узла, Регистрирует узел в ROS Master, Устанавливает связь с другими узлами
    pub = rospy.Publisher('chatter', String, queue_size=10) #Создание публикатора: имя топика, тип сообщения, размер очереди сообщений (буфер)
    rate = rospy.Rate(1)  # 1Hz
    
    while not rospy.is_shutdown(): #Главный цикл узла
        msg = "Hello from node1" #Создание сообщения
        pub.publish(msg) #убликация сообщения в топик 'chatter'. Все подписчики на этот топик получат это сообщение.
        rospy.loginfo(msg) #Отображается в консоли
        rate.sleep() #риостановка выполнения на время 1Hz

if __name__ == '__main__': #выполняется только если скрипт запущен напрямую
    try:
        node1() #апуск основной функции узла в блоке try для перехвата исключений.
    except rospy.ROSInterruptException:
        pass #авершении узла (например, Ctrl+C). Блок pass означает "ничего не делать" - корректно завершить работу.
