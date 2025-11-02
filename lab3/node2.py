#!/usr/bin/env python3
import rospy #основная библиотека для узлов
from std_msgs.msg import String #Импорт типа сообщения - строковое сообщение ROS

def callback(data): #Функция обработки - вызывается при получении сообщения
    rospy.loginfo(f"node2 received: {data.data}") #вывод полученного сообщения в консоль

def node2(): #содержит логику узла
    rospy.init_node('node2') #регистрация в ROS с именем 'node2'
    rospy.Subscriber('chatter', String, callback) #Создание подписчика - слушает топик 'chatter', тип String, вызывает callback
    rospy.spin() #поддерживает узел активным, обрабатывает сообщения

if __name__ == '__main__': #Проверка запуска - выполняется только при прямом запуске
    node2() #Запуск узла - вызов основной функции
