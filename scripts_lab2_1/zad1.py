#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32, String

def even_numbers_publisher():
    # Инициализация узла
    rospy.init_node('even_numbers_publisher', anonymous=True)
    
    # Получение имени узла и пространства имен для логирования
    node_name = rospy.get_name()
    namespace = rospy.get_namespace()
    
    # Создание публикаторов
    pub_even = rospy.Publisher('even_numbers', Int32, queue_size=10)
    pub_overflow = rospy.Publisher('overflow_topic', String, queue_size=10)
    
    # Частота 10 Гц
    rate = rospy.Rate(10)
    
    # Счетчик четных чисел
    even_number = 0
    
    rospy.loginfo("Controller %s started in namespace %s. Publishing at 10 Hz...", 
                  node_name, namespace)
    
    while not rospy.is_shutdown():
        # Публикация четного числа
        pub_even.publish(even_number)
        
        # Логируем только каждое 10-е число для уменьшения спама
        if even_number % 20 == 0:
            rospy.loginfo("[%s] Published: %d", node_name, even_number)
        
        # Проверка на переполнение (достигли 100)
        if even_number >= 100:
            overflow_msg = "Counter overflow in {}! Resetting from {} at time {}".format(
                node_name, even_number, rospy.get_time())
            pub_overflow.publish(overflow_msg)
            rospy.logwarn("[%s] %s", node_name, overflow_msg)
            
            # Сброс счетчика
            even_number = 0
        else:
            # Увеличение на 2 для следующего четного числа
            even_number += 2
        
        # Ожидание для поддержания частоты 10 Гц
        rate.sleep()

if __name__ == '__main__':
    try:
        even_numbers_publisher()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS interrupt exception")
    except KeyboardInterrupt:
        rospy.loginfo("Node stopped by user")
