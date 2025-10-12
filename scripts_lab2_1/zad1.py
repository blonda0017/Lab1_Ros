#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32, String

def even_numbers_publisher():
    # Инициализация узла
    rospy.init_node('even_numbers_publisher', anonymous=True)
    
    # Создание публикаторов
    pub_even = rospy.Publisher('even_numbers', Int32, queue_size=10)
    pub_overflow = rospy.Publisher('overflow_topic', String, queue_size=10)
    
    # Частота 10 Гц
    rate = rospy.Rate(10)
    
    # Счетчик четных чисел
    even_number = 0
    
    rospy.loginfo("Even numbers publisher started. Publishing at 10 Hz...")
    
    while not rospy.is_shutdown():
        # Публикация четного числа
        pub_even.publish(even_number)
        rospy.loginfo("Published: %d", even_number)
        
        # Проверка на переполнение (достигли 100)
        if even_number >= 100:
            overflow_msg = "Counter overflow! Resetting from {} at time {}".format(
                even_number, rospy.get_time())
            pub_overflow.publish(overflow_msg)
            rospy.logwarn(overflow_msg)
            
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
