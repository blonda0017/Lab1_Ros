#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, Float64

class SummingNode:
    def __init__(self):
        rospy.init_node('summing_node')
        
        # Подписываемся на обработанные числа
        self.processed_sub = rospy.Subscriber('/processed_numbers', Float64MultiArray, self.processed_callback)
        
        # Публикуем результат суммы
        self.result_pub = rospy.Publisher('/calculation_result', Float64, queue_size=10)
        
        rospy.loginfo("Summing node started")
        
    def processed_callback(self, msg):
        """Суммирует все числа в массиве"""
        numbers = msg.data
        total_sum = sum(numbers)
        
        # Публикуем результат
        result_msg = Float64()
        result_msg.data = total_sum
        
        self.result_pub.publish(result_msg)
        rospy.loginfo("Sum calculated: %s = %.2f", numbers, total_sum)

if __name__ == '__main__':
    try:
        SummingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
