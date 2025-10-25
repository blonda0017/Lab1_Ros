#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, Float64

class PolynomialNode:
    def __init__(self):
        rospy.init_node('polynomial_node')
        
        # Подписываемся на входные числа
        self.input_sub = rospy.Subscriber('/input_numbers', Float64MultiArray, self.input_callback)
        
        # Публикуем обработанные числа
        self.output_pub = rospy.Publisher('/processed_numbers', Float64MultiArray, queue_size=10)
        
        rospy.loginfo("Polynomial node started")
        
    def input_callback(self, msg):
        """Обрабатывает входные числа: возводит в степень по позиции"""
        numbers = msg.data
        
        if len(numbers) != 3:
            rospy.logwarn("Expected 3 numbers, got %d", len(numbers))
            return
            
        # Возводим в степень: первое число^1, второе^2, третье^3
        processed_numbers = [
            numbers[0] ** 1,  # x^1
            numbers[1] ** 2,  # x^2  
            numbers[2] ** 3   # x^3
        ]
        
        # Публикуем обработанные числа
        output_msg = Float64MultiArray()
        output_msg.data = processed_numbers
        
        self.output_pub.publish(output_msg)
        rospy.loginfo("Processed numbers: %s -> %s", numbers, processed_numbers)

if __name__ == '__main__':
    try:
        PolynomialNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
