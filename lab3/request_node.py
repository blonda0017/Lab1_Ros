#!/usr/bin/env python3
import rospy
import sys
from std_msgs.msg import Float64MultiArray, Float64

class RequestNode:
    def __init__(self, numbers):
        rospy.init_node('request_node', anonymous=True)
        self.numbers = numbers
        self.result = None
        self.result_received = False
        
        # Подписываемся на результат
        self.result_sub = rospy.Subscriber('/calculation_result', Float64, self.result_callback)
        
        # Публикуем входные числа
        self.input_pub = rospy.Publisher('/input_numbers', Float64MultiArray, queue_size=10)
        
    def result_callback(self, msg):
        """Получает результат вычислений"""
        self.result = msg.data
        self.result_received = True
        rospy.loginfo("Result received: %.2f", self.result)
        
    def send_request(self):
        """Отправляет запрос и ждет ответ"""
        # Ждем подключения к топику результатов
        rospy.sleep(1)
        
        # Отправляем входные числа
        input_msg = Float64MultiArray()
        input_msg.data = self.numbers
        
        self.input_pub.publish(input_msg)
        rospy.loginfo("Sent numbers: %s", self.numbers)
        
        # Ждем результат (таймаут 5 секунд)
        timeout = rospy.Time.now() + rospy.Duration(5)
        while not self.result_received and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
            
        if self.result_received:
            print(f"\n=== RESULT ===")
            print(f"Input numbers: {self.numbers}")
            print(f"Calculation: {self.numbers[0]}^1 + {self.numbers[1]}^2 + {self.numbers[2]}^3")
            print(f"Result: {self.result}")
            print("==============\n")
            return self.result
        else:
            rospy.logerr("Timeout waiting for result")
            return None

if __name__ == '__main__':
    if len(sys.argv) != 4:
        print("Usage: rosrun super_blonda_study_pkg request_node.py <num1> <num2> <num3>")
        sys.exit(1)
        
    try:
        numbers = [float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])]
        
        node = RequestNode(numbers)
        result = node.send_request()
        
        if result is not None:
            sys.exit(0)
        else:
            sys.exit(1)
            
    except ValueError:
        print("Error: All arguments must be numbers")
        sys.exit(1)
    except rospy.ROSInterruptException:
        pass
