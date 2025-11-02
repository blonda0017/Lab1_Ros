#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, Float64 #Импорт типов сообщений - массив чисел и одиночное число

class SummingNode: #Класс узла суммирования - ООП подход
    def __init__(self): #Конструктор класса - инициализация узла
        rospy.init_node('summing_node') #Инициализация ROS узла - регистрация под именем 'summing_node'
        
        # Подписываемся на обработанные числа
        self.processed_sub = rospy.Subscriber('/processed_numbers', Float64MultiArray, self.processed_callback) #Подписчик - слушает топик с обработанными числами
        
        # Публикуем результат суммы
        self.result_pub = rospy.Publisher('/calculation_result', Float64, queue_size=10) #Публикатор - отправляет результат вычислений
        
        rospy.loginfo("Summing node started")#Логирование - сообщение о запуске узла
        
    def processed_callback(self, msg): #обработка входящих сообщений
        """Суммирует все числа в массиве"""
        numbers = msg.data #Извлечение данных - получение массива чисел из сообщения
        total_sum = sum(numbers)#Вычисление суммы - сложение всех чисел в массиве
        
        # Публикуем результат
        result_msg = Float64() #Создание сообщения - для результата
        result_msg.data = total_sum #Заполнение данными - запись суммы в сообщение
        
        self.result_pub.publish(result_msg) #Публикация результата - отправка суммы в топик
        rospy.loginfo("Sum calculated: %s = %.2f", numbers, total_sum) #Логирование - вывод вычислений для отладки

if __name__ == '__main__': #Точка входа - выполняется при прямом запуске
    try:
        SummingNode() #Создание узла - инициализация и запуск
        rospy.spin() #Главный цикл - поддержание работы узла
    except rospy.ROSInterruptException:
        pass #Обработка прерывания - корректное завершение
