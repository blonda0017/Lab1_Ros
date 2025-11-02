#!/usr/bin/env python3
import rospy 
from std_msgs.msg import Float64MultiArray, Float64 #Импорт типов сообщений - массив чисел и одиночное число

class PolynomialNode: #Объявление класса узла - использование ООП подхода для организации кода
    def __init__(self): #Конструктор класса - выполняется при создании объекта класса
        rospy.init_node('polynomial_node') #регистрирует узел
        
        # Подписываемся на входные числа
        self.input_sub = rospy.Subscriber('/input_numbers', Float64MultiArray, self.input_callback) #Создание подписчика: Слушает топик, Ожидает сообщения,При получении вызывает функцию input_callback
        
        # Публикуем обработанные числа
        self.output_pub = rospy.Publisher('/processed_numbers', Float64MultiArray, queue_size=10) #Создание публикатора:Публикует в топик, Тип сообщения, queue_size=10 - размер буфера сообщений
        
        rospy.loginfo("Polynomial node started")#вывод информационного сообщения о запуске узла
        
    def input_callback(self, msg): #вызывается при получении нового сообщения в топике /input_numbers
        """Обрабатывает входные числа: возводит в степень по позиции"""
        numbers = msg.data #получение массива чисел из сообщения
        
        if len(numbers) != 3: #Проверка количества чисел 
            rospy.logwarn("Expected 3 numbers, got %d", len(numbers)) #Логирование предупреждения - если получено не 3 числа
            return #прекращает обработку при неверном количестве чисел
            
        # Возводим в степень: первое число^1, второе^2, третье^3
        processed_numbers = [
            numbers[0] ** 1,  # x^1
            numbers[1] ** 2,  # x^2  
            numbers[2] ** 3   # x^3
        ]
        
        # Публикуем обработанные числа
        output_msg = Float64MultiArray() #Создание объекта сообщения - для отправки результата
        output_msg.data = processed_numbers #запись обработанных чисел в сообщение
        
        self.output_pub.publish(output_msg) #отправка результата в топик /processed_numbers
        rospy.loginfo("Processed numbers: %s -> %s", numbers, processed_numbers) #вывод входных и выходных данных для отладки

if __name__ == '__main__': #Проверка условия запуска - код выполняется только при прямом запуске файла
    try: #для перехвата исключений
        PolynomialNode() # Создание экземпляра класса - инициализация узла
        rospy.spin() #поддерживает узел активным, обрабатывает входящие сообщения
    except rospy.ROSInterruptException: #Перехват исключения
        pass
