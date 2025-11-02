#!/usr/bin/env python3
import rospy
import sys #Импорт системной библиотеки
from std_msgs.msg import Float64MultiArray, Float64 #Импорт типов сообщений - массив чисел и одиночное число

class RequestNode: # Класс узла запроса
    def __init__(self, numbers): #Конструктор - принимает список чисел для обработки
        rospy.init_node('request_node', anonymous=True) #Инициализация узла - anonymous=True позволяет запускать multiple копии
        self.numbers = numbers #Сохранение входных чисел - в переменную класса
        self.result = None #Инициализация результата - изначально пустой
        self.result_received = False #Флаг получения результата - индикатор готовности данных
        
        # Подписываемся на результат
        self.result_sub = rospy.Subscriber('/calculation_result', Float64, self.result_callback) #Подписчик на результат - слушает топик с конечным результатом
        
        # Публикуем входные числа
        self.input_pub = rospy.Publisher('/input_numbers', Float64MultiArray, queue_size=10) #Публикатор входных данных - отправляет числа на обработку
        
    def result_callback(self, msg): #обработка полученного результата
        """Получает результат вычислений"""
        self.result = msg.data #Сохранение результата - извлекает число из сообщения
        self.result_received = True #Установка флага - отмечает что результат получен
        rospy.loginfo("Result received: %.2f", self.result) #
        
    def send_request(self): #Основной метод - отправка запроса и ожидание ответа
        """Отправляет запрос и ждет ответ"""
        # Ждем подключения к топику результатов
        rospy.sleep(1) #Пауза - ожидание инициализации ROS соединений
        
        # Отправляем входные числа
        input_msg = Float64MultiArray() #Создание сообщения - для входных данных
        input_msg.data = self.numbers #Заполнение данными - упаковка чисел в сообщение
        
        self.input_pub.publish(input_msg) #Публикация - отправка чисел на обработку
        rospy.loginfo("Sent numbers: %s", self.numbers) #Логирование - подтверждение отправки
        
        # Ждем результат (таймаут 5 секунд)
        timeout = rospy.Time.now() + rospy.Duration(5) #Установка таймаута - 5 секунд на ожидание ответа
        while not self.result_received and rospy.Time.now() < timeout: #Цикл ожидания - пока не получен результат или не истек таймаут
            rospy.sleep(0.1) #Короткая пауза - чтобы не нагружать процессор
            
        if self.result_received: #Проверка получения результата
            print(f"\n=== RESULT ===") 
            print(f"Input numbers: {self.numbers}")
            print(f"Calculation: {self.numbers[0]}^1 + {self.numbers[1]}^2 + {self.numbers[2]}^3")
            print(f"Result: {self.result}")
            print("==============\n")
            return self.result #Возврат результата
        else:
            rospy.logerr("Timeout waiting for result") #Ошибка таймаута - если результат не получен вовремя
            return None

if __name__ == '__main__': #Точка входа - выполняется при прямом запуске
    if len(sys.argv) != 4: #Проверка аргументов - должно быть 3 числа + имя скрипта
        print("Usage: rosrun super_blonda_study_pkg request_node.py <num1> <num2> <num3>")
        sys.exit(1) #Выход с ошибкой - код 1 означает ошибку
        
    try:
        numbers = [float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])] #Преобразование аргументов - из строк в числа с плавающей точкой
        
        node = RequestNode(numbers) #Создание узла - инициализация с числами
        result = node.send_request() #Отправка запроса - запуск основного процесса
        
        if result is not None:
            sys.exit(0) #Успешный выход - код 0 означает успех
        else:
            sys.exit(1) #Выход с ошибкой - если результат не получен
            
    except ValueError:
        print("Error: All arguments must be numbers")#Обработка ошибки - если аргументы не числа
        sys.exit(1) #Выход с ошибкой
    except rospy.ROSInterruptException:
        pass #Обработка прерывания ROS - корректное завершение
