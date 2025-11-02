#!/usr/bin/env python3

import rospy
import tf
import math
from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped

class TurtleCarrotBroadcaster:
    def __init__(self):
        # Инициализируем узел
        rospy.init_node('turtle_carrot_broadcaster')
        
        # Получаем имя черепашки из параметра
        self.turtlename = rospy.get_param('~turtle_tf_name', 'turtle1')
        
        # Параметры вращения морковки
        self.rotation_radius = 1.0  # радиус вращения в метрах
        self.rotation_speed = 1.0   # скорость вращения (рад/сек)
        self.start_time = rospy.Time.now().to_sec()
        
        # Подписываемся на топик с позой черепашки
        rospy.Subscriber('input_pose',
                        Pose,
                        self.handle_turtle_pose,
                        self.turtlename)
        
        rospy.loginfo("Turtle and Carrot TF Broadcaster started for %s", self.turtlename)

    def handle_turtle_pose(self, msg, turtlename):
        # Создаем объекты для трансформации
        br = tf.TransformBroadcaster()
        
        # Публикуем трансформацию для самой черепашки
        translation_turtle = (msg.x, msg.y, 0.0)
        rotation_turtle = tf.transformations.quaternion_from_euler(0, 0, msg.theta)
        
        br.sendTransform(translation_turtle,
                        rotation_turtle,
                        rospy.Time.now(),
                        turtlename,
                        "world")
        
        # Вычисляем позицию вращающейся морковки
        current_time = rospy.Time.now().to_sec()
        elapsed_time = current_time - self.start_time
        
        # Угол вращения морковки вокруг черепашки
        carrot_angle = self.rotation_speed * elapsed_time
        
        # Позиция морковки относительно черепашки (круговая орбита)
        carrot_x = self.rotation_radius * math.cos(carrot_angle)
        carrot_y = self.rotation_radius * math.sin(carrot_angle)
        carrot_z = 0.0
        
        # Ориентация морковки (можно сделать так, чтобы она всегда смотрела наружу)
        carrot_yaw = carrot_angle + math.pi  # смотрит наружу от черепашки
        
        translation_carrot = (carrot_x, carrot_y, carrot_z)
        rotation_carrot = tf.transformations.quaternion_from_euler(0, 0, carrot_yaw)
        
        # Публикуем трансформацию для морковки
        br.sendTransform(translation_carrot,
                        rotation_carrot,
                        rospy.Time.now(),
                        "carrot",
                        turtlename)
        
        rospy.loginfo_throttle(2, "Carrot position: (%.2f, %.2f) around %s", 
                              carrot_x, carrot_y, turtlename)

if __name__ == '__main__':
    try:
        broadcaster = TurtleCarrotBroadcaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
