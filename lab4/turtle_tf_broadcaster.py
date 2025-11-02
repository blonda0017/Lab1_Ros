#!/usr/bin/env python3

import rospy
import tf
from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped

def handle_turtle_pose(msg, turtlename):
    # Создаем объект для трансформации
    br = tf.TransformBroadcaster()
    
    # Преобразуем Pose в TransformStamped
    t = TransformStamped()
    
    # Заполняем заголовок
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = turtlename
    
    # Позиция (x, y, z)
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0
    
    # Ориентация (из theta в кватернион)
    q = tf.transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    
    # Публикуем трансформацию
    br.sendTransformMessage(t)

if __name__ == '__main__':
    # Инициализируем узел
    rospy.init_node('turtle_tf_broadcaster')
    
    # Получаем имя черепашки из параметра
    turtlename = rospy.get_param('~turtle_tf_name', 'turtle1')
    
    # Подписываемся на топик с позой черепашки
    rospy.Subscriber('input_pose',
                     Pose,
                     handle_turtle_pose,
                     turtlename)
    
    rospy.loginfo("TF Broadcaster started for %s", turtlename)
    rospy.spin()
