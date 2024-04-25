import rospy
from tf.msg import tfMessage
from geometry_msgs.msg import TransformStamped

def callback(msg):
    # 터틀봇의 현재 위치와 방향을 출력합니다.
    for transform in msg.transforms:
        if transform.header.frame_id == "map" and transform.child_frame_id == "base_link":
            print("Turtlebot Position: ({}, {}, {}), Orientation: ({}, {}, {}, {})".format(
                transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z,
                transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w))

def main():
    rospy.init_node('turtlebot_position_monitor')

    # TF 메시지를 구독하여 "/map"에서 "/base_link" 프레임의 변환 정보를 가져옵니다.
    subscriber = rospy.Subscriber("/tf", tfMessage, callback)

    rospy.spin()

if __name__ == '__main__':
    main()
