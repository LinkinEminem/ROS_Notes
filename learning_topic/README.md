# Topic相关的编程实现（Python）
Topic是ROS中的一种**异步**通信方式，通过“发布-订阅（Publisher-Subscriber）”方式通信，一般用于连续、高频的数据发布，如激光雷达、里程计发布等。
1. ROS 中的每一个进程都成为节点（Node）；
2. 每一个 node 在启动时，都会向 ROS Master 注册；
3. Master 负责管理每一个节点之间的通信；
4. 在 topic 通信方式中，node 之间通过 publish-subscribe 机制通信

   即：
   **Publisher
   $\Rightarrow$
   Topic
   $\Rightarrow$
   Subscriber**

## 发布者（Publisher）
1. 初始化 ROS 节点
    ```
    def velocity_publisher():
      rospy.init_node('velocity_publisher', anonymous=True)
    ```
2. 创建 Publisher
    ```
      turtle_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)  # 参数：发布的Topic，消息的类型，队列长度
    ```
3. 设置循环频率
    ```
      rate = rospy.Rate(10)
    ```
4. 循环发布消息，初始化 Twist 类型的消息
    ```
      while not rospy.is_shutdown():
        vel_msg = Twist()
        vel_msg.linear.x = 0.5
        vel_msg.angular.z = 0.2
    ```
5. 发布消息，打印发布的内容
    ```
        turtle_vel_pub.publish(vel_msg)
        rospy.loginfo("Publish turtle velocity command[%0.2f] m/s, %0.2f rad/s",
                      vel_msg.linear.x, vel_msg.angular.z)
    ```
6. 按照循环频率延时
    ```
        rate.sleep()
    ```
## 订阅者（Subscriber）
1. 定义回调函数，当订阅者订阅的 topic 发布了 msg 时，立即调用回调函数：
    ```
    def poseCallback(msg):
      rospy.loginfo("Turtle pose: x:%0.6f, y:%0.6f", msg.x, msg.y)
    ```
3. 初始化 ROS 节点
    ```
    def pose_subscriber():
      rospy.init_node("pose_subscriber", anonymous=True)
    ```
2. 创建 Subscriber
    ```
      rospy.Subscriber("/turtle1/pose", Pose, poseCallback)
    ```
3. 循环等待回调函数
    ```
      rospy.spin()
    ```
