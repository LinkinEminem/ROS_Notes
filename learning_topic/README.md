# Topic 相关的编程实现（Python）
Topic 是 ROS 中的一种**异步**通信方式，通过“发布-订阅（Publisher-Subscriber）”方式通信，一般用于连续、高频的数据发布，如激光雷达、里程计发布等。
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
+ 初始化 ROS 节点
    ```
    def velocity_publisher():
      rospy.init_node('velocity_publisher', anonymous=True)
    ```
+ 创建 Publisher
    ```
      turtle_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)  # 参数：发布的Topic，消息的类型，队列长度
    ```
+ 设置循环频率
    ```
      rate = rospy.Rate(10)
    ```
+ 循环发布消息，初始化 Twist 类型的消息
    ```
      while not rospy.is_shutdown():
        vel_msg = Twist()
        vel_msg.linear.x = 0.5
        vel_msg.angular.z = 0.2
    ```
+ 发布消息，打印发布的内容
    ```
        turtle_vel_pub.publish(vel_msg)
        rospy.loginfo("Publish turtle velocity command[%0.2f] m/s, %0.2f rad/s",
                      vel_msg.linear.x, vel_msg.angular.z)
    ```
+ 按照循环频率延时
    ```
        rate.sleep()
    ```
## 订阅者（Subscriber）
+ 定义回调函数，当订阅者订阅的 topic 发布了 msg 时，立即调用回调函数：
    ```
    def poseCallback(msg):
      rospy.loginfo("Turtle pose: x:%0.6f, y:%0.6f", msg.x, msg.y)
    ```
+ 初始化 ROS 节点
    ```
    def pose_subscriber():
      rospy.init_node("pose_subscriber", anonymous=True)
    ```
+ 创建 Subscriber
    ```
      rospy.Subscriber("/turtle1/pose", Pose, poseCallback)
    ```
+ 循环等待回调函数
    ```
      rospy.spin()
    ```
## Topic 消息（Message）的格式定义与使用
Message 是 topic 内容中的数据类型，格式标准定义在`*.msg`中。
#### 1. 定义 msg 文件
+ 首先，新建`msg`文件夹，并使用`touch`命令新建扩展名为`.msg`的文件
   ```
   mkdir msg
   touch Person.msg
   ```
+ 在文件中写入自定义的消息类型：
   ```
   string name
   uint8 gender
   uint8 age
   
   uint8 unknown = 0
   uint8 male = 1
   uint8 female = 2
   ```
#### 2. 在 package.xml 中添加功能包依赖
打开 package.xml 文件，在指定位置加入：
```
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```

![image](https://user-images.githubusercontent.com/45569291/177653036-a666b9fc-b1ce-4736-bb45-094a61ee4717.png)

其中
   + `build_depend`为编译依赖，此处是一个会动态产生message的功能包
   + `exer_depend`为执行依赖，此处是一个动态runtime运行的功能包

#### 3. 在 CMakeLists.txt 中添加编译选项
+ 在 CMakeList.txt 里的 find_package 中加入功能包编译依赖

   ![image](https://user-images.githubusercontent.com/45569291/177653141-9ad6914a-02bc-4c59-8fa9-f0514f69358a.png)

+ 将定义的 Person.msg 作为消息接口，针对它做编译

   ![image](https://user-images.githubusercontent.com/45569291/177653173-a7c2adf3-6a1d-4e96-b0cf-b0227309bdf3.png)

+ 指明编译此消息接口需要的 ROS 包

   ![image](https://user-images.githubusercontent.com/45569291/177653200-194b5e64-4df0-4a1e-9454-bb6a767a3be9.png)

返回工作根目录，执行编译操作后，可以在`/devel/lib/python2.7/dist-packages/learning_topic/msg`找到相对应的Python包。
