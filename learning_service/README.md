# Serive 相关的编程实现（Python）
Servie 是 ROS 中的一种**同步**通信方式，通过“请求-回复（Request-Response）”方式通信，一般用于偶尔调用的功能或具体任务，如开关传感器、拍照、机器人逆解等。

   **Node A (Client)
   $\Leftrightarrow$
   Service
   $\Leftrightarrow$
   Node B (Server)**

## 客户端（Client）
+ 初始化 ROS 节点
    ```
    def turtle_spawn():
      rospy.init_node('turtle_spawn')
    ```
+ 使用阻塞性函数，等待名为Spawn的服务
    ```
      rospy.wait_for_service('/spawn')
    ```
+ 发现名为spawn的服务端后，创建一个客户端，向服务端发送服务请求
    ```
      try:
        add_turtle = rospy.ServiceProxy('/spawn', Spawn)
    ```
+ 请求服务调用，输入请求数据
    ```
        response = add_turtle(2.0, 2.0, 0.0, "turtle2")
        return response.name
    ```
+ 异常处理
    ```
      except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
    ```

## 服务端（Server）
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
