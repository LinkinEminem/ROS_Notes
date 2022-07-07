# Serive 相关的编程实现（Python）
Servie 是 ROS 中的一种**同步**通信方式，通过“请求-回复（Request-Response）”方式通信，一般用于偶尔调用的功能或具体任务，如开关传感器、拍照、机器人逆解等。

   **Node A (Client)
   $\Leftrightarrow$
   Service
   $\Leftrightarrow$
   Node B (Server)**

## 客户端（Client）
+ 初始化 ROS 节点
+ 使用阻塞性函数，等待名为 spawn 的 service
    ```
    import rospy
    from service_demo.srv import *
    
    
    def client_srv():
      rospy.init_node('greetings_client')
      rospy.wait_for_service('greetings')  # 阻塞直到名为"greetings"的Service可用
    ```
+ 发现名为 spawn 的 Service 后，创建一个 Client ，向 Server 发送 Service 请求
+ 使用异常处理，请求 Service 调用，输入请求数据
    ```
      try:
        greetings_client = rospy.ServiceProxy('greetings', Greeting)  # 创建代理连接，传入Service名和服务类型
        response = greetings_client('Liu', 23)  # 传入Server需要的输入
        rospy.loginfo("Message From Server: %s" % response.feedback)
        return response.name
      except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s" % e)
    ```

## 服务端（Server）
+ 初始化 ROS 节点
+ 创建一个 Service
    ```
    def server_srv():
      rospy.init_node("greetings_client", anonymous=True)
      s = rospy.Service('greetings', Greeting, handle_function)  # service_name, service_type, handle function
      rospy.loginfo("Ready to handle the request.")
      rospy.spin()
    ```
+ 定义handle function
    ```
    def handle_function(req):
      rospy.loginfo("Request from ", req.name, "with age ", req.age)
      return GreetingResponse( "Hi %s, I'm server!" % req.name)
    ```
## Service 服务数据的格式定义与使用
Srv 是 service 服务中的数据格式，格式标准定义在`*.srv`中。
#### 1. 定义 srv 文件
+ 首先，新建`srv`文件夹，并使用`touch`命令新建扩展名为`.srv`的文件
   ```
   mkdir srv
   touch Person.srv
   ```
+ 在文件中写入自定义的消息类型：
   ```
  string name
  uint8 sex
  uint8 age
  
  uint8 unknown = 0
  uint8 male = 1
  uint8 female = 2
  ---
  string result
   ```
   以`---`为界，上面是`request`的内容，下面是`response`结果
#### 2. 在 package.xml 中添加功能包依赖
与增加`message`类似，打开 package.xml 文件，在指定位置加入：
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

+ 将定义的 Person.srv 作为消息接口，针对它做编译

   ![image](https://user-images.githubusercontent.com/45569291/177813810-8c89d4e1-b243-4e01-a43c-ab5103a1ef07.png)
   
+ 指明编译此消息接口需要的 ROS 包

   ![image](https://user-images.githubusercontent.com/45569291/177814151-8bb8e461-f1ec-4ed9-8052-f2e0e4401ddc.png)

返回工作根目录，执行编译操作后，可以在`/devel/lib/python2.7/dist-packages/learning_service/srv`找到相对应的Python包。
