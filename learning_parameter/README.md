# Parameter的使用与编程方法（Python）
在 ROS 中，存在一个参数服务器（Parameter Server），这是一个**全局词典**，即一个全局变量的存储空间，用来保存各个节点的配置数据。所有节点都可以对其参数进行全局访问。
## rosparam 命令行使用
+ 在 ROS 中，参数文件以`YAML`文件格式保存，`rosparam`命令的用法：
1. 列出当前参数

    `rosparam list`
    
2. 显示某个参数值

    `rosparam get <param_key>`
    
3. 设置某个参数值

    `rosparam set <param_key> <param_value>`
    
4. 保存参数到文件

    `rosparam dump <file_name>`
    
5. 从文件读取参数

    `rosparam load <file_name>`
    
6. 删除参数

    `rosparam delete <file_name>`   
    
 ## Python 编程方法
 
 1. 读取参数
 
  ```
  rospy.get_param()
  ``` 
  
 2. 设置参数

  ```
  rospy.set_param()
  ```  
 
