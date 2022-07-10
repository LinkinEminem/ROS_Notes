# Parameter的使用与编程方法（Python）
在 ROS 中，存在一个参数服务器（Parameter Server），这是一个**全局词典**，即一个全局变量的存储空间，用来保存各个节点的配置数据。所有节点都可以对其参数进行全局访问。
## Parameter命令行使用
1. 在 ROS 中，参数文件以
