# ROS 笔记
## Copyright
所有[代码](https://github.com/guyuehome/ros_21_tutorials)来自[【古月居】古月·ROS入门21讲](https://www.bilibili.com/video/BV1zt411G7Vn?p=1&vd_source=6c2184781d2aa58fa5e90d1cc75a1dd1);

讲解来自[【古月居】古月·ROS入门21讲](https://www.bilibili.com/video/BV1zt411G7Vn?p=1&vd_source=6c2184781d2aa58fa5e90d1cc75a1dd1)与[中科院软件所-机器人操作系统入门（ROS入门教程）](https://www.bilibili.com/video/BV1mJ411R7Ni?p=1&vd_source=6c2184781d2aa58fa5e90d1cc75a1dd1)

## 目录
[Topic](https://github.com/LinkinEminem/ROS_Notes/tree/master/learning_topic)

[Service](https://github.com/LinkinEminem/ROS_Notes/tree/master/learning_service)

## Tips
#### 1. 配置代码编译规则（Python）
在 CMakeLists.txt 如图的位置上，加入路径 / 文件名：

   ![image](https://user-images.githubusercontent.com/45569291/177819272-35c0b4e9-7b0d-415f-aeef-ed672bd5a12d.png)


#### 2. 建议在`./bashrc`路径下面，加上根功能包路径下的`source`命令，如：

   ```
   echo "source ros_learn/devel/setup.bash" >> ~/.bashrc
   ```
   如果不添加，那么每打开新的终端都需要重新`source`，否则找不到功能包目录。

#### 3. 开始新的工作空间
   
   ```
   mkdir -p ~/ros_learn/src  # 创建目录
   cd ~/ros_learn/src
   catkin_init_workspace  # 初始化
   
   cd ~/ros_learn
   catkin_make  # 编译src里全部的源码
   ```
   此时新生成了`build`和`devel`两个文件夹。

#### 4. 创建功能包

   功能包是 ROS 的最小单元，创建功能包：
   ```
   catkin_create_pkg <package_name> [depend1] [depend2] [depend3] ...
   ```
#### 5. 编译功能包

   回到工作空间根目录，再编译即可：
   ```
   cd ~/ros_learn
   catkin_make
   ```
#### 6. 两个重要文件`package.xml`和`CMakeLists.txt`
   + `package.xml`涉及功能包名，描述，作者联系方式，依赖信息等；
   + `CMakeLists.txt`涉及编译规则。
