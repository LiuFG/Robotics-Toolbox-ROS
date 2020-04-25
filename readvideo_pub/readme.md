# how to run

copy the two folders into your ros_workspace/src, then catkin_make

finally, run:

```
rosrun video_transport_tutorial video_publisher
rosrun rqt_image_view rqt_image_view
```



# 明晰基础概念

好处：C++,python
ROS系统由多个各自独立的节点（组件）组成，并且各个节点之间可以通过发布/订阅(publish/subscribe)消息模型进行通信。
# 创建node
## 创建工作空间，创建功能包
1. 工作空间下（一个文件夹），创建工作空间   
```
mkdir src
catkin_init_workspace
```
2. 创建功能包，生成CMakeList.txt和package.xml  
`catkin_create_pkg ros_test std_msgs rospy roscpp`
3. CMakeList.txt中添加依赖
```
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)
```
4. CMakeList.txt中链接执行文件
```
add_executable(readvideo_publisher
  readvideo_publisher.cpp
)
target_link_libraries(readvideo_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
```
## 发布多个topic与订阅
readvideo_publisher.cpp
video_subscriber.cpp
## 订阅多个topic，并发布多个（用多个类实现）
other_case.cpp(不能跑)
## 更新代码
重新到工作空间catkin_make
# 运行node
## source
source devel/setup.bash
## 运行一个节点
rosrun 
## 查看节点与话题情况
* rosnode
```
list
info
```
* rostopic
```
list
info
rostopic hz
rostopic echo
```
* 图形显示
看节点/topic视图：rosrun rqt_graph rqt_graph或rqt
rosrun rqt_plot qt_plot

## 多节点一句运行
用[roslaunch](https://blog.csdn.net/weixin_41995979/article/details/81784987)
