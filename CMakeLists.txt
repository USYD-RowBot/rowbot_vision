cmake_minimum_required(VERSION 2.8.3)
project(rowbot_vision)



add_compile_options(-std=c++11)



find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)







catkin_python_setup()


























 add_message_files(
   FILES
   Object.msg
   ObjectArray.msg
 )



add_service_files(
  FILES
 # Service2.srv
	BuoyDetect.srv
)









 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
	sensor_msgs
 )





























catkin_package(


CATKIN_DEPENDS message_runtime


)







include_directories(

 ${catkin_INCLUDE_DIRS}
)














































































