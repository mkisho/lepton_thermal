cmake_minimum_required(VERSION 2.8.3)
project(lepton_thermal)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  message_generation
)

# generate_messages(
#   DEPENDENCIES
# )

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs message_runtime  #opencv3
)



include_directories(
 include
  src  ${catkin_INCLUDE_DIRS}
  src/leptonSDKEmb32PUB
/opt/ros/kinetic/include/opencv-3.3.1-dev/
)

add_executable(${PROJECT_NAME}_node src/main.cpp 
		src/gui.cpp 
		src/lepton-threads.cpp 
		src/Lepton_I2C.cpp 
		src/Palettes.cpp 
		src/SPI.cpp 
		src/leptonSDKEmb32PUB/LEPTON_SDK.c
		src/leptonSDKEmb32PUB/LEPTON_SYS.c
		src/leptonSDKEmb32PUB/LEPTON_VID.c
		src/leptonSDKEmb32PUB/LEPTON_AGC.c
		src/leptonSDKEmb32PUB/LEPTON_I2C_Service.c
		src/leptonSDKEmb32PUB/LEPTON_I2C_Protocol.c
		src/leptonSDKEmb32PUB/raspi_I2C.c
		src/leptonSDKEmb32PUB/crc16fast.c
		)













 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )
