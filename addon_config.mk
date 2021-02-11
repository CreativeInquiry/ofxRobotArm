# All variables and this file are optional, if they are not present the PG and the
# makefiles will try to parse the correct values from the file system.
#
# Variables that specify exclusions can use % as a wildcard to specify that anything in
# that position will match. A partial path can also be specified to, for example, exclude
# a whole folder from the parsed paths from the file system
#
# Variables can be specified using = or +=
# = will clear the contents of that variable both specified from the file or the ones parsed
# from the file system
# += will add the values to the previous ones in the file or the ones parsed from the file 
# system
# 
# The PG can be used to detect errors in this file, just create a new project with this addon 
# and the PG will write to the console the kind of error and in which line it is

meta:
	ADDON_NAME = ofxABBDriver
	ADDON_DESCRIPTION = ofxABBDriver for ofxRobotArm!
	ADDON_AUTHOR = @danzeeeman
	ADDON_TAGS = "addon" "template"
	ADDON_URL = http://github.com/CreativeInquiry/ofxABBDriver

common:
	# dependencies with other addons, a list of them separated by spaces 
	# or use += in several lines
	ADDON_DEPENDENCIES = ofxProtobuf ofxIKArm
	
	# include search paths, this will be usually parsed from the file system
	# but if the addon or addon libraries need special search paths they can be
	# specified here separated by spaces or one per line using +=
	# ADDON_INCLUDES += libs/abb_libegm/include/abb_libegm
	# ADDON_INCLUDES += libs/abb_librws/include/abb_librws
	# ADDON_INCLUDES += libs/abb_librws/protp/
	# ADDON_INCLUDES += libs/abb_libegm/protp/
	# ADDON_INCLUDES += libs/urDriver/
	# ADDON_INCLUDES += src/constrollers
	# ADDON_INCLUDES += src/drivers
	# ADDON_INCLUDES += src/kinematics
	# ADDON_INCLUDES += src/path
	# ADDON_INCLUDES += src/utils
	# ADDON_INCLUDES += src/world
	
	# any special flag that should be passed to the compiler when using this
	# addon
	# ADDON_CFLAGS =
	
	# any special flag that should be passed to the compiler for c++ files when 
	# using this addon
	# ADDON_CPPFLAGS =
	
	# any special flag that should be passed to the linker when using this
	# addon, also used for system libraries with -lname
	# ADDON_LDFLAGS =
	
	# source files, these will be usually parsed from the file system looking
	# in the src folders in libs and the root of the addon. if your addon needs
	# to include files in different places or a different set of files per platform
	# they can be specified here

	# ADDON_SOURCES = libs/abb_libegm/src/egm_base_interface.cpp
	# ADDON_SOURCES += libs/abb_libegm/src/egm_common_auxiliary.cpp
	# ADDON_SOURCES += libs/abb_libegm/src/egm_common.cpp
	# ADDON_SOURCES += libs/abb_libegm/src/egm_controller_interface.cpp
	# ADDON_SOURCES += libs/abb_libegm/src/egm_interpolator.cpp
	# ADDON_SOURCES += libs/abb_libegm/src/egm_logger.cpp
	# ADDON_SOURCES += libs/abb_libegm/src/egm_trajectory_interface.cpp
	# ADDON_SOURCES += libs/abb_libegm/src/egm_udp_server.cpp
	# ADDON_SOURCES += libs/abb_librws/src/rws_client.cpp
	# ADDON_SOURCES += libs/abb_librws/src/rws_common.cpp
	# ADDON_SOURCES += libs/abb_librws/src/rws_interface.cpp
	# ADDON_SOURCES += libs/abb_librws/src/rws_poco_client.cpp
	# ADDON_SOURCES += libs/abb_librws/src/rws_rapid.cpp
	# ADDON_SOURCES += libs/abb_librws/src/rws_state_machine_interface.cpp
	# ADDON_SOURCES += libs/abb_libegm/proto/egm_wrapper_trajectory.pb.cc
	# ADDON_SOURCES += libs/abb_libegm/proto/egm_wrapper.pb.cc
	# ADDON_SOURCES += libs/abb_libegm/proto/egm.pb.cc
	# ADDON_SOURCES += src/controllers/RobotController.cpp
	# ADDON_SOURCES += src/controllers/PathController.cpp
	# ADDON_SOURCES += src/controllers/Move.cpp
	# ADDON_SOURCES += src/drivers/ABBDriver.cpp
	# ADDON_SOURCES += src/drivers/URDriver.cpp
	# ADDON_SOURCES += src/kinematics/InverseKinematics.cpp
	# ADDON_SOURCES += src/kinematics/kin.cpp
	# ADDON_SOURCES += src/kinematics/RobotKinematicModel.cpp
	# ADDON_SOURCES += src/path/GMLPath.cpp
	# ADDON_SOURCES += src/path/Path.cpp
	# ADDON_SOURCES += src/path/Path3D.cpp
	# ADDON_SOURCES += src/utils/CylinderRestrictor.cpp
	# ADDON_SOURCES += src/utils/JointRestrictor.cpp
	# ADDON_SOURCES += src/utils/PathPlayer.cpp
	# ADDON_SOURCES += src/utils/PathRecorder.cpp
	# ADDON_SOURCES += src/utils/RobotAngleOffsets.cpp
	# ADDON_SOURCES += src/utils/RobotArmCollision.cpp
	# ADDON_SOURCES += src/utils/RobotArmSafety.cpp
	# ADDON_SOURCES += src/utils/RobotConstants.cpp
	# ADDON_SOURCES += src/utils/RobotStateMachine.cpp
	# ADDON_SOURCES += src/utils/ToolHead.cpp
	# ADDON_SOURCES += src/world/WorkSurface3D.cpp
	# ADDON_SOURCES += libs/urDriver/do_output.cpp
	# ADDON_SOURCES += libs/urDriver/robot_state_RT.cpp
	# ADDON_SOURCES += libs/urDriver/robot_state.cpp
	# ADDON_SOURCES += libs/urDriver/ur_communication.cpp
	# ADDON_SOURCES += libs/urDriver/ur_driver.cpp
	# ADDON_SOURCES += libs/urDriver/ur_realtime_communication.cpp
    
			
	
	# source files that will be included as C files explicitly
	# ADDON_C_SOURCES = 
	
	# source files that will be included as header files explicitly
	# ADDON_HEADER_SOURCES = src/constrollers/RobotController.h
	# ADDON_HEADER_SOURCES += src/constrollers/PathController.h
	# ADDON_HEADER_SOURCES += src/constrollers/Move.h
	# ADDON_HEADER_SOURCES += src/drivers/ABBDriver.h
	# ADDON_HEADER_SOURCES += src/drivers/URDriver.h
	# ADDON_HEADER_SOURCES += src/kinematics/InverseKinematics.h
	# ADDON_HEADER_SOURCES += src/kinematics/kin.h
	# ADDON_HEADER_SOURCES += src/kinematics/RobotKinematicModel.h
	# ADDON_HEADER_SOURCES += src/path/GMLPath.h
	# ADDON_HEADER_SOURCES += src/path/Path.h
	# ADDON_HEADER_SOURCES += src/path/Path3D.h
	# ADDON_HEADER_SOURCES += src/utils/CylinderRestrictor.h
	# ADDON_HEADER_SOURCES += src/utils/JointRestrictor.h
	# ADDON_HEADER_SOURCES += src/utils/PathPlayer.h
	# ADDON_HEADER_SOURCES += src/utils/PathRecorder.h
	# ADDON_HEADER_SOURCES += src/utils/RobotAngleOffsets.h
	# ADDON_HEADER_SOURCES += src/utils/RobotArmCollision.h
	# ADDON_HEADER_SOURCES += src/utils/RobotArmSafety.h
	# ADDON_HEADER_SOURCES += src/utils/RobotConstants.h
	# ADDON_HEADER_SOURCES += src/utils/RobotStateMachine.h
	# ADDON_HEADER_SOURCES += src/utils/ToolHead.h
	# ADDON_HEADER_SOURCES += src/world/WorkSurface3D.h
	# ADDON_HEADER_SOURCES += libs/urDriver/do_output.h
	# ADDON_HEADER_SOURCES += libs/urDriver/robot_state_RT.h
	# ADDON_HEADER_SOURCES += libs/urDriver/robot_state.h
	# ADDON_HEADER_SOURCES += libs/urDriver/ur_communication.h
	# ADDON_HEADER_SOURCES += libs/urDriver/ur_driver.h
	# ADDON_HEADER_SOURCES += libs/urDriver/ur_realtime_communication.h
	
	# source files that will be included as c++ files explicitly
	# ADDON_CPP_SOURCES = 
	
	# source files that will be included as objective c files explicitly
	# ADDON_OBJC_SOURCES = 
	
	# derines that will be passed to the compiler when including this addon
	# ADDON_DEFINES
	
	# some addons need resources to be copied to the bin/data folder of the project
	# specify here any files that need to be copied, you can use wildcards like * and ?
	# ADDON_DATA = 
	
	# when parsing the file system looking for libraries exclude this for all or
	# a specific platform
	# ADDON_LIBS_EXCLUDE =
	
	# binary libraries, these will be usually parsed from the file system but some 
	# libraries need to passed to the linker in a specific order/
	# 
	# For example in the ofxOpenCV addon we do something like this:
	#
	# ADDON_LIBS =
	# ADDON_LIBS += libs/opencv/lib/linuxarmv6l/libopencv_legacy.a
	# ADDON_LIBS += libs/opencv/lib/linuxarmv6l/libopencv_calib3d.a
	# ...
	
	
linux64:
	# linux only, any library that should be included in the project using
	# pkg-config
	# ADDON_PKG_CONFIG_LIBRARIES =
	
	ADDON_LDFLAGS = -L /usr/local/lib/librelaxed_ik_lib 
	ADDON_LDFLAGS += -labb_libegm
	ADDON_LDFLAGS += -labb_librws
	ADDON_LDFLAGS += -lboost_thread 
	ADDON_LDFLAGS += -lboost_regex 
vs:
	# After compiling copy the following dynamic libraries to the executable directory
	# only windows visual studio
	# ADDON_DLLS_TO_COPY = 
	
linuxarmv6l:
linuxarmv7l:
android/armeabi:	
android/armeabi-v7a:	
osx:
	# osx/iOS only, any framework that should be included in the project
	# ADDON_FRAMEWORKS =
	ADDON_LIBS += ../../../libs/boost/lib/osx/libboost_thread.a 
	ADDON_LIBS += ../../../libs/boost/lib/osx/libboost_regex.a 

ios:
tvos:
