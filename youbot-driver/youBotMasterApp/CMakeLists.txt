#ein beispiel, wie ich den youBotDriver benutzen kann,
#wenn sich die SharedLibrary und die Header in dem Systemverzeichnis befinden.
#KommentarEnde

project(youBotMasterApplication)

cmake_minimum_required(VERSION 2.6)

SET(locationSlaveFiles ${CMAKE_SOURCE_DIR}/EtherCatSlaves)

include_directories(${locationSlaveFiles})
include_directories(${locationSlaveFiles}/messages)

SET(srcYouBotMasterSlaves
		${locationSlaveFiles}/youBotSlaveComponent.cpp
		${locationSlaveFiles}/youBotArm.cpp
	)

add_executable(../youBotApp youBotMasterApp.cpp ${srcYouBotMasterSlaves} )

target_link_libraries(../youBotApp youBotDriver memoryMapAndSemaphore)

install(DIRECTORY  ${locationSlaveFiles}/messages/
		DESTINATION include
		FILES_MATCHING PATTERN "*.h"	#only header files
		PATTERN .svn EXCLUDE 			#exclude .svn directory
		)
