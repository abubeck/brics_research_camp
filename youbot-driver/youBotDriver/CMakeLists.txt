project(youBotDriver)
cmake_minimum_required(VERSION 2.8)

#SET(locationSM ./src/simpleMaster)
#SET(locationYM ./src/youBotMaster)

#include_directories(${CMAKE_SOURCE_DIR}/src/simpleMaster)
#include_directories(${CMAKE_SOURCE_DIR}/src/youBotMaster)

SET(locationSM ${CMAKE_SOURCE_DIR}/src/soemEtherCatCore) 
SET(locationYM ${CMAKE_SOURCE_DIR}/src/soemEtherCatMaster)

include_directories( ${locationSM} ) #This is necessary for the source-files in the youBotMaster directory, which include headers from here.
include_directories( ${locationYM} ) #This might not be neccessary, but I still include it

SET(srcSimpleMaster 
		${locationSM}/nicdrv.c  
		${locationSM}/ethercatsoe.c 
		${locationSM}/ethercatprint.c 
		${locationSM}/ethercatmain.c
		${locationSM}/ethercatfoe.c
		${locationSM}/ethercatdc.c
		${locationSM}/ethercatconfig.c
		${locationSM}/ethercatcoe.c
		${locationSM}/ethercatbase.c
	)
	
SET(srcYouBotMaster 
		${locationYM}/soem_master.cpp
		${locationYM}/soem_driver_factory.cpp
	)

SET(allSources ${srcSimpleMaster} ${srcYouBotMaster})

# create Shared Library
add_library(youBotDriver SHARED ${allSources})

# copy the library to ${CMAKE_INSTALL_PREFIX}/../lib
# on linux: /usr/lib
install(TARGETS youBotDriver DESTINATION ../lib)


# copy the Headers to ${CMAKE_INSTALL_PREFIX}/include --> /usr/local/include
install(DIRECTORY ${locationSM}/ ${locationYM}/ 
		DESTINATION include 
		FILES_MATCHING PATTERN "*.h"		#only header files
		PATTERN .svn EXCLUDE 				#exclude .svn directory
		)
