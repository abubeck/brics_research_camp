project(youBotApi)
cmake_minimum_required(VERSION 2.8)

SET(source  ${CMAKE_SOURCE_DIR}/src/) 
SET(include ${CMAKE_SOURCE_DIR}/include/)

include_directories( ${include} )
	
SET(srcYouBotApi 
		${source}/youBotApi.cpp
	)

# create Shared Library --> not needed, because the methods are implemented in the header
#add_library(youBotApi SHARED ${srcYouBotApi})

# copy the library to ${CMAKE_INSTALL_PREFIX}/../lib
# on linux: /usr/lib
#install(TARGETS youBotApi DESTINATION ../lib)


# copy the Headers to ${CMAKE_INSTALL_PREFIX}/include --> /usr/local/include
install(DIRECTORY ${include}/ 
		DESTINATION include 
		FILES_MATCHING PATTERN "*.h"	#only header files
		PATTERN .svn EXCLUDE 			#exclude .svn directory
		)
