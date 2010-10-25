project(MemoryMappedFilesAndSemaphore)
cmake_minimum_required(VERSION 2.6)


SET(includeDir ${CMAKE_SOURCE_DIR}/include)
SET(sourceDir  ${CMAKE_SOURCE_DIR}/src) 

include_directories( ${includeDir} )

SET(sources
		${sourceDir}/MemoryMappedFiles.cpp  
		${sourceDir}/SemaphoreLock.cpp
	)


# create Shared Library
add_library(memoryMapAndSemaphore SHARED ${sources})

# copy the library to ${CMAKE_INSTALL_PREFIX}/../lib
# on linux: /usr/lib
install(TARGETS memoryMapAndSemaphore DESTINATION ../lib)

# copy the Headers to ${CMAKE_INSTALL_PREFIX}/include --> /usr/local/include
install(DIRECTORY ${includeDir}/ 
		DESTINATION include 
		FILES_MATCHING PATTERN "*.h"		#only header files
		PATTERN .svn EXCLUDE 				#exclude .svn directory
		)
