//
// File:   MemoryMappedFiles.h
// Sourcecode:    Oezguer Sen; sen@gps-stuttgart.de
// origin source: Martin Kompf; www.kompf.de
//
// Created on 8. September 2010, 14:11
//
#ifndef MEMMORY_MAPPED_FILES_H
#define MEMMORY_MAPPED_FILES_H

#include <string>
//#define DEFAULT_DIR /tmp/youBotMappedFile
const std::string DEFAULT_DIR="/tmp/youBotMappedFile";

namespace memmap {

//! A struct to save the begin of the address for the mapped memory and its length
struct MappedRegionS 
{
    void * addr;
    int length;
};

//! The pointer-type of the MappedRegionS
typedef struct MappedRegionS* MappedRegion;

//! A class to map a file into the memory of a process,
//! which can be shared with other processes
class MemmoryMappedFiles
{
  public:

    //! Default constructor.
    MemmoryMappedFiles(){ pMemMapReg = NULL; }
	
	//! Overloaded constructor for creating mapped file during initation.
    MemmoryMappedFiles(const char* path, int length, int id=0);

    //! Destructor.
    ~MemmoryMappedFiles();
  
    //! Memberfunction to create shared memory mapped file
    //! Input parameters:
    //!   path:   The name of the file to be mapped.
    //!           The file is created if it doesn't exist.
    //!   id:     A system wide unique identifier of the mapped region --> used only in windows
    //!   length: The length of the mapped region.
    //!           This parameter is used only if the file doesn't exist, 
    //!           otherwise the length of the existing file is used.
    //!	  fileChecked: Check, if another method/process has checked the existens of a memory mapped file
    //!				   If false, than the existence check wasn't performed and must be done in this method
    //!				   If true, the file was checked and is existing.
    //! Return value:			
    //!   0: Successfull completion.
    //!  -1: An error occured. The cause is specified in the variable errno.  
    //!  -2: Not enough memory available.
    //!  -3: An error occured. 
    //!      The cause may be determined by GetLastError (Windows only).
    int mapFile( const char* path, int length, int id=0, bool fileChecked=false );
    
    //! Memberfunction to open an existing shared memory mapped file, which was 
    //! created by another process. This is necessary, if you want only
    //! one process to be responsible for the creation of the memory mapped file.
    //! Input parameters:
    //!   path:   The name of the file to be opened.
    //!   id:     A system wide unique identifier of the mapped region --> used only in windows 
    //! Return value:
    //!   0: Successfull completion.
    //!  -1: An error occured. The cause is specified in the variable errno.  
    //!  -2: Not enough memory available.
    //!  -3: An error occured. 
    //!      The cause may be determined by GetLastError (Windows only).
    int openMappedFile(const char* path, int id=0);

    //! Unmap a file from the memory 
    void unmapFile( );

    //! Get the (local) address of a mapped region
    //! Input parameter:
    //!   hReg:   The handle of the mapped region created by memmap
    //! Return value:
    //!   The address
    char* getAddr();

    //! Get the length of a mapped region
    //! Input parameter:
    //!   hReg:   The handle of the mapped region created by memmap
    //! Return value:
    //!   The length in bytes
    int getLength();

  private:
    
    //! A handle that describes the mapped region.
    MappedRegion pMemMapReg;
    
};

} //namespace memmap

#endif //MEMMORY_MAPPED_FILES
