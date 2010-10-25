#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>

#ifdef _WIN32
  #include <windows.h>
  #include <process.h>
#else
  #include <sys/mman.h>
  #include <unistd.h>
#endif

#include <stdlib.h>
#include <string.h>
#include <iostream>
// #include <MemoryMappedFiles.h>
#include <MemoryMappedFiles.h>

namespace memmap {

MemmoryMappedFiles::MemmoryMappedFiles(const char* path, int length, int id) : pMemMapReg(0) 
{
  int rc = mapFile(path, length, id);
  
  if(rc != 0) {
	printf("Coudln't map file. Error Code: %d. Does directory for this file exist?Is the file or the length parameter greater than 0? Or check permissions to access this file: %s\n", rc, path);
	exit(1);
  }
}  
  
MemmoryMappedFiles::~MemmoryMappedFiles()
{
  unmapFile();
}

int MemmoryMappedFiles::mapFile(const char* path, int length, int id, bool fileChecked) 
{  
  struct stat stat_buf;
  int exists;
  char buffer[1024];
  int res;

#ifdef _WIN32
  PSECURITY_DESCRIPTOR pSD;
  HANDLE hFile, hMem;
  SECURITY_ATTRIBUTES  sa;
  DWORD sz;
#else
  int fdes;
#endif
 
  // Check if there is another MappedRegion. 
  // if so, delete it first. Reason: one object shall allocate only one mapped file
  if (pMemMapReg != NULL) {
	unmapFile();
  } 

  
  MappedRegion reg = (MappedRegion)malloc( sizeof( struct MappedRegionS));
  
  if (NULL == reg) { 
    return -3;
  }
  
  pMemMapReg = reg;

	/* check if the existence of the file was checked before.
	 * (1) If not: 
	 *				- than check if the file exists and than determine its length 
	 *				- or set exists to 0 and set the length to the value which was assigned to the method
	 * (2) If yes:
	 *				- set exists to 1 
	 *				- AND set the length to the size, which was assigned to the method 
	 *					--> in this case the length of the existing file was determined 
	 *							by another method/process.
	 */
	if (!fileChecked) {
	//(1)
		res = stat( path, &stat_buf);

		if (res < 0) {
			if (errno == ENOENT) 
				exists = 0;
			else 
				return -1;
		}
		else {
			exists = 1;
		}

		if (exists) {
			reg->length = stat_buf.st_size;
		}
		else {
			reg->length = length;
		}
	}
	//(2)
	else {
		exists = 1;
		reg->length = length;  	
	}
    
#ifdef _WIN32
    /* create security descriptor (needed for Windows NT) */
    pSD = (PSECURITY_DESCRIPTOR) malloc( SECURITY_DESCRIPTOR_MIN_LENGTH );
    if( pSD == NULL ) return -2;

    InitializeSecurityDescriptor(pSD, SECURITY_DESCRIPTOR_REVISION);
    SetSecurityDescriptorDacl(pSD, TRUE, (PACL) NULL, FALSE);

    sa.nLength = sizeof(sa);
    sa.lpSecurityDescriptor = pSD;
    sa.bInheritHandle = TRUE;  

    /* create or open file */
    if (exists) {
        hFile = CreateFile ( path, GENERIC_READ | GENERIC_WRITE, 
            FILE_SHARE_READ | FILE_SHARE_WRITE, &sa, OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL, NULL);  
    }
    else {
        hFile = CreateFile ( path, GENERIC_READ | GENERIC_WRITE, 
            FILE_SHARE_READ | FILE_SHARE_WRITE, &sa, OPEN_ALWAYS,
            FILE_ATTRIBUTE_NORMAL, NULL);  
    }
    if (hFile == INVALID_HANDLE_VALUE) {   
        free( pSD);
        return -3;
    }
    if (! exists) {
        /* ensure that file is long enough and filled with zero */
        memset( buffer, 0, sizeof(buffer));
        for (int i = 0; i < reg->length/sizeof(buffer); ++i) {
            if (! WriteFile( hFile, buffer, sizeof(buffer), &sz, NULL)) {
                return -3;
            }
        }
        if (! WriteFile( hFile, buffer, reg->length, &sz, NULL)) {
            return -3;
        }
    }
        
    /* create file mapping */
    sprintf( buffer, "%d", id);
    hMem = CreateFileMapping( hFile, &sa, PAGE_READWRITE, 0, 
            reg->length, buffer);
    free( pSD);
    if (NULL == hMem) return -3;

    /* map the file to addregs */
    reg->addr = MapViewOfFile( hMem, FILE_MAP_ALL_ACCESS, 0, 0, 0);
    if (NULL == reg->addr) return -3;

    CloseHandle( hFile);
    CloseHandle( hMem);

#else
    /* UNIX */
    if (exists) 
    {
        /* open mapped file */
        fdes = open( path, O_RDWR, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH );
        if (fdes < -1) return -1;
    }
    else /* not exists */ 
    {
        /* create mapped file */
        fdes = open( path, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR );
        if (fdes < -1) return -1;
        /* change modes --> if the file is created in the tmp-directory, the modes has to be changed that way*/
        if( fchmod( fdes, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH ) != 0 ) {
			printf("could not change modes for file: %s\n", path);
			return -1;
		}
		
        /* ensure that file is long enough and filled with zero */
        memset( buffer, 0, sizeof(buffer));
        
        for (int i = 0; i < reg->length/sizeof(buffer); ++i) {    
	    	if (write( fdes, buffer, sizeof(buffer)) != sizeof(buffer)) {
                return -1;
            }
        }
        
        if (write( fdes, buffer, reg->length) != reg->length) {
            return -1;
        }
    }

    /* map the file to m_addr */
    reg->addr = mmap( NULL, reg->length, PROT_READ | PROT_WRITE, MAP_SHARED, fdes, 0);  
    close( fdes);
    if (reg->addr == (void *)-1) return -1;
#endif

    return 0;
  
}


int MemmoryMappedFiles::openMappedFile(const char* path, int id)
{
  struct stat stat_buf;
   
  if ( stat( path, &stat_buf) < 0) {
  	return -1; //file does not exist
  }
  else {
    int ec = mapFile( path, stat_buf.st_size, id, true);
    return ec;
  }
}



void MemmoryMappedFiles::unmapFile() 
{
	MappedRegion reg = pMemMapReg;
	if (reg) {
#ifdef _WIN32
	if (reg->addr) {
	UnmapViewOfFile( reg->addr);
    }
#else
	if (reg->addr) {
	munmap( reg->addr, reg->length);
    }
#endif
    free( reg); 
  }
  pMemMapReg = 0;
}

char* MemmoryMappedFiles::getAddr() 
{
  return (char *)(pMemMapReg->addr);
}

int MemmoryMappedFiles::getLength()
{
   return pMemMapReg->length;
}

  
} //namespace memmap
