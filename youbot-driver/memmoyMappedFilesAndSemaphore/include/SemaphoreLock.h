//
// File:   MemoryMappedFiles.h
// Sourcecode:    Oezguer Sen; sen@gps-stuttgart.de
// origin source: Martin Kompf; www.kompf.de
//
// Created on 8. September 2010, 18:07
//
#ifndef SEMAPHORE_LOCK_H
#define SEMAPHORE_LOCK_H

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

namespace semlock {

#ifdef _WIN32
typedef HANDLE SemLock;
#else
typedef int SemLock;
#endif

// Number is generated from this table
//
// 0 1 2 3 4 5 6 7 8 9
// ------------------- 
// A B C D E F G H I J
// K L M N O P Q R S T
// U V W X Y Z
// -------------------
// ==> STANDARD = 89033073
// if no ID is defined, every process is using the standard key
#define STANDARDKEY 89033073


//! A class to create a semaphore easily
class SemaphoreLock 
{

  public:
	
	//! default constructor
	SemaphoreLock(){}
		
	//! Destructor
	~SemaphoreLock(){ unlock(); }
	
	//! create a new semaphore, if it does not exist; otherwise see below.
	//! Input Parameter:
	//!   key_t : system unique key of the semaphore
	//! Return value:
	//!    0 : OK 
	//!   -1 : error
	int createLock( int key = STANDARDKEY);


	//! open an existing semaphore 
	//! Input Parameter:
	//!   key_t : system unique key of the semaphore
	//! Return value:
	//!    0 : OK 
	//!   -1 : error
	int openLock( int key = STANDARDKEY);


	//! destroy a semaphore 
	void destroyLock();


	//! request a lock 
	//! Return value:
	//!   0 : OK
	//!   else : error 
	int lock();


	//! release a lock
	//! Input Parameter:
	//!   s : the semaphore handle
	//! Return value:
	//!   0 : OK
	//!   else : error 
	int unlock();
	
  private:
	SemLock mSemaphore;
	
	
}; //class

} //namespace semlock
#endif // SEMAPHORE_LOCK_H