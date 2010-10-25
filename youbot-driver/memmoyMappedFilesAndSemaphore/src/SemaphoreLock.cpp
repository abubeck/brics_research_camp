//#include <SemaphoreLock.h>
#include <SemaphoreLock.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef _WIN32
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#endif

namespace semlock {

int SemaphoreLock::createLock( int key) {
#ifdef _WIN32
    char semname[64];
    sprintf( semname, "sem%08x", key);
    mSemaphore = CreateSemaphore( NULL, 1, 1, semname);
    return ( mSemaphore != NULL)? 0 : -1;
#else
	printf("KEY: %i\n", key);
    mSemaphore = semget( key, 1, IPC_CREAT|0600);
    if ( mSemaphore > 0) unlock();
    return ( mSemaphore >= 0)? 0 : -1;
#endif
}

int SemaphoreLock::openLock( int key ) {
#ifdef _WIN32
    char semname[64];
    sprintf( semname, "sem%08x", key);
    mSemaphore = OpenSemaphore( SEMAPHORE_ALL_ACCESS, FALSE, semname);
    return ( mSemaphore != NULL)? 0 : -1;
#else
    printf("KEY: %i\n", key);
    mSemaphore = semget( key, 1, 0);
    return ( mSemaphore >= 0)? 0 : -1;
#endif
}

void SemaphoreLock::destroyLock() {
#ifdef _WIN32
    CloseHandle( mSemaphore);
#else
    semctl( mSemaphore, 0, IPC_RMID);
#endif
}
 
int SemaphoreLock::lock() {
#ifdef _WIN32
    return (WAIT_FAILED == WaitForSingleObject( mSemaphore, INFINITE));
#else
    struct sembuf sop;

	sop.sem_num = 0;
	sop.sem_op = -1;
	sop.sem_flg = 0;

	return semop( mSemaphore, &sop, 1);
#endif
}

int SemaphoreLock::unlock(){
#ifdef _WIN32
    return !ReleaseSemaphore ( mSemaphore, 1, NULL);
#else
    struct sembuf sop;

	if (semctl( mSemaphore, 0, GETVAL) > 0)
		return 0; /* already unlocked */

	sop.sem_num = 0;
	sop.sem_op = 1;
	sop.sem_flg = 0;

	return semop( mSemaphore, &sop, 1);
#endif
}

} //namespace
