#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include <iostream>

#include <MemoryMappedFiles.h>
#include <SemaphoreLock.h>

#ifdef _WIN32
#include <windows.h>

void sleep( int x) { Sleep( 1000*x); }
void usleep( int x) { Sleep( x/1000); }
#endif

/* get integer random number in range a <= x <= e */
int irand( int a, int e)
{
    double r = e - a + 1;
    return a + (int)(r * rand()/(RAND_MAX+1.0));
}

using namespace memmap;
using namespace semlock;

int main( int argc, char **argv)
{
    //printf("Starte mapt!");
    int rc, sum, i, key;
    int *a;

    int  memMapID   = 123123;  //i think it is used in windows only 
    long memMapSize = 200*sizeof(int);


	SemaphoreLock semLock;

	if (argc == 1) {
		key = 12345;
	}
	else {
		key = atoi(argv[1]);
	}
	
	//semLock.createLock();
	//semLock.destroyLock();
	if ( semLock.createLock(key) < 0) {
		printf( "createLock failed\n");
		if (semLock.openLock() < 0) {
			printf("openlock failed\n");
		}
	  exit(1);
    }

	MemmoryMappedFiles memFile("/tmp/semAndMemFile", memMapSize, memMapID);	

	a = (int *)memFile.getAddr( );

    srand( time(0));
    for (int j = 0;j< 1000;j++) {
	semLock.lock();
	  a[irand(0,198)] = irand(-1000, 1000);
	  /* compute sum and put it into a[199] */
	  for (i=0, sum=0; i<199; ++i) {
		  sum += a[i];
	  }
	  printf("summe: %d <--> i: %i\n", sum, j);
      a[199] = sum;
	  semLock.unlock();
    }
}
