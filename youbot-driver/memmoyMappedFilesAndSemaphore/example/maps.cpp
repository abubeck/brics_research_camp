#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>


#include <MemoryMappedFiles.h>
#include <SemaphoreLock.h>

using namespace memmap;
using namespace semlock;

int main( int argc, char **argv)
{
    int rc, sum, i, key;
    int *a;
    
    int  memMapID   = 123123;  //i think it is used only in windows
    long memMapSize = 200*sizeof(int);
    
	MemmoryMappedFiles memFile;
	SemaphoreLock semLock;	
	if (argc == 1) {
		key = 12345;
	}
	else {
		key = atoi(argv[1]);
	}
	
	if ( semLock.openLock(key) < 0) {
	  
	  printf( "openLock failed\n");
	  exit(1);
    }
	
	memFile.openMappedFile("/tmp/semAndMemFile");
    
	a = (int *)memFile.getAddr(); 

  for(;;){
		
	  semLock.lock();
      for ( i=0; i<200; ++i) {
				if (i%10 == 0) printf("\n");
				printf( "%6d", a[i]);
      }
		 
      /* compute sum and compare it with a[199] */
      for (i=0, sum=0; i<199; ++i) {
	  sum += a[i];
      }
      printf("\n Summe = %d\n", sum);
      if (sum != a[199]) { 
		printf( " *** Fehler, %d != %d\n", sum, a[199]);
		sleep(1); 
	  }
	  semLock.unlock();
  }
    memFile.unmapFile();
}
