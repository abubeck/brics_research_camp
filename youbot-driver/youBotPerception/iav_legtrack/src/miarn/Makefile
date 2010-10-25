CC = g++
CFLAGS = -Wall -pedantic

libmiarn: *.cpp *.hpp
	$(CC) $(CFLAGS) -c *.cpp
	ar rcuv $@.a *.o

libmiarn_debug: *.cpp *.hpp
	$(CC) $(CFLAGS) -g -c *.cpp
	ar rcuv $@.a *.o

clean:
	rm -f *.o *~ libmiarn.a libmiarn_debug.a
