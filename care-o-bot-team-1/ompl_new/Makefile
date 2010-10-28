include Makefile.params

INSTALL_LOCATION=installation

EXTRA_CMAKE_FLAGS=-DCMAKE_INSTALL_PREFIX=`pwd`/../$(INSTALL_LOCATION) -DOMPL_SOURCE:string="$(SOURCE_DIR)"

ompltoplevel:	all install

include $(shell rospack find mk)/cmake.mk

clean: extra-clean

install:
	cd build && make install

extra-clean:
	make -f Makefile.ompl clean
	rm -rf bin $(INSTALL_LOCATION)

wipe:	clean
