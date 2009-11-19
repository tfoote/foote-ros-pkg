all:build


TARBALL     = build/Phidgetlinux_2.1.6.20091020.tar.gz
TARBALL_URL = http://www.phidgets.com/downloads/libraries/Phidgetlinux_2.1.6.20091020.tar.gz
MD5SUM_FILE = Phidgetlinux.md5sum
UNPACK_CMD  = tar xzf
SOURCE_DIR  = build/Phidgetlinux

include $(shell rospack find mk)/download_unpack_build.mk

build: $(SOURCE_DIR)/unpacked
	make  -C $(SOURCE_DIR)/phidget21
	mkdir -p include 
	cp $(SOURCE_DIR)/phidget21/phidget21.h include/
	mkdir -p lib
	cp $(SOURCE_DIR)/phidget21/libphidget21.so lib/


clean:
	rm -rf $(SOURCE_DIR)

wipe: clean
	rm -rf build
