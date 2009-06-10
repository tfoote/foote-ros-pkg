all: ucview

TARBALL     = build/ucview-0.23.tar.gz
TARBALL_URL = http://unicap-imaging.org/downloads/ucview-0.23.tar.gz
UNPACK_CMD  = tar xzf
SOURCE_DIR  = build/ucview-0.23

include $(shell rospack find mk)/download_unpack_build.mk

ucview: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && ./configure --prefix=$(PWD)/build/ucview UCVIEW_CFLAGS=`rospack find unicap`/build/unicap/include/unicap UCVIEW_LIBS=`rospack find unicap`/build/unicap/lib
	cd $(SOURCE_DIR) && make UCVIEW_CFLAGS=`rospack find unicap`/build/unicap/include/unicap UCVIEW_LIBS=`rospack find unicap`/build/unicap/lib && make install


clean:
	rm -rf $(SOURCE_DIR)

wipe: clean
	rm -rf build
