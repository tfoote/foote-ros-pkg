all: gdata-python

TARBALL     = build/gdata-1.3.2.tar.gz
TARBALL_URL = http://gdata-python-client.googlecode.com/files/gdata-1.3.2.tar.gz
UNPACK_CMD  = tar xfz
SOURCE_DIR  = build/gdata-1.3.2

include $(shell rospack find mk)/download_unpack_build.mk

gdata-python: $(SOURCE_DIR)/unpacked

clean:
	rm -rf $(SOURCE_DIR)

wipe: clean
	rm -rf build
