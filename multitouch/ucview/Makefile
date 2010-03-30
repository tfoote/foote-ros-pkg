all: installed

TARBALL     = build/ucview-0.23.tar.gz
TARBALL_URL = http://unicap-imaging.org/downloads/ucview-0.23.tar.gz
UNPACK_CMD  = tar xzf
SOURCE_DIR  = build/ucview-0.23

include $(shell rospack find mk)/download_unpack_build.mk

installed: wiped $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && ./configure --prefix=$(PWD) PKG_CONFIG_PATH=$(strip $(shell rospack export --lang=cpp --attrib=pkgconfig ucview))
	cd $(SOURCE_DIR) && make && make install
	touch installed

clean:
	rm -rf $(SOURCE_DIR)

wipe: clean
	rm -rf build installed

wiped: Makefile
	make wipe
	touch wiped
