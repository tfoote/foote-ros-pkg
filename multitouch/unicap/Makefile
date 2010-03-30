all: installed


TARBALL     = build/unicap-0.9.7.tar.gz
TARBALL_URL = http://unicap-imaging.org/downloads/unicap-0.9.7.tar.gz
UNPACK_CMD  = tar xzf
SOURCE_DIR  = build/unicap-0.9.7

include $(shell rospack find mk)/download_unpack_build.mk

installed: wiped $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && ./configure --prefix=$(PWD)
	cd $(SOURCE_DIR) && make && make install
	touch installed

clean:
	rm -rf $(SOURCE_DIR)

wipe: clean
	rm -rf build installed

wiped: Makefile
	make wipe
	touch wiped
