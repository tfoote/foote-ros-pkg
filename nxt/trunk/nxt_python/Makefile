all: nxt-python

TARBALL     = build/nxt-python-1.1.1.tar.gz
TARBALL_URL = http://nxt-python.googlecode.com/files/nxt-python-1.1.1.tar.gz
UNPACK_CMD  = tar xfz
SOURCE_DIR  = build/nxt-python-1.1.1
include $(shell rospack find mk)/download_unpack_build.mk

INSTALL_DIR = $(shell rospack find nxt_python)


nxt-python: $(INSTALL_DIR)/installed

$(INSTALL_DIR)/installed: $(SOURCE_DIR)/unpacked
	rm -rf src
	cd $(SOURCE_DIR) && python setup.py install --prefix=$(INSTALL_DIR) --install-purelib=$(INSTALL_DIR)/src
	patch -p0<ultrasonic.patch 
	patch -p0 src/nxt/sensor.py gyro.patch 
	patch -p0 src/nxt/sensor.py colorsensor.patch 
	touch $(INSTALL_DIR)/installed

clean:
	rm -rf $(SOURCE_DIR) bin lib src installed

wipe: clean
	rm -rf build
