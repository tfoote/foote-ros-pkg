all: installed

SVN_DIR = ccv_linux
SVN_URL = http://nuicode.svnrepository.com/svn/tbeta/trunk/tbeta/Linux
SVN_REVISION = -r 179
SVN_PATCH = intrepid_ffmpeg.patch
include $(shell rospack find mk)/svn_checkout.mk


#build: SVN_UP_REVERT_PATCH gazebo
installed: $(SVN_DIR) 
	#	cd $(SVN_DIR) && make
	ln -sf ccv_linux/libs/fmodex/lib/libfmodex-4.22.00.so ccv_linux/libs/fmodex/lib/libfmodex.so 
	ln -sf ccv_linux/libs/fmodex/lib/libfmodexp-4.22.00.so ccv_linux/libs/fmodex/lib/libfmodexp.so 
	touch installed

clean:
	rm -rf  installed 
	-make -C $(SVN_DIR) clean
wipe: clean
	rm -rf $(SVN_DIR) 

