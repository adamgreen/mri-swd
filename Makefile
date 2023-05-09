.PHONY: init all clean

all :
	@echo Building...
	@mkdir build/ 2>/dev/null; exit 0
	@cd build; cmake ..
	@cd build; make

#	-@cd pico-sdk; patch -N -p1 <../pico-sdk.patch 1>/dev/null ;exit 0

init :
	@echo Initializing git submodules...
	@git submodule update --init
	@cd pico-sdk; git submodule update --init
	@cd mri; git submodule update --init
	-@cd pico-sdk; patch -N -p1 --reject-file ../rejects.txt <../pico-sdk.patch 1>/dev/null ;exit 0
	-@rm rejects.txt 2>/dev/null ;exit 0

clean :
	@echo Removing build output for clean build...
	@rm -rf build/
