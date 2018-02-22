#!/bin/bash
#-std=gnu11  gives this: cc1plus: warning: command line option ‘-std=gnu11’ is valid for C/ObjC but not for C++

compile_flags="-pthread -lm -Wall -pipe -O3 -I. -fno-strict-aliasing -ffunction-sections -fdata-sections -fno-strict-aliasing -fmessage-length=0 -Wl,--gc-sections"
# link_flags="-pthread -lm -Wl,--gc-sections"
src_dir="src"
obj_dir="obj"
exec_file="mcp"



#	rm -rf src/*
	[ ! -d $src_dir ] && mkdir $src_dir
	cp /mnt/dev/OPi_MainConrol/sunxi_gpio.*	$src_dir/						&&
	cp /mnt/dev/OPi_MainConrol/softPwm.*	$src_dir/						&&
	cp /mnt/dev/OPi_MainConrol/softServo.*	$src_dir/						&&
	cp /mnt/dev/OPi_MainConrol/mcp.*		$src_dir/						&&
#	cp /mnt/dev/OPi_MainConrol/TCPServer.*	$src_dir/						&&

	echo "Main Control Build script v3"
	echo "============================"
	echo "Compiling..."

	[ ! -d $obj_dir ] && mkdir $obj_dir
	[ -f $exec_file ] && rm -f $exec_file
#	rm -rf obj/*

	gcc $compile_flags -c -o $obj_dir/sunxi_gpio.o	$src_dir/sunxi_gpio.cpp	&&
	gcc $compile_flags -c -o $obj_dir/softPwm.o		$src_dir/softPwm.cpp	&&
	gcc $compile_flags -c -o $obj_dir/softServo.o	$src_dir/softServo.cpp	&&
	gcc $compile_flags -c -o $obj_dir/mcp.o			$src_dir/mcp.cpp		&&
#	gcc $compile_flags -c -o $obj_dir/TCPServer.o	$src_dir/TCPServer.cpp	&&

	echo "Linking..."														&&
#	gcc -O3 -c -I. -fno-strict-aliasing -ffunction-sections -fdata-sections -fno-strict-aliasing -fmessage-length=0 -lpthread -Wl,--gc-sections -o mcp obj/mcp.o obj/piHiPri.o obj/sunxi_gpio.o obj/softPwm.o obj/softServo.o
#	gcc -lpthread -Wl,--gc-sections -o mcp obj/mcp.o obj/piHiPri.o obj/sunxi_gpio.o obj/softPwm.o obj/softServo.o
#gcc $compile_flags -o $exec_file $obj_dir/mcp.o $obj_dir/sunxi_gpio.o $obj_dir/softPwm.o $obj_dir/softServo.o $src_dir/TCPServer.cpp &&
gcc $compile_flags -o $exec_file $obj_dir/mcp.o $obj_dir/sunxi_gpio.o $obj_dir/softPwm.o $obj_dir/softServo.o &&
	echo "Building Done."
	if [ -x $exec_file ]
	then
		echo "Starting the program..."
		./$exec_file -r
	fi

