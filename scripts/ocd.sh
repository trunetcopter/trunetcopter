#!/bin/bash

openocd -c "gdb_memory_map enable" \
	-c "gdb_flash_program enable" \
	-c "reset_config trst_and_srst" \
	-f /usr/share/openocd/scripts/interface/olimex-arm-usb-ocd.cfg \
	-f ./stm32f4x.cfg \
	#-c "log_output openocd.log" \
