#!/bin/bash

arm-none-eabi-size build/obj/*.o | \
	awk '{print ($3+$2)" "($2+$1)" "$6}' | \
	sort -n | \
	awk '{printf "%s%-10s%s%-10s%s\n","RAM: ", $1, "Flash: ", $2, $3}'
