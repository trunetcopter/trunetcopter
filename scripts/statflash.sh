#!/bin/bash

arm-none-eabi-size build/obj/*.o | \
	awk '{print ($1+$2)" "($2+$3)" "$6}' | \
	sort -n | \
	awk '{printf "%s%-10s%s%-10s%s\n","Flash: ", $1, "RAM: ", $2, $3}'
