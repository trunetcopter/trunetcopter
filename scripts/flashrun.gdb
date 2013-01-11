target remote localhost:3333

monitor reset halt
monitor wait_halt

monitor poll
monitor flash probe 0
monitor flash erase_sector 0 10 11
monitor flash write_image build/trunetcopter.bin 0x08000000 bin
monitor verify_image build/trunetcopter.bin 0x08000000
monitor soft_reset_halt
thbreak main
continue

