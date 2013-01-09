import sys
import mavutil
import time

mavcon = mavutil.mavlink_connection("/dev/tty.usbserial-A8006AtT")

#print("Waiting for HEARTBEAT")
mavcon.wait_heartbeat()
#print("Heartbeat from APM (system %u component %u)" % (mavcon.target_system, mavcon.target_system))

while True:
	try:
		msg = mavcon.recv_match(type='RAW_IMU')
	except KeyboardInterrupt:
		sys.exit(0)
	if not msg:
		continue
	msg_type = msg.get_type()
	if msg_type == "RAW_IMU":
		#print "%d %d %d %d %d %d %d %d %d" % (msg.xacc, msg.yacc, msg.zacc, msg.xgyro, msg.ygyro, msg.zgyro, msg.xmag, msg.ymag, msg.zmag)
                print "%d 81 IMU_ACCEL_RAW %.4f %.4f %.4f" % (msg.time_usec, msg.xacc/4096.0, msg.yacc/4096.0, msg.zacc/4096.0)
                print "%d 81 IMU_GYRO_RAW %d %d %d" % (msg.time_usec, msg.xgyro, msg.ygyro, msg.zgyro)
                print "%d 81 IMU_MAG_RAW %d %d %d" % (msg.time_usec, msg.xmag, msg.ymag, msg.zmag)
