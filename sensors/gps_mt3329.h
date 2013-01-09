#ifndef GPS_MTK_H
#define GPS_MTK_H

#define GPS_SERIAL_DEVICE SD3
#define GPS_PRIORITY		NORMALPRIO-5

#define MTK_SET_BINARY  "$PGCMD,16,0,0,0,0,0*6A\r\n"
#define MTK_SET_NMEA    "$PGCMD,16,1,1,1,1,1*6B\r\n"

#define MTK_OUTPUT_1HZ  "$PMTK220,1000*1F\r\n"
#define MTK_OUTPUT_2HZ  "$PMTK220,500*2B\r\n"
#define MTK_OUTPUT_4HZ  "$PMTK220,250*29\r\n"
#define MTK_OTUPUT_5HZ  "$PMTK220,200*2C\r\n"
#define MTK_OUTPUT_10HZ "$PMTK220,100*2F\r\n"

#define MTK_BAUD_RATE_38400 "$PMTK251,38400*27\r\n"

#define SBAS_ON  "$PMTK313,1*2E\r\n"
#define SBAS_OFF "$PMTK313,0*2F\r\n"

#define WAAS_ON  "$PSRF151,1*3F\r\n"
#define WAAS_OFF "$PSRF151,0*3E\r\n"

enum mtk_fix_type {
	FIX_NONE = 1,
	FIX_2D = 2,
	FIX_3D = 3
};

//float cosine (float x);
//float sine (float x);

//void swap_endian_long (uint8_t *x);
void parse_gps_sentence (char *s);

void gps_mtk_start(void);

#endif
