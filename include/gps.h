#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include <stdbool.h>

#define GPS_BUFFER_SIZE 128

typedef struct {
    // Time and date
    int hours, minutes, seconds;
    int day, month, year;

    // Position
    double latitude;
    double longitude;
    char lat_dir;
    char lon_dir;

    // Fix info
    int fix_quality;      // 0 = invalid, 1 = GPS fix, 2 = DGPS fix
    int fix_quality_3d;   // 2 = 2D, 3 = 3D
    bool has_fix;
    bool has_3d_fix;

    // Satellites
    int satellites;
    double hdop;
    double altitude_m;
    double geoid_height;

    // Movement
    double speed_knots;
    double track_angle_deg;

    // Raw NMEA sentence
    char last_sentence[GPS_BUFFER_SIZE];
} gps_data_t;

uint8_t init_gps(void);
uint8_t do_gps(const char *nmea_sentence, gps_data_t *gps);

#endif