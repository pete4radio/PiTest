// # SPDX-FileCopyrightText: 2017 Tony DiCola for Adafruit Industries
// # SPDX-FileCopyrightText: 2021 James Carr
// #
// # SPDX-License-Identifier: MIT

#include "gps.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "hardware/uart.h"
#include "pico/stdlib.h"

#define GPS_UART_ID uart0
#define GPS_BAUDRATE 9600
#define GPS_UART_TX_PIN 0
#define GPS_UART_RX_PIN 1

extern char buffer_UART, char buffer_GPS, int buflen;


static double nmea_to_decimal(const char *nmea, char dir) {
    if (!nmea || !*nmea) return 0.0;
    double val = atof(nmea);
    int deg = (int)(val / 100);
    double min = val - deg * 100;
    double dec = deg + min / 60.0;
    if (dir == 'S' || dir == 'W') dec = -dec;
    return dec;
}

// Helper: parse int, returns 0 if empty or invalid
static int parse_int(const char *s) {
    if (!s || !*s) return 0;
    return atoi(s);
}

// Helper: parse double, returns 0.0 if empty or invalid
static double parse_double(const char *s) {
    if (!s || !*s) return 0.0;
    return atof(s);
}

// Helper: parse time in hhmmss format
static void parse_time(const char *s, gps_data_t *gps) {
    if (!s || strlen(s) < 6) return;
    gps->hours = (s[0]-'0')*10 + (s[1]-'0');
    gps->minutes = (s[2]-'0')*10 + (s[3]-'0');
    gps->seconds = (s[4]-'0')*10 + (s[5]-'0');
}

// Helper: parse date in ddmmyy format
static void parse_date(const char *s, gps_data_t *gps) {
    if (!s || strlen(s) < 6) return;
    gps->day = (s[0]-'0')*10 + (s[1]-'0');
    gps->month = (s[2]-'0')*10 + (s[3]-'0');
    gps->year = 2000 + (s[4]-'0')*10 + (s[5]-'0');
}

// Parse GGA sentence
static void parse_gga(char **fields, gps_data_t *gps) {
    parse_time(fields[1], gps);
    gps->latitude = nmea_to_decimal(fields[2], fields[3][0]);
    gps->longitude = nmea_to_decimal(fields[4], fields[5][0]);
    gps->fix_quality = parse_int(fields[6]);
    gps->satellites = parse_int(fields[7]);
    gps->hdop = parse_double(fields[8]);
    gps->altitude_m = parse_double(fields[9]);
    gps->geoid_height = parse_double(fields[11]);
    gps->has_fix = gps->fix_quality > 0;
}

// Parse RMC sentence
static void parse_rmc(char **fields, gps_data_t *gps) {
    parse_time(fields[1], gps);
    gps->latitude = nmea_to_decimal(fields[3], fields[4][0]);
    gps->longitude = nmea_to_decimal(fields[5], fields[6][0]);
    gps->speed_knots = parse_double(fields[7]);
    gps->track_angle_deg = parse_double(fields[8]);
    parse_date(fields[9], gps);
    gps->has_fix = (fields[2][0] == 'A');
}

// Parse GLL sentence
static void parse_gll(char **fields, gps_data_t *gps) {
    gps->latitude = nmea_to_decimal(fields[1], fields[2][0]);
    gps->longitude = nmea_to_decimal(fields[3], fields[4][0]);
    parse_time(fields[5], gps);
    gps->has_fix = (fields[6][0] == 'A');
}

// Parse GSA sentence
static void parse_gsa(char **fields, gps_data_t *gps) {
    gps->fix_quality_3d = parse_int(fields[2]);
    gps->pdop = parse_double(fields[15]);
    gps->hdop = parse_double(fields[16]);
    gps->vdop = parse_double(fields[17]);
    gps->has_3d_fix = gps->fix_quality_3d >= 3;
}

// Parse GSV sentence (satellite info, not fully implemented)
static void parse_gsv(char **fields, gps_data_t *gps) {
    gps->satellites = parse_int(fields[3]);
}

// Split NMEA sentence into fields, returns number of fields
static int split_fields(char *sentence, char **fields, int max_fields) {
    int n = 0;
    char *p = sentence;
    while (n < max_fields && p && *p) {
        fields[n++] = p;
        p = strchr(p, ',');
        if (p) {
            *p = 0;
            p++;
        }
    }
    return n;
}

// Validate NMEA checksum
static bool validate_nmea_checksum(const char *sentence) {
    if (sentence[0] != '$') return false;
    const char *star = strchr(sentence, '*');
    if (!star) return false;
    uint8_t sum = 0;
    for (const char *p = sentence + 1; p < star; ++p) sum ^= (uint8_t)(*p);
    uint8_t expected = (uint8_t)strtol(star + 1, NULL, 16);
    return sum == expected;
}

// Main GPS sentence parser
uint8_t do_gps(const char *nmea_sentence, gps_data_t *gps) {
    if (!nmea_sentence || !gps) return PICO_ERROR_GENERIC;
    if (!validate_nmea_checksum(nmea_sentence)) return PICO_ERROR_GENERIC;

    strncpy(gps->last_sentence, nmea_sentence, GPS_BUFFER_SIZE-1);
    gps->last_sentence[GPS_BUFFER_SIZE-1] = 0;

    // Copy sentence for tokenizing
    char sentence[GPS_BUFFER_SIZE];
    strncpy(sentence, nmea_sentence, GPS_BUFFER_SIZE-1);
    sentence[GPS_BUFFER_SIZE-1] = 0;

    // Remove $ and checksum
    char *star = strchr(sentence, '*');
    if (star) *star = 0;
    char *body = sentence + 1;

    // Split into fields
    char *fields[20] = {0};
    int nfields = split_fields(body, fields, 20);
    if (nfields < 1) return PICO_ERROR_GENERIC;

    // Identify sentence type
    if (strncmp(fields[0], "GGA", 3) == 0) {
        parse_gga(fields, gps);
    } else if (strncmp(fields[0], "RMC", 3) == 0) {
        parse_rmc(fields, gps);
    } else if (strncmp(fields[0], "GLL", 3) == 0) {
        parse_gll(fields, gps);
    } else if (strncmp(fields[0], "GSA", 3) == 0) {
        parse_gsa(fields, gps);
    } else if (strncmp(fields[0], "GSV", 3) == 0) {
        parse_gsv(fields, gps);
    } else {
        // Not a supported sentence
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}