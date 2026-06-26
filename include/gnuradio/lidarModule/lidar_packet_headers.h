/* -*- c++ -*- */
/*
 * Copyright 2026 Kevin Hermstein
 *
 */

#ifndef LIDAR_PACKET_HEADERS_H
#define LIDAR_PACKET_HEADERS_H

#include <gnuradio/lidarModule/lidar_header_types.h>

class header_lidar
{
public:
    // size: 42 bytes: ETH, IP, UDP (8 byte)
    uint64_t seqnum;

    //header_seq_num(void) { 
    //   seqnum = 0; 
    //    length = 0;
    //};
};

#endif /* LIDAR_PACKET_HEADERS_H */
