/* -*- c++ -*- */
/*
 * Copyright 2026 Kevin Hermstein.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_LIDARMODULE_VELO_TO_XYZ_IMPL_H
#define INCLUDED_LIDARMODULE_VELO_TO_XYZ_IMPL_H

#include <gnuradio/lidarModule/Velo_to_XYZ.h>
#include <gnuradio/lidarModule/ofdm_utils.h>
#include <cstdint>

namespace gr {
namespace lidarModule {

class Velo_to_XYZ_impl : public Velo_to_XYZ
{
private:
    //Elevation angles of all 16 lasers fired for a single aximuth angle
    const double ELEVATION_ANGLES_DEG[32] = {
        -25,   -1, -1.667,   -15.639, -11.31,   0,  -0.667,   -8.843,
        -7.254, 0.333, -0.333, -6.148, -5.333, 1.333, 0.667, -4,
        -4.667, 1.667, 1, -3.667, -3.333, 3.333, 2.333, -2.667,
        -3, 7, 4.667, -2.333, -2, 15, 10.333, -1.333
    };
    double d_elevation_angles_rad[32];
    const double AZIMUTH_OFFSETS_DEG[32] = {
        -1.4,   4.2, -1.4,   1.4, -1.4,   1.4,  -4.2,   1.4,
        -1.4, 4.2, -1.4, 1.4, -4.2, 1.4, -4.2, 1.4,
        -1.4, 4.2, -1.4, 4.2, -4.2, 1.4, -1.4, 1.4,
        -1.4, 1.4, -1.4, 4.2, -4.2, 1.4, -1.4, 1.4
    };
    double d_azimuth_offsets_rad[32];
    double d_firing_time = 0.000055296;
    double d_laser_pair_time = 0.000002304;
    double d_laser_pair_drift_deg;
    double d_curr_dps; //rotation per second
    const uint8_t FLAG[2] = {0xFF, 0xEE};

    //12 blocks * 32 lasers (points) per firing
    int d_points_per_packet = 12 * 32;

    int d_rpm;
    bool d_single;
    const std::string& d_len_tag_key;

    bool d_printed_frame=false;

    std::vector<PointXYZ> d_points;
    std::vector<float> d_float_points;

    FILE* d_logfile;

protected:
    int calculate_output_stream_length(const gr_vector_int& ninput_items);
    float deg_to_rad(double degree);
    bool check_flag(const uint8_t* bytes);
    uint16_t rev_bytes_to_uint16(const uint8_t* bytes);
    int process_block(const uint8_t* data_bytes, std::vector<float>& points, int block);

public:

    Velo_to_XYZ_impl(int rpm, bool single, const std::string& len_tag_key);
    ~Velo_to_XYZ_impl();

    // Where all the action really happens
    int work(int noutput_items,
             gr_vector_int& ninput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items);
};

} // namespace lidarModule
} // namespace gr

#endif /* INCLUDED_LIDARMODULE_VELO_TO_XYZ_IMPL_H */
