/* -*- c++ -*- */
/*
 * Copyright 2026 Kevin Hermstein.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "Velo_to_XYZ_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace lidarModule {

using input_type = char;
using output_type = float;
Velo_to_XYZ::sptr Velo_to_XYZ::make(int rpm, bool single, const std::string& len_tag_key)
{
    return gnuradio::make_block_sptr<Velo_to_XYZ_impl>(rpm,single,len_tag_key);
}


/*
 * The private constructor
 */
Velo_to_XYZ_impl::Velo_to_XYZ_impl(int rpm, bool single, const std::string& len_tag_key)
    : gr::tagged_stream_block(
          "Velo_to_XYZ",
          gr::io_signature::make(
              1 /* min inputs */, 1 /* max inputs */, sizeof(input_type)),
          gr::io_signature::make(
              1 /* min outputs */, 1 /*max outputs */, sizeof(output_type)*4),
          len_tag_key),
          d_rpm(rpm),
          d_single(single),
          d_len_tag_key(len_tag_key)
{
    
    for(int i=0; i<32; i++){
        d_elevation_angles_rad[i]=deg_to_rad(ELEVATION_ANGLES_DEG[i]);
        d_azimuth_offsets_rad[i]=deg_to_rad(AZIMUTH_OFFSETS_DEG[i]);
    }

    //double azimuth_res_deg = static_cast<double>(d_rpm) / 60 * 360;
    //d_azimuth_res_rad = deg_to_rad(d_azimuth_res_deg);
    //d_laser_pair_drift_deg = azimuth_res_deg * d_laser_pair_time;
    //std::cout << d_laser_pair_drift_deg << std::endl;
    d_points.reserve(d_points_per_packet);
    d_float_points.reserve(d_points_per_packet*4);

    d_logfile = std::fopen("/home/wimnet/repos/lidar_data_link/velo_xyz.log", "w");
    fprintf(d_logfile, "Intensity, Laser index, Azi, Range, Elevation, x,y,z\n");
}

/*
 * Our virtual destructor.
 */
Velo_to_XYZ_impl::~Velo_to_XYZ_impl() {
    fclose(d_logfile);
}

int Velo_to_XYZ_impl::calculate_output_stream_length(const gr_vector_int& ninput_items)
{
    int noutput_items = sizeof(PointXYZ) * d_points_per_packet;
    return noutput_items;
}

float Velo_to_XYZ_impl::deg_to_rad(double degree){
    return degree * (M_PI / 180.0);
}

uint16_t Velo_to_XYZ_impl::rev_bytes_to_uint16(const uint8_t* bytes){
    uint16_t decimal = bytes[0] | (bytes[1] << 8);
    return decimal;
}

bool Velo_to_XYZ_impl::check_flag(const uint8_t* bytes){
    if(bytes[0] == FLAG[0] && bytes[1] == FLAG[1]){
        return true;
    }
    return false;
}

int Velo_to_XYZ_impl::process_block(const uint8_t* data_bytes, std::vector<float>& points, int block){
    int num_points=0;
    int block_offset=block*32*4;
    int curr_offset=0;
    double x,y,z;
    if(!check_flag(&data_bytes[curr_offset])){
        std::cout << "UHOH" << std::endl;
        return 0;
    }
    curr_offset+=2;

    uint16_t curr_azimuth_deg_int = rev_bytes_to_uint16(&data_bytes[curr_offset]);
    double curr_azimuth_deg = static_cast<double>(curr_azimuth_deg_int) / 100;

    if(block < 11){
        uint16_t next_azimuth_deg_int = rev_bytes_to_uint16(&data_bytes[curr_offset + 100]);
        double next_azimuth_deg = static_cast<double>(next_azimuth_deg_int) / 100;
        double azimuth_diff = next_azimuth_deg - curr_azimuth_deg;
        if(azimuth_diff < -180){
            azimuth_diff += 360;
        }
        d_curr_dps = azimuth_diff / d_firing_time;
        d_laser_pair_drift_deg = d_curr_dps * d_laser_pair_time;  
    }

    curr_offset+=2;

    for(int i=0; i<32; i++){
        double true_azimuth_deg = curr_azimuth_deg - AZIMUTH_OFFSETS_DEG[i] + ((i/2) * d_laser_pair_drift_deg);
        if(true_azimuth_deg > 359.99){
            true_azimuth_deg -= 360;
        } else if (true_azimuth_deg < 0){
            true_azimuth_deg += 360;
        }
        double true_azimth_rad = deg_to_rad(true_azimuth_deg);

        uint16_t curr_range_4mm = rev_bytes_to_uint16(&data_bytes[curr_offset]);
        //VLP-32c reports range in increments of 4mm
        double curr_range_m = (static_cast<double>(curr_range_4mm) * 4) / 1000;
        curr_offset+=2;

        x = curr_range_m * cos(d_elevation_angles_rad[i]) * sin(true_azimth_rad);
        y = curr_range_m * cos(d_elevation_angles_rad[i]) * cos(true_azimth_rad);
        z = curr_range_m * sin(d_elevation_angles_rad[i]);
        
        // points[num_points+block_offset].x = x;
        // points[num_points+block_offset].y = y;
        // points[num_points+block_offset].z = z;
        // points[num_points+block_offset].reflectivity = data_bytes[curr_offset];

        points[4*num_points+block_offset] = static_cast<float>(x);
        points[4*num_points+block_offset+1] = static_cast<float>(y);
        points[4*num_points+block_offset+2] = static_cast<float>(z);
        points[4*num_points+block_offset+3] = static_cast<float>(data_bytes[curr_offset]);


        //d_logfile << data_bytes[curr_offset] << "," << i << ","  << curr_azimuth_rad+d_azimuth_offsets_rad[i] << "," << curr_range_m << ","
        //<< d_elevation_angles_rad[i] << "," << x << "," << y << "," << z << std::endl;
        std::fprintf(d_logfile, "%3.2f, %3d, %2d, %0.2f, %0.3f, %0.3f, %0.6f, %0.6f, %0.6f\n", 
        d_curr_dps / 360.0 * 60.0, data_bytes[curr_offset], i, curr_azimuth_deg+ ((i/2) * d_laser_pair_drift_deg), curr_range_m,ELEVATION_ANGLES_DEG[i], x, y, z);

        num_points+=1;
        curr_offset += 1;

    }
    return num_points;
}

int Velo_to_XYZ_impl::work(int noutput_items,
                           gr_vector_int& ninput_items,
                           gr_vector_const_void_star& input_items,
                           gr_vector_void_star& output_items)
{
    auto in = static_cast<const input_type*>(input_items[0]);
    auto out = static_cast<output_type*>(output_items[0]);

    //extract xyz
    int check=0;
    for(int block=0; block < 12; block++){
        auto curr_check = process_block(reinterpret_cast<const uint8_t*>(&in[block*100]), d_float_points, block);
        check += curr_check;
    }

    if(check != d_points_per_packet){
        std::cout << "Wrong number of points: " << check << std::endl;
    }

    // if(!d_printed_frame){
    //     d_printed_frame=true;
    //     for(int i=0; i<12*32; i++){
    //         std::cout << d_points[i].x << "," << d_points[i].y << "," << d_points[i].z << std::endl;  
    //     }
    // }
    std::memcpy(out, d_float_points.data(), check*sizeof(float)*4);

    //consume_each(1206);
    // Tell runtime system how many output items we produced.
    return check;
    //return noutput_items;
}

} /* namespace lidarModule */
} /* namespace gr */
