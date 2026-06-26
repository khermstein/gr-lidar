/* -*- c++ -*- */
/*
 * Copyright 2026 KevinHermstein.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_LIDARMODULE_OFDM_PACKETIZER_IMPL_H
#define INCLUDED_LIDARMODULE_OFDM_PACKETIZER_IMPL_H

#include <gnuradio/lidarModule/OFDM_Packetizer.h>
#include <gnuradio/lidarModule/ofdm_utils.h>
#include <algorithm>
#include <iterator>
#include <fftw3.h>
#include <cmath>
#include <iomanip>
#include <chrono>

namespace gr {
namespace lidarModule {

class OFDM_Packetizer_impl : public OFDM_Packetizer
{
private:
    bool d_debug;

    int d_q;
    int d_seed;

    int d_fft_len;
    float d_normalizer;
    int d_cp_len;
    int d_pkt_length;

    std::vector<int> d_data_carriers;
    int d_num_data_carriers;
    std::vector<int> d_pilot_carriers;
    int d_num_pilot_carriers;
    std::vector<gr_complex> d_pkt_with_pilots;
    std::vector<int> d_occupied_carriers;
    std::vector<int> d_sync_carriers;
    int d_num_sync_carriers;
    std::vector<gr_complex> d_PSS;
    std::vector<gr_complex> d_SSS;

    const std::string& d_len_tag_key;

    int d_pkt_count=0;
    std::chrono::microseconds d_total_time=std::chrono::microseconds::zero();

    std::vector<int> gen_sync_carriers(const std::vector<int>& occupied_carriers);
    //void allocate_onto_carriers(std::vector<gr_complex>& packet, const gr_complex* symbols, int num_symbols, const std::vector<int>& carriers);
    //void allocate_onto_carriers(gr_complex* packet, const gr_complex* symbols, int num_symbols, const std::vector<int>& carriers);
    void cp_ifft(const std::vector<gr_complex> pkt_fft, std::vector<gr_complex>& ofdm_pkt, bool pre_copy=false);
    void map_bytes_to_qpsk(const uint8_t* bytes, int num_bytes, std::vector<gr_complex>& data_symbols);

    //For taking the ifft
    fftwf_plan d_ifft_plan;
    gr_complex *d_ifft_in;
    gr_complex *d_ifft_out;

protected:
    int calculate_output_stream_length(const gr_vector_int& ninput_items);

public:
    OFDM_Packetizer_impl(int q, int fft_len, int cp_len, 
                    const std::vector<int>& data_carriers,
                    const std::vector<int>& pilot_carriers,
                    int seed, bool debug, 
                    const std::string& len_tag_key);
    ~OFDM_Packetizer_impl();

    // Where all the action really happens
    int work(int noutput_items,
             gr_vector_int &ninput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items);
};

} // namespace lidarModule
} // namespace gr

#endif /* INCLUDED_LIDARMODULE_OFDM_PACKETIZER_IMPL_H */