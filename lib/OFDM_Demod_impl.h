/* -*- c++ -*- */
/*
 * Copyright 2026 Kevin Hermstein.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_LIDARMODULE_OFDM_DEMOD_IMPL_H
#define INCLUDED_LIDARMODULE_OFDM_DEMOD_IMPL_H

#include <gnuradio/lidarModule/OFDM_Demod.h>
#include <gnuradio/lidarModule/ofdm_utils.h>
#include <fftw3.h>
#include <Eigen/Dense>
#include <tuple>
#include <iomanip>
#include <chrono>

namespace gr {
namespace lidarModule {

class OFDM_Demod_impl : public OFDM_Demod
{
private:
    float d_samp_rate;
    int d_q;
    int d_fft_len;
    int d_cp_len;
    const std::vector<int> d_data_carriers;
    const std::vector<int> d_pilot_carriers;
    int d_seed;
    bool d_debug;
    const std::string& d_len_tag_key;
    int d_payload_len;
    int d_num_data_pkts;

    int d_num_data_carriers;
    int d_num_pilot_carriers;
    int d_num_occupied_carriers;
    std::vector<int> d_occupied_carriers;
    std::vector<gr_complex> d_SSS_fft;
    std::vector<std::tuple<int, int>> d_carrier_runs;
    Eigen::VectorXcf d_CSI;
    std::vector<gr_complex> d_CSI_simple;
    std::vector<gr_complex> d_pilot_symbols;

    std::vector<gr::tag_t> d_tags;

    std::chrono::microseconds d_total_time=std::chrono::microseconds::zero();
    int d_pkt_count=0;

    //For taking the fft
    fftwf_plan d_fft_plan;
    gr_complex *d_fft_in;
    gr_complex *d_fft_out;

    std::vector<std::tuple<int, int>> get_consecutive_carriers(std::vector<int>& d_occupied_carriers);
    unsigned char qpsk_demod(gr_complex sample);

protected:
    int calculate_output_stream_length(const gr_vector_int& ninput_items);

public:
    OFDM_Demod_impl(float samp_rate, int q, int fft_len, int cp_len, 
                    const std::vector<int>& data_carriers,
                    const std::vector<int>& pilot_carriers,
                    int seed, bool debug,
                    const std::string& len_tag_key,
                    int payload_len,
                    int num_data_pkts);
    ~OFDM_Demod_impl();

    // Where all the action really happens
    int work(int noutput_items,
             gr_vector_int& ninput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items);
};

} // namespace lidarModule
} // namespace gr

#endif /* INCLUDED_LIDARMODULE_OFDM_DEMOD_IMPL_H */
