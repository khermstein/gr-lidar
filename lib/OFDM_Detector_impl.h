/* -*- c++ -*- */
/*
 * Copyright 2026 Kevin Hermstein.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_LIDARMODULE_OFDM_DETECTOR_IMPL_H
#define INCLUDED_LIDARMODULE_OFDM_DETECTOR_IMPL_H

#include <numeric>
#include <gnuradio/lidarModule/OFDM_Detector.h>
#include <Eigen/Dense>
#include <pmt/pmt.h>

namespace gr {
namespace lidarModule {

class OFDM_Detector_impl : public OFDM_Detector
{
private:
    
    int d_frame_length;
    int d_fft_len;
    int d_cp_len;
    float d_samp_rate;
    int d_chunk_size=512;
    float d_thresh;
    float d_Tsymb;
    
    
    int d_L;
    gr_complex d_p;
    float d_r;
    float d_max_r=0;
    std::vector<float> d_M;
    int d_m_iter = 0;
    int d_m_prev = 0;
    int d_m_cp_offset_iter = 0;
    std::vector<float> d_M_smooth;
    float d_prev_M_smooth;
    int d_m_smooth_iter = 0;
    bool d_initial = false;
    const float d_min_power;
    const float d_min_p;

    bool d_peak_flag = false;
    float d_curr_peak;
    bool d_keep_peak = false;
    int d_curr_keep_peak_duration=0;
    int d_keep_peak_duration;
    gr_complex d_first_out_sample;
    bool d_can_detect = true;

    float d_phase_offset;
    float d_freq_offset_hz;
    float d_phi_inc;
    float d_phi=0;
    gr_complex d_correction;
    float d_normalized_offset;
    gr_complex d_offset_corrector;

    bool d_write_iq_out=false;
    int d_num_iq_written=0;
    int d_num_pkts_detected=0;
    //std::vector<float> d_window;

    enum class State {
        STARTUP,
        UNSYNCED,
        SYNCED
    };
    State d_curr_state;

    void update_m_iter();
    void update_cfo_corrector();

public:
    OFDM_Detector_impl(int frame_len, int N, int cplen, float samp_rate, float thresh, float min_r, float min_p);
    ~OFDM_Detector_impl();

    // Where all the action really happens
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);
};

} // namespace lidarModule
} // namespace gr

#endif /* INCLUDED_LIDARMODULE_OFDM_DETECTOR_IMPL_H */
