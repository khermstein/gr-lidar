/* -*- c++ -*- */
/*
 * Copyright 2026 Kevin Hermstein.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "OFDM_Detector_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace lidarModule {

using input_type = gr_complex;

using output_type = float;
OFDM_Detector::sptr OFDM_Detector::make(int frame_length, int N, int cplen, float samp_rate, float thresh, float min_r,  float min_p)
{
    return gnuradio::make_block_sptr<OFDM_Detector_impl>(frame_length, N, cplen, samp_rate, thresh, min_r, min_p);
}


/*
 * The private constructor
 */
OFDM_Detector_impl::OFDM_Detector_impl(int frame_length, int N, int cplen, float samp_rate, float thresh, float min_r,  float min_p)
    : gr::block("OFDM_Detector",
                gr::io_signature::make(
                    1 /* min inputs */, 1 /* max inputs */, sizeof(input_type)),
                gr::io_signature::make(
                    4 /* min outputs */, 4 /*max outputs */, {sizeof(output_type),sizeof(output_type),sizeof(gr_complex),sizeof(gr_complex)})),
      d_frame_length(frame_length),
      d_fft_len(N),
      d_cp_len(cplen),
      d_samp_rate(samp_rate),
      d_thresh(thresh),
      d_min_power(min_r),
      d_min_p(min_p)
{
    d_curr_state = State::STARTUP;
    d_L = d_fft_len / 2;
    d_M.resize(2 * d_L);
    d_M_smooth.resize(2*d_cp_len);
    d_keep_peak_duration = d_cp_len;
    d_Tsymb = static_cast<float>(d_fft_len) / d_samp_rate;

    set_tag_propagation_policy(gr::block::TPP_DONT);
}

/*
 * Our virtual destructor.
 */
OFDM_Detector_impl::~OFDM_Detector_impl() {
    std::cout << "[SC RX] Detected " << d_num_pkts_detected << " pkts" << std::endl;
}

void OFDM_Detector_impl::forecast(int noutput_items, gr_vector_int& ninput_items_required)
{

    /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
    switch (d_curr_state) {
    case State::STARTUP:
        ninput_items_required[0] = std::min(2 * d_L + noutput_items, 2 * d_L + d_chunk_size);
        break;
    case State::UNSYNCED:
        ninput_items_required[0] = std::min(2 * d_L + noutput_items, 2 * d_L + d_chunk_size);
        break;
    case State::SYNCED:
        ninput_items_required[0] = std::min(noutput_items, 12*(512+d_cp_len));
        break;
    }
}

void OFDM_Detector_impl::update_m_iter()
{
    d_m_iter = (d_m_iter + 1) % (2 * d_L);
    d_m_prev = (d_m_iter + (2*d_L) - 1) % (2*d_L);
    d_m_cp_offset_iter = (d_m_iter + (2*d_L) - d_cp_len) % (2*d_L);
}

void OFDM_Detector_impl::update_cfo_corrector()
{
    float cos_val = std::cos(d_phi);
    float sin_val = std::sin(d_phi);
    d_correction = gr_complex(cos_val, sin_val);
    d_phi += d_phi_inc;

    if (d_phi > M_PI) d_phi -= 2.0f * M_PI;
    if (d_phi < -M_PI) d_phi += 2.0f * M_PI;
}

int OFDM_Detector_impl::general_work(int noutput_items,
                                  gr_vector_int& ninput_items,
                                  gr_vector_const_void_star& input_items,
                                  gr_vector_void_star& output_items)
{
    auto in = static_cast<const input_type*>(input_items[0]);
    auto out = static_cast<output_type*>(output_items[0]);
    auto out2= static_cast<output_type*>(output_items[1]);
    auto out3= static_cast<gr_complex*>(output_items[2]);
    auto out4= static_cast<gr_complex*>(output_items[3]);

    int num_to_process = std::min(noutput_items, ninput_items[0]);
    // if (num_to_process < 2*d_L) {
    //     return 0;
    // }

    std::vector<gr_complex> rx_signal{ in, in + num_to_process };
    
    int energy_samples=0;
    int outidx = 0;
    int outidx2 = 0;
    int outidx3 = 0;
    int outidx4 = 0;
    int i = 0;
    //std::vector<gr_complex> p_hist{num_to_process};

    while(i < num_to_process - (2*d_L)){
        if(d_write_iq_out){
            out3[outidx3++] = rx_signal[i];
            out4[outidx4++] = rx_signal[i] * d_correction;
            update_cfo_corrector();
            d_num_iq_written++;
            if(d_num_iq_written==(d_frame_length-d_cp_len)){
                d_can_detect=true;
                d_write_iq_out= false;
                d_num_iq_written = 0;
                memset(d_M.data(), 0, d_M.size() * sizeof(float));
                memset(d_M_smooth.data(), 0, d_M_smooth.size() * sizeof(float));
                d_initial=false;
                energy_samples=0;
                continue;
            }
        }
        // Calculate intial P with autocorrelation
        Eigen::Map<Eigen::VectorXcf> rx_vect(rx_signal.data()+i, d_L);
        Eigen::Map<Eigen::VectorXcf> rx_vect_delay(rx_signal.data() + i + d_L, d_L);
        d_p = rx_vect.dot(rx_vect_delay);
        d_r = rx_vect_delay.cwiseAbs2().sum();
        //Always check the energy level - if it is too low set M to 0
        //Otherwise the very small denominator can cause M to increase 
        //drastically and suddenly
        if (d_r > d_min_power && std::norm(d_p) > d_min_p) {
            auto m = std::norm(d_p) / (d_r * d_r);
            if (std::isnan(m) || std::isinf(m)) {
                std::cout << m << ", " << std::norm(d_p) << ", " << d_r << std::endl;
                d_M[d_m_iter] = 0.0f;
            } else {
                d_M[d_m_iter] = m;
            }
        } else {
            d_M[d_m_iter] = 0;
        }
        i++;
        update_m_iter();
        int init_i = i;
        // Calculte each metric recursively (add next value subtract oldest value)
        for (; (i < init_i + d_chunk_size) || d_keep_peak || d_peak_flag; i++) {
            if(i >= num_to_process - (2*d_L)) break;
            
            d_p = d_p + std::conj(rx_signal[i + d_L]) * rx_signal[i + 2 * d_L] -
                    std::conj(rx_signal[i]) * rx_signal[i + d_L];
            d_r = d_r + std::norm(rx_signal[i + 2 * d_L]) - std::norm(rx_signal[i + d_L]);
            //Calculate M and place onto ring buffer. Check the energy is high enough
            if (d_r > d_min_power && std::norm(d_p) > d_min_p) {
                auto m = std::norm(d_p) / (d_r * d_r);
                if (std::isnan(m) || std::isinf(m)) {
                    std::cout << m << ", " << std::norm(d_p) << ", " << d_r << "," << rx_signal[i+2*d_L] << std::endl;
                    d_M[d_m_iter] = 0.0f;
                } else {
                    d_M[d_m_iter] = m;
                    energy_samples+=1;
                }
            } else {
                d_M[d_m_iter] = 0;
            }
            //Smooth M
            //Check if we have already initialized the smoothed M and if we have enough samples with energy
            if(!d_initial && energy_samples == d_cp_len){
                //Check if we need to wrap around the ring buffer
                if(d_m_iter < d_cp_len - 1){
                    unsigned int remaining = d_cp_len - d_m_iter;
                    auto partial = std::accumulate(d_M.begin(), d_M.begin() + d_m_iter + 1, 0.0f);
                    d_prev_M_smooth = (1 / static_cast<float>(d_cp_len)) * (std::accumulate(d_M.end() - remaining, d_M.end(), 0.0f) + partial);
                } else { //Handle going past end of rung buffer
                    d_prev_M_smooth = (1 / static_cast<float>(d_cp_len)) * std::accumulate(d_M.begin() + (d_m_iter - d_cp_len + 1), d_M.begin() + d_m_iter + 1, 0.0f);
                }
                d_M_smooth[d_m_smooth_iter] = d_prev_M_smooth;
                //std::cout << "Init M smooth: " << d_prev_M_smooth << std::endl;
                d_initial = true;
            //If we already initialized M smooth calculate next values recursively
            } else if (d_initial) {
                d_prev_M_smooth = d_prev_M_smooth + (1 / static_cast<float>(d_cp_len)) * (d_M[d_m_iter] - d_M[d_m_cp_offset_iter]);
                d_M_smooth[d_m_smooth_iter] = d_prev_M_smooth;
            }
            //Check for peak
            //If we already found a peak, wait for some samples before checking for another peak
            if(d_keep_peak && d_curr_keep_peak_duration < d_keep_peak_duration){
                d_curr_keep_peak_duration++;
            } else {
                d_curr_keep_peak_duration=0;
                d_keep_peak=false;
                //First sample above threshold indicates we are approaching a peak
                if (d_peak_flag == false && d_M_smooth[d_m_smooth_iter] > d_thresh && d_can_detect) {
                    d_curr_peak = d_M_smooth[d_m_smooth_iter];
                    d_peak_flag = true;
                    d_phase_offset = std::arg(d_p);
                    d_first_out_sample = rx_signal[i];
                //If we have already approached a peak, wait until the metric decreases for the first time
                //The max value is our peak
                } else if(d_peak_flag == true){
                    if(d_curr_peak < d_M_smooth[d_m_smooth_iter]){
                        d_curr_peak = d_M_smooth[d_m_smooth_iter];
                        d_phase_offset = std::arg(d_p);
                        d_first_out_sample = rx_signal[i];
                    } else { // Here the peak has been found
                        //add_item_tag(0, nitems_written(0) + outidx, pmt::mp("Peak"), pmt::from_float(d_curr_peak));
                        d_can_detect=false;
                        d_num_pkts_detected++;
                        add_item_tag(2, nitems_written(2) + outidx3, pmt::mp("packet_len"), pmt::from_long(d_frame_length-d_cp_len));
                        add_item_tag(3, nitems_written(3) + outidx4, pmt::mp("packet_len"), pmt::from_long(d_frame_length-d_cp_len));
                        d_write_iq_out = true;
                        d_keep_peak = true;
                        d_peak_flag = false;
                        d_curr_peak = 0;
                        d_freq_offset_hz = (d_phase_offset * d_samp_rate) / (M_PI * d_fft_len);
                        d_phi_inc = -2.0f * M_PI * (d_freq_offset_hz / d_samp_rate);
                        d_phi=0;
                        add_item_tag(2, nitems_written(2) + outidx3, pmt::mp("cfo"), pmt::from_float(d_freq_offset_hz));
                        //std::cout << "delf: " << d_freq_offset_hz << std::endl;
                        //std::cout << "phi: " << d_phase_offset << std::endl;
                        out3[outidx3++] = d_first_out_sample;
                        out4[outidx4++] = d_first_out_sample * d_correction;
                        update_cfo_corrector();
                        d_num_iq_written++;
                    }
                }
            }
            if(d_initial){
                out[outidx++] = d_M_smooth[d_m_smooth_iter];
                d_m_smooth_iter = (d_m_smooth_iter + 1) % (2*d_cp_len);
            } else {
                out[outidx++] = 0;
            }
            out2[outidx2++] = d_M[d_m_iter];
            update_m_iter();
            if(d_write_iq_out){
                //float phase = 2*M_PI*d_num_iq_written*d_freq_offset_hz;
                //gr_complex corrector = std::exp(gr_complex(0, phase));
                out3[outidx3++] = rx_signal[i];
                out4[outidx4++] = rx_signal[i] * d_correction;
                update_cfo_corrector();
                d_num_iq_written++;
                if(d_num_iq_written==(d_frame_length-d_cp_len)){
                    d_can_detect=true;
                    d_write_iq_out= false;
                    d_num_iq_written = 0;
                    //reset the ring buffers
                    memset(d_M.data(), 0, d_M.size() * sizeof(float));
                    memset(d_M_smooth.data(), 0, d_M_smooth.size() * sizeof(float));
                    d_initial=false;
                    energy_samples=0;
                    break;
                }
            }
        }
    }
    // Tell runtime system how many input items we consumed on
    // each input stream.
    consume_each(i);
    // Tell runtime system how many output items we produced.
    produce(0, outidx);
    produce(1, outidx2);
    produce(2, outidx3);
    produce(3, outidx4);
    
    return 0;
    //return outidx;
}

} /* namespace lidarModule */
} /* namespace gr */
