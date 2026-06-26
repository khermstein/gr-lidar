/* -*- c++ -*- */
/*
 * Copyright 2026 Kevin Hermstein.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "OFDM_Demod_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace lidarModule {


using input_type = gr_complex;
using output_type = unsigned char; //char for byte

OFDM_Demod::sptr OFDM_Demod::make(float samp_rate, int q, int N, int cp_len, 
                    const std::vector<int>& data_carriers,
                    const std::vector<int>& pilot_carriers,
                    int seed, bool debug,
                    const std::string& len_tag_key,
                    int payload_len,
                    int num_data_pkts)
{
    return gnuradio::make_block_sptr<OFDM_Demod_impl>( samp_rate, q, N, cp_len, 
                                                       data_carriers,
                                                       pilot_carriers,
                                                       seed, debug,
                                                       len_tag_key,
                                                    payload_len,
                                                    num_data_pkts);
}


/*
 * The private constructor
 */
OFDM_Demod_impl::OFDM_Demod_impl(float samp_rate, int q, int N, int cp_len, 
                    const std::vector<int>& data_carriers,
                    const std::vector<int>& pilot_carriers,
                    int seed, bool debug,
                    const std::string& len_tag_key,
                    int payload_len,
                    int num_data_pkts)
    : gr::tagged_stream_block(
          "OFDM_Demod",
          gr::io_signature::make(
              1 /* min inputs */, 1 /* max inputs */, sizeof(input_type)),
          gr::io_signature::make(
              2 /* min outputs */, 2 /*max outputs */, {sizeof(output_type), sizeof(gr_complex)}),
          len_tag_key),
           d_samp_rate(samp_rate),
           d_q(q),
           d_fft_len(N),
           d_cp_len(cp_len),
           d_data_carriers(data_carriers),
           d_pilot_carriers(pilot_carriers),
           d_seed(seed),
           d_debug(debug),
           d_len_tag_key(len_tag_key),
           d_payload_len(payload_len),
           d_num_data_pkts(num_data_pkts)
{
    set_tag_propagation_policy(gr::block::TPP_DONT);

    d_num_data_carriers = d_data_carriers.size();
    d_num_pilot_carriers = d_pilot_carriers.size();
    d_num_occupied_carriers = d_num_data_carriers + d_num_pilot_carriers;
    d_occupied_carriers.resize(d_num_data_carriers+d_num_pilot_carriers);
    std::merge(data_carriers.begin(), data_carriers.end(), pilot_carriers.begin(), pilot_carriers.end(), d_occupied_carriers.begin());
    d_carrier_runs = get_consecutive_carriers(d_occupied_carriers);
    if(debug){
        std::cout << "Carrier runs: " << std::endl;
        for(auto segment : d_carrier_runs){
            std::cout << "(" << std::get<0>(segment) << ", " << std::get<1>(segment) << ")" << std::endl;
        }
    }
    // 1. Allocate memory (FFTW likes specific alignment for speed)
    d_fft_in = (gr_complex*) fftwf_malloc(sizeof(fftwf_complex) * d_fft_len);
    d_fft_out = (gr_complex*) fftwf_malloc(sizeof(fftwf_complex) * d_fft_len);

    // 2. Create the plan. 
    d_fft_plan = fftwf_plan_dft_1d(d_fft_len, 
                                reinterpret_cast<fftwf_complex*>(d_fft_in), 
                                reinterpret_cast<fftwf_complex*>(d_fft_out), 
                                FFTW_FORWARD, FFTW_MEASURE);
    
    //generate the SSS
    d_SSS_fft.resize(d_fft_len);
    std::cout << d_SSS_fft.size() << std::endl;
    auto zc_seq = gen_zc_symbols(d_q, d_occupied_carriers.size());
    allocate_onto_carriers(d_SSS_fft, zc_seq.data(), zc_seq.size(), d_occupied_carriers, d_fft_len);
    //Because we use same root as PSS, take the conjugate to avoid synchronization issues
    conjugate(d_SSS_fft);
    d_CSI.resize(d_fft_len);
    d_CSI.setZero();
    //d_CSI_simple.resize(d_fft_len);
    d_pilot_symbols = gen_pilot_symbols(d_seed, d_num_pilot_carriers);
    //writeComplexBinary("comb_pilots.bin", pilot_symbols);
}

/*
 * Our virtual destructor.
 */
OFDM_Demod_impl::~OFDM_Demod_impl() {
    //Cleanup the fft plan
    fftwf_destroy_plan(d_fft_plan);
    fftwf_free(d_fft_in);
    fftwf_free(d_fft_out);

    auto avg = d_total_time / d_pkt_count;
    std::cout << "[Demod] " << avg.count() << " us over " << d_pkt_count << " pkts" << std::endl;
}

int OFDM_Demod_impl::calculate_output_stream_length(const gr_vector_int& ninput_items)
{
    int noutput_items = d_payload_len;
    return noutput_items;
}

std::vector<std::tuple<int, int>> OFDM_Demod_impl::get_consecutive_carriers(std::vector<int>& occupied_carriers) {
    std::vector<std::tuple<int, int>> runs;
    int start=occupied_carriers[0], count=1;
    if(start < 0) start = start + d_fft_len;
    for(unsigned int i = 1; i < occupied_carriers.size(); i++){
        if (occupied_carriers[i] == occupied_carriers[i-1] + 1) {
            count++;
        } else {
            runs.push_back(std::tuple<int,int>(start, count));
            start = occupied_carriers[i];
            if(start < 0) start = start + d_fft_len;
            count = 1;
        }
    }
    runs.push_back(std::tuple<int,int>(start, count));\
    //Sort the runs so d_CSI gets set with positive frequency convention
    std::sort(runs.begin(), runs.end(), [](const auto& t1, const auto& t2) {
        return std::get<0>(t1) < std::get<0>(t2); // Sort based on the string element
    });
    return runs;
}

unsigned char OFDM_Demod_impl::qpsk_demod(gr_complex sample) {
    unsigned char byte = 0;
    if(std::real(sample) < 0){
        byte |= 1;
    }
    if(std::imag(sample) < 0){
        byte |= (1 << 1);
    }
    return byte;
}

int OFDM_Demod_impl::work(int noutput_items,
                          gr_vector_int& ninput_items,
                          gr_vector_const_void_star& input_items,
                          gr_vector_void_star& output_items)
{
    
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<unsigned char> output_bytes(1206);
    //input frame is only missing the cp of the PSS (7488 - 64 = 7424 samples)
    //Need to offset by 512 to get to the SSS (+ its cp)
    //Then the remaining samples are the 11 data packets
    auto in = static_cast<const input_type*>(input_items[0]);
    auto out = static_cast<output_type*>(output_items[0]);
    auto out2 = static_cast<gr_complex*>(output_items[1]);
    std::vector<float> phases(d_num_pilot_carriers); 
    //auto out2 = static_cast<output_type*>(output_items[1]);
    //std::vector<gr_complex> rx_signal{ in, in + ninput_items[0] };
    int input_offset = d_fft_len + d_cp_len - 32; //skip the remaining PSS and the cp of the SSS, subtract a few samples to avoid going past cp
    std::memcpy(d_fft_in, &in[input_offset], d_fft_len*sizeof(gr_complex));
    //std::memcpy(d_fft_in, &corrected_in[input_offset], d_fft_len*sizeof(gr_complex));
    input_offset += (d_fft_len+d_cp_len); // advance to next packet
    //Recover the SSS symbols
    fftwf_execute(d_fft_plan);
    
    Eigen::Map<const Eigen::VectorXcf> eig_fft_out(d_fft_out, d_fft_len);
    Eigen::Map<const Eigen::VectorXcf> tx_SSS_fft(d_SSS_fft.data(), d_fft_len);
    int offset = 0;
    for(auto run : d_carrier_runs){
        auto rx_slice = eig_fft_out.segment(std::get<0>(run), std::get<1>(run));
        auto tx_slice = tx_SSS_fft.segment(std::get<0>(run), std::get<1>(run));
        d_CSI.segment(std::get<0>(run), std::get<1>(run)) = rx_slice.cwiseQuotient(tx_slice);
        //offset += std::get<1>(run);
    }

    int o=0;
    int o2=0;
    int byte_counter=0;
    int symbol_counter=0;
    unsigned char curr_byte = 0;
    for(int k=0; k<d_num_data_pkts; k++){
        std::memcpy(d_fft_in, &in[input_offset], d_fft_len*sizeof(gr_complex));
        //std::memcpy(d_fft_in, &corrected_in[input_offset], d_fft_len*sizeof(gr_complex));
        input_offset += (d_fft_len+d_cp_len);

        fftwf_execute(d_fft_plan);
        Eigen::VectorXcf test;
        test.resize(d_fft_len);
        offset = 0;
        //CSI corrrection
        for(auto run : d_carrier_runs){
            auto csi_slice = d_CSI.segment(std::get<0>(run), std::get<1>(run));
            auto rx_slice = eig_fft_out.segment(std::get<0>(run), std::get<1>(run));
            test.segment(std::get<0>(run), std::get<1>(run)) = rx_slice.cwiseQuotient(csi_slice);
            offset += std::get<1>(run);
        }

        //Phase and DC Correction
        //TODO - Optimize with volk and eigen
        float mean_phase=0;
        gr_complex mean_vector=gr_complex(0,0);
        for(int i = 0; i < d_num_pilot_carriers; i++){
            int carrier_idx = (d_pilot_carriers[i] >= 0) ? d_pilot_carriers[i] : d_pilot_carriers[i] + d_fft_len;
            mean_phase += std::arg(test[carrier_idx] / d_pilot_symbols[i]);
        }
        mean_phase /= d_num_pilot_carriers;
        gr_complex phase_corrector = std::exp(gr_complex(0, -1.0*mean_phase));
        for(int i = 0; i < d_num_pilot_carriers; i++){
            int carrier_idx = (d_pilot_carriers[i] >= 0) ? d_pilot_carriers[i] : d_pilot_carriers[i] + d_fft_len;
            mean_vector += test[carrier_idx] * phase_corrector;
        }
        mean_vector /= d_num_pilot_carriers;

        //write symbols out for debug - symbol demod
        for(int i = 0; i < d_num_data_carriers; i++){
            int carrier_idx = (d_data_carriers[i] >= 0) ? d_data_carriers[i] : d_data_carriers[i] + d_fft_len;
            gr_complex symbol = (test[carrier_idx] * phase_corrector) - mean_vector;
            

            if(symbol_counter%4 == 0){
                if(k > 0 || i>0){
                    output_bytes[byte_counter++] = curr_byte;
                    out[o++] = curr_byte;
                    curr_byte = 0;
                    if(byte_counter >= d_payload_len){
                        break;
                    }
                }
                //out[o++] = test[carrier_idx];
                curr_byte |= (qpsk_demod(symbol) << 6);
            } else {
                //out[o++] = test[carrier_idx];
                curr_byte |= (qpsk_demod(symbol) << (6-(2*(symbol_counter%4))));
            }
            symbol_counter++;
            out2[o2++] = symbol;
        }
        // for(int i = 0; i < d_num_pilot_carriers; i++){
        //     int carrier_idx = (d_pilot_carriers[i] >= 0) ? d_pilot_carriers[i] : d_pilot_carriers[i] + d_fft_len;
        //     out2[o2++] = test[carrier_idx];
        // }
    }
    // std::cout << "Bytes: " << byte_counter << std::endl;
    // for(int i = 0; i < 1206; i++){
    //     unsigned char byte = output_bytes[i];
    //     std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned int>(byte) << " ";
    //     if(i % 8 == 7){
    //         std::cout << std::endl;
    //     }
    // }
    d_pkt_count++;
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    d_total_time += duration;
    //std::memcpy(&out[offset], test.data() + offset, std::get<1>(run)*sizeof(gr_complex));
    //std::vector<gr_complex> output_vec(d_CSI.data(), d_CSI.data() + d_CSI.size());
    //writeBinary("bit_check.bin", output_bytes, true);
    // Tell runtime system how many output items we produced.
    //std::memcpy(out, output_bytes.data(), d_payload_len*sizeof(output_type));
    //return noutput_items;
    if(o!=d_payload_len) std::cout << o << std::endl;
    produce(0, o);
    produce(1, o2);
    //return o;
    return 0;
}

} /* namespace lidarModule */
} /* namespace gr */
