/* -*- c++ -*- */
/*
 * Copyright 2026 KevinHermstein.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "OFDM_Packetizer_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace lidarModule {

using input_type = char;
using output_type = gr_complex;
OFDM_Packetizer::sptr OFDM_Packetizer::make(int q, int fft_len, int cp_len, 
                    const std::vector<int>& data_carriers,
                    const std::vector<int>& pilot_carriers,
                    int seed, bool debug,
                    const std::string& len_tag_key)
{
    return gnuradio::make_block_sptr<OFDM_Packetizer_impl>(q, fft_len, cp_len,
                                                        data_carriers, pilot_carriers,
                                                        seed, debug,
                                                        len_tag_key);
}


/*
 * The private constructor
 */
OFDM_Packetizer_impl::OFDM_Packetizer_impl(int q, int fft_len, int cp_len, 
                    const std::vector<int>& data_carriers,
                    const std::vector<int>& pilot_carriers,
                    int seed, bool debug,
                    const std::string& len_tag_key)
    : gr::tagged_stream_block(
          "OFDM_Packetizer",
          gr::io_signature::make(
              1 /* min inputs */, 1 /* max inputs */, sizeof(input_type)),
          gr::io_signature::make(
              1 /* min outputs */, 1 /*max outputs */, sizeof(output_type)),
            len_tag_key),
          d_debug(debug),
          d_q(q),
          d_seed(seed),
          d_fft_len(fft_len),
          d_cp_len(cp_len),
          d_data_carriers(data_carriers),
          d_pilot_carriers(pilot_carriers),
          d_len_tag_key(len_tag_key)
{
    d_pkt_length = d_cp_len + d_fft_len;
    set_output_multiple(d_pkt_length);
    d_normalizer = 1.0 / d_fft_len;
    //std::cout << d_normalizer << std::endl;
    //Construct the fftw plan - from Gemini
    // 1. Allocate memory (FFTW likes specific alignment for speed)
    d_ifft_in = (gr_complex*) fftwf_malloc(sizeof(fftwf_complex) * d_fft_len);
    d_ifft_out = (gr_complex*) fftwf_malloc(sizeof(fftwf_complex) * d_fft_len);

    // 2. Create the plan. 
    // FFTW_BACKWARD = IFFT. FFTW_MEASURE = Find the fastest way - longer intialization but faster performance.
    d_ifft_plan = fftwf_plan_dft_1d(d_fft_len, 
                                reinterpret_cast<fftwf_complex*>(d_ifft_in), 
                                reinterpret_cast<fftwf_complex*>(d_ifft_out), 
                                FFTW_BACKWARD, FFTW_MEASURE);
    if(d_debug){
        std::cout << "FFTW Plan Created" << std::endl;
    }
    d_num_data_carriers = data_carriers.size();
    d_num_pilot_carriers = pilot_carriers.size();
    //Precompute the pilot frames
    d_occupied_carriers.resize(d_num_data_carriers+d_num_pilot_carriers);
    std::merge(data_carriers.begin(), data_carriers.end(), pilot_carriers.begin(), pilot_carriers.end(), d_occupied_carriers.begin());
    if(d_debug){
        std::cout << "Num data carriers: " << d_num_data_carriers << std::endl;
        std::cout << "Num pilot carriers: " << d_num_pilot_carriers << std::endl;
        std::cout << "Num occupied carriers: " << d_occupied_carriers.size() << std::endl;
    }
    //get sync carriers
    d_sync_carriers = gen_sync_carriers(d_occupied_carriers);
    if(d_debug) std::cout << "Num sync carriers: " << d_sync_carriers.size() << std::endl;
    //generate the PSS
    auto zc_seq = gen_zc_symbols(d_q, d_sync_carriers.size());
    //allocate zc to carriers
    std::vector<gr_complex> pilot_pre_ifft(d_fft_len);
    allocate_onto_carriers(pilot_pre_ifft, zc_seq.data(), zc_seq.size(), d_sync_carriers, d_fft_len);
    if(d_debug){
        std::cout << "ZC sequence allocated to carriers" << std::endl;
    }
    //ifft to make zc time signal and add cp
    d_PSS.resize(d_fft_len+d_cp_len);
    cp_ifft(pilot_pre_ifft, d_PSS);

    //generate the SSS
    zc_seq = gen_zc_symbols(d_q, d_occupied_carriers.size());
    allocate_onto_carriers(pilot_pre_ifft, zc_seq.data(), zc_seq.size(), d_occupied_carriers, d_fft_len);
    //Because we use same root as PSS, take the conjugate to avoid synchronization issues
    conjugate(pilot_pre_ifft);
    d_SSS.resize(d_fft_len+d_cp_len);
    cp_ifft(pilot_pre_ifft, d_SSS);

    //generate the symbols for pilot subcarriers
    auto pilot_symbols = gen_pilot_symbols(d_seed, d_num_pilot_carriers);
    if(d_debug){
        std::cout << "Pilot symbols generated with seed: " << d_seed << std::endl;
    }
    //allocate onto subcarriers
    d_pkt_with_pilots.resize(d_fft_len);
    allocate_onto_carriers(d_pkt_with_pilots, pilot_symbols.data(), pilot_symbols.size(), d_pilot_carriers, d_fft_len);
    if(d_debug){
        std::cout << "Template packet with pilots created" << std::endl;
    }
    // if(d_debug){
    //     std::cout << "Pilots:" << std::endl;
    //     for(auto carrier : d_pilot_carriers){
    //         int carrier_idx = (carrier < 0) ? carrier + d_fft_len : carrier;
    //         std::cout << carrier << ": " << d_frame_with_pilots[carrier_idx] << std::endl;
    //     }
    // }
}
/*
 * Our virtual destructor.
 */
OFDM_Packetizer_impl::~OFDM_Packetizer_impl() {
    //Cleanup the fft plan
    fftwf_destroy_plan(d_ifft_plan);
    fftwf_free(d_ifft_in);
    fftwf_free(d_ifft_out);
    //avg time to write packet
    auto avg = d_total_time / d_pkt_count;
    std::cout << "[TX] Avg time: " << avg.count() << "  over " << d_pkt_count << " packets." << std::endl;
}

int OFDM_Packetizer_impl::calculate_output_stream_length(const gr_vector_int& ninput_items)
{
    //Output items will be length of 1 PSS + 1 SSS plus ceiling of 
    //Number of input symbols (bytes/M order) divided by data carriers per ofdm
    //Times the length of OFDM packet
    //noutput = 2*(cp_len + pkt_len) + ceil(input_bytes/M/num_data_carrier) * (cp_len + pkt_len)
    int full_ofdm_pkt_len = d_cp_len + d_fft_len;
    int num_ofdm_pkt = 2 + std::ceil(static_cast<float>(ninput_items[0] * 4) / d_num_data_carriers);
    int noutput_items = full_ofdm_pkt_len * num_ofdm_pkt;
    //std::cout << "Calc output: " << noutput_items << std::endl;
    //std::cout << "num pkts: " << num_ofdm_pkt << std::endl;
    return noutput_items;
}


/*
 * Generate the Zadoff-Chu PSS carriers
 */
std::vector<int> OFDM_Packetizer_impl::gen_sync_carriers(const std::vector<int>& occupied_carriers){
    std::vector<int> sync_carriers;
    for(unsigned int i=0; i<occupied_carriers.size(); i++){
        if(occupied_carriers[i] % 2 == 0 && occupied_carriers[i] !=0){
            sync_carriers.push_back(occupied_carriers[i]);
        }
    }
    if(sync_carriers.size() % 2 == 0){
        sync_carriers.pop_back();
    }
    return sync_carriers;
}

/*
 * Take a freqeuncy domain frame and calculate the ifft. Add the cyclic prefix
 * pre_copy flag indicates if the fft of the pkt is already in the fft input buffer
 * this allows direct copying of data into the input buffer to avoid unnecessary 
 * vector instantiations and copying
 */
void OFDM_Packetizer_impl::cp_ifft(const std::vector<gr_complex> pkt_fft, std::vector<gr_complex>& ofdm_pkt, bool pre_copy){
    //Take ifft - from Gemini
    // 1. Copy your mapped subcarriers into the FFT input buffer
    //if(d_debug) std::cout << "prepping ifft...";
    if(!pre_copy){
        std::memcpy(d_ifft_in, pkt_fft.data(), sizeof(gr_complex) * d_fft_len);
    }
    // 2. Run the actual math
    //if(d_debug) std::cout << "taking ifft...";
    fftwf_execute(d_ifft_plan);
    //Normalize the output - fftw requires this only on ifft
    //if(d_debug) std::cout << "Copying to output frame" << std::endl;
    for(int i = 0; i < d_fft_len; i++){
        ofdm_pkt[i+d_cp_len] = d_ifft_out[i] * d_normalizer;
        if(i+d_cp_len-d_fft_len >= 0){ // assign the cp
            ofdm_pkt[i+d_cp_len-d_fft_len] = d_ifft_out[i] * d_normalizer;
        }
    }

    return;
}



//Logic for mapping the bytes to qpsk symbols - from Gemini
void OFDM_Packetizer_impl::map_bytes_to_qpsk(const uint8_t* bytes, int num_bytes, std::vector<gr_complex>& data_symbols){
    int symb_idx = 0;
    // Amplitude scaling (standard is 1/sqrt(2) so the vector length is 1)
    const float A = 0.70710678118f; 

    for (int i = 0; i < num_bytes; ++i) {
        uint8_t byte = bytes[i];
        
        // Process byte in 2-bit chunks (from most significant to least)
        for (int shift = 6; shift >= 0; shift -= 2) {
            uint8_t bits = (byte >> shift) & 0x03; // Get 2 bits

            float re, im;
            // QPSK Mapping logic (Gray Coded)
            switch(bits) {
                case 0x00: re =  A; im =  A; break; // 00
                case 0x01: re = -A; im =  A; break; // 01
                case 0x02: re =  A; im = -A; break; // 10
                case 0x03: re = -A; im = -A; break; // 11
                default:   re =  0; im =  0; break;
            }
            data_symbols[symb_idx++] = gr_complex(re, im);
        }
    }
}

int OFDM_Packetizer_impl::work(int noutput_items,
                             gr_vector_int &ninput_items,
                             gr_vector_const_void_star& input_items,
                             gr_vector_void_star& output_items)
{
    auto in = static_cast<const input_type*>(input_items[0]);
    auto out = static_cast<output_type*>(output_items[0]);
    d_pkt_count++;
    auto start = std::chrono::high_resolution_clock::now();
    // Do <+signal processing+>
    //std::cout << std::dec << "Out:" << noutput_items << std::endl;
    //std::cout << std::dec << "In: " << ninput_items[0] << std::endl;

    // for(int i=0; i<ninput_items[0]; i++){
    //     unsigned int val = static_cast<unsigned char>(in[i]);
    //     std::cout << std::hex << std::setw(2) << std::setfill('0') 
    //               << val << " ";
    //     if((i+1) % 8 == 0){
    //         std::cout << std::dec << "  ";
    //     }
    //     if((i+1) % 16 == 0){
    //         std::cout << std::dec << std::endl;
    //     }
    // }

    //4 symbols per byte (2 bits per symbol)
    int num_symbols = ninput_items[0]*4;
    int num_pkts = std::ceil(static_cast<float>(num_symbols) / d_num_data_carriers);
    std::vector<gr_complex> data_symbols(num_symbols);
    map_bytes_to_qpsk(reinterpret_cast<const uint8_t*>(in), ninput_items[0], data_symbols);
    std::vector<gr_complex> ofdm_pkt_stream(d_pkt_length);
    int remaining_symbols = num_symbols;
    std::memcpy(&out[0], d_PSS.data(), d_PSS.size()*sizeof(gr_complex));
    std::memcpy(&out[d_pkt_length], d_SSS.data(), d_SSS.size()*sizeof(gr_complex));
    //Pre the fftw in buffer by loading the pkt with just pilots
    std::memcpy(d_ifft_in, d_pkt_with_pilots.data(), d_pkt_with_pilots.size()*sizeof(gr_complex));
    //Now all we must do it keep overwriting the actual data carriers and taking the ifft
    //Do not need to initialize new pkts and copy them into the fft input buffer
    for(int i=0; i < num_pkts; i++){
        //See if we fill all data carriers. If we do not, only copy the remainder left. Note: This leaves previous data in untouched data carriers
        int num_symbols_in_pkt = (remaining_symbols > d_num_data_carriers) ? d_num_data_carriers : remaining_symbols;
        allocate_onto_carriers(d_ifft_in, data_symbols.data() + i*d_num_data_carriers, num_symbols_in_pkt, d_data_carriers, d_fft_len);
        //we pass any vector data pkt, it is unused with pre_copy is true
        cp_ifft(d_pkt_with_pilots, ofdm_pkt_stream, true);
        //Plus 2 to i for PSS and SSS
        std::memcpy(&out[(i+2)*(d_pkt_length)], ofdm_pkt_stream.data(), ofdm_pkt_stream.size()*sizeof(gr_complex));
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start); 
    if(d_debug) std::cout << "Finished frame " << d_pkt_count << ": " << duration.count() << " us" <<  std::endl;
    d_total_time += duration;
    // Tell runtime system how many output items we produced.
    return (num_pkts+2)*d_pkt_length;
}

} /* namespace lidarModule */
} /* namespace gr */
