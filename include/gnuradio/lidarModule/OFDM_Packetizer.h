/* -*- c++ -*- */
/*
 * Copyright 2026 KevinHermstein.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_LIDARMODULE_OFDM_PACKETIZER_H
#define INCLUDED_LIDARMODULE_OFDM_PACKETIZER_H

#include <gnuradio/lidarModule/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr {
namespace lidarModule {

/*!
 * \brief <+description of block+>
 * \ingroup lidarModule
 *
 */
class LIDARMODULE_API OFDM_Packetizer : virtual public gr::tagged_stream_block
{
public:
    typedef std::shared_ptr<OFDM_Packetizer> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of lidarModule::OFDM_Packetizer.
     *
     * To avoid accidental use of raw pointers, lidarModule::OFDM_Packetizer's
     * constructor is in a private implementation
     * class. lidarModule::OFDM_Packetizer::make is the public interface for
     * creating new instances.
     */
    static sptr make(int q, int fft_len, int cp_len, 
                    const std::vector<int>& data_carriers,
                    const std::vector<int>& pilot_carriers,
                    int seed, bool debug,
                    const std::string& len_tag_key);
};

} // namespace lidarModule
} // namespace gr

#endif /* INCLUDED_LIDARMODULE_OFDM_PACKETIZER_H */
