/* -*- c++ -*- */
/*
 * Copyright 2026 Kevin Hermstein.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_LIDARMODULE_OFDM_DEMOD_H
#define INCLUDED_LIDARMODULE_OFDM_DEMOD_H

#include <gnuradio/lidarModule/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr {
namespace lidarModule {

/*!
 * \brief <+description of block+>
 * \ingroup lidarModule
 *
 */
class LIDARMODULE_API OFDM_Demod : virtual public gr::tagged_stream_block
{
public:
    typedef std::shared_ptr<OFDM_Demod> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of lidarModule::OFDM_Demod.
     *
     * To avoid accidental use of raw pointers, lidarModule::OFDM_Demod's
     * constructor is in a private implementation
     * class. lidarModule::OFDM_Demod::make is the public interface for
     * creating new instances.
     */
    static sptr make(float samp_rate, int q, int N, int cp_len, 
                    const std::vector<int>& data_carriers,
                    const std::vector<int>& pilot_carriers,
                    int seed, bool debug,
                    const std::string& len_tag_key,
                    int payload_len,
                    int num_data_pkts);
};

} // namespace lidarModule
} // namespace gr

#endif /* INCLUDED_LIDARMODULE_OFDM_DEMOD_H */
