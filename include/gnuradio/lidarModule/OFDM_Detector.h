/* -*- c++ -*- */
/*
 * Copyright 2026 Kevin Hermstein.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_LIDARMODULE_OFDM_DETECTOR_H
#define INCLUDED_LIDARMODULE_OFDM_DETECTOR_H

#include <gnuradio/block.h>
#include <gnuradio/lidarModule/api.h>

namespace gr {
namespace lidarModule {

/*!
 * \brief <+description of block+>
 * \ingroup lidarModule
 *
 */
class LIDARMODULE_API OFDM_Detector : virtual public gr::block
{
public:
    typedef std::shared_ptr<OFDM_Detector> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of lidarModule::OFDM_Detector.
     *
     * To avoid accidental use of raw pointers, lidarModule::OFDM_Detector's
     * constructor is in a private implementation
     * class. lidarModule::OFDM_Detector::make is the public interface for
     * creating new instances.
     */
    static sptr make(int frame_len, int N, int cplen, float samp_rate, float thresh, float min_r, float min_p);
};

} // namespace lidarModule
} // namespace gr

#endif /* INCLUDED_LIDARMODULE_OFDM_DETECTOR_H */
