/* -*- c++ -*- */
/*
 * Copyright 2026 Kevin Hermstein.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_LIDARMODULE_VELO_TO_XYZ_H
#define INCLUDED_LIDARMODULE_VELO_TO_XYZ_H

#include <gnuradio/lidarModule/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr {
namespace lidarModule {

/*!
 * \brief <+description of block+>
 * \ingroup lidarModule
 *
 */
class LIDARMODULE_API Velo_to_XYZ : virtual public gr::tagged_stream_block
{
public:
    typedef std::shared_ptr<Velo_to_XYZ> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of lidarModule::Velo_to_XYZ.
     *
     * To avoid accidental use of raw pointers, lidarModule::Velo_to_XYZ's
     * constructor is in a private implementation
     * class. lidarModule::Velo_to_XYZ::make is the public interface for
     * creating new instances.
     */
    static sptr make(int rpm, bool single, const std::string& len_tag_key);
};

} // namespace lidarModule
} // namespace gr

#endif /* INCLUDED_LIDARMODULE_VELO_TO_XYZ_H */
