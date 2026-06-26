/* -*- c++ -*- */
/*
 * Copyright 2026 Kevin Hermstein
 *
 */

#ifndef LIDAR_HEADERTYPES_H_
#define LIDAR_HEADERTYPES_H_

/* Make value large to avoid overlap with existing
 * types included by gr-network. 
 * This head actually includes the Ethernet and IP
 * headers but is necessary for proper interpretation
 * of the packets at the receiver.
 */
constexpr int HEADERTYPE_LIDAR = 100;

#endif /* LIDAR_HEADERTYPES_H_ */
