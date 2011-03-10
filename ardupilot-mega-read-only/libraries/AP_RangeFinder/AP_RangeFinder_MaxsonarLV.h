#ifndef AP_RangeFinder_MaxsonarLV_H
#define AP_RangeFinder_MaxsonarLV_H

#include "RangeFinder.h"

#define AP_RANGEFINDER_MAXSONARLV_MIN_DISTANCE 15
#define AP_RANGEFINDER_MAXSONARLV_MAX_DISTANCE 645

class AP_RangeFinder_MaxsonarLV : public RangeFinder
{	
  public:
    AP_RangeFinder_MaxsonarLV();
	int convert_raw_to_distance(int raw_value) { return raw_value * 2.54; }   // read value from analog port and return distance in cm
};
#endif
