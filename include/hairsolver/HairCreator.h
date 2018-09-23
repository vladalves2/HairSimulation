#pragma once

#include "HairGeo.h"
#include <vector>

class HairCreator {
public:
	static HairGeo createRadialHair(unsigned int seed, unsigned int numHairs, unsigned int numPointsPerHair, float hairLength);

};