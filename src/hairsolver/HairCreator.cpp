#include "HairCreator.h"

#include <Eigen\Core>

HairGeo HairCreator::createRadialHair(unsigned int seed, unsigned int numHairs, unsigned int numPointsPerHair, float hairLength) {
	HairGeo geo;

	if ((hairLength == 0) || (numPointsPerHair < 2) || (numHairs < 1)) return geo;

	std::vector<unsigned int> offsets; offsets.resize(numHairs + 1, 0);
	for (auto i = 1; i <= numHairs; i++) {
		offsets[i] = offsets[i - 1] + numPointsPerHair;
	}
	
	geo.resize(offsets);

	float segmentLength = hairLength / (numPointsPerHair - 1);
	srand(seed);
	for (auto strandId = 0; strandId < numHairs; strandId++) {
		Eigen::Vector3f dir ( (float)rand() / (float)RAND_MAX, (float)rand() / (float)RAND_MAX, (float)rand() / (float)RAND_MAX );
		dir -= Eigen::Vector3f(0.5f, 0.5f, 0.5f);
		dir.normalize();

		Eigen::Vector3f root = dir * 0.1f;

		for (auto pointId = 0; pointId < numPointsPerHair; pointId++) {
			geo << (root + dir * (pointId * segmentLength));
		}
	}
	
	return geo;
}