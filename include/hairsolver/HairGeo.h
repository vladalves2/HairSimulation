#pragma once

#include <vector>
#include <Eigen/Core>

class HairVertex {
public:
	Eigen::Vector3f pos;
	float t;
	unsigned int id;
	HairVertex();
};

class HairSegment {
public:
	HairVertex a, b;
	HairSegment();
};

class HairGeo {
public:
	unsigned int pointIter, strandIter;
	std::vector<unsigned int> offsets;
	std::vector<Eigen::Vector3f> points;

	HairGeo();
	HairGeo(HairGeo&&o) noexcept;

	HairGeo & operator= (HairGeo&&o);
	HairGeo & operator= (const std::vector<Eigen::Vector3f> &src);
	void operator<<(const Eigen::Vector3f &p);
	void operator>>(HairSegment &seg);
	void operator>>(HairVertex &vtx);
	void operator>>(Eigen::Vector3f &p);

	void resetIter();
	void resize(std::vector<unsigned int> &offs);

	unsigned int numSegments() const;
	unsigned int numPoints() const;
};