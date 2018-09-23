#include "HairGeo.h"

HairVertex::HairVertex() :pos(), t(), id(0) {}

HairSegment::HairSegment():a(),b(){}

HairGeo::HairGeo() : pointIter(0), strandIter(0){}
HairGeo::HairGeo(HairGeo&&o) noexcept : pointIter(o.pointIter), strandIter(o.strandIter), offsets(std::move(o.offsets)), points(std::move(o.points)){}

HairGeo & HairGeo::operator= (HairGeo&&o) {
	pointIter = o.pointIter;
	strandIter = o.strandIter;
	offsets = std::move(o.offsets);
	points = std::move(o.points);
	return *this;
}

HairGeo & HairGeo::operator= (const std::vector<Eigen::Vector3f> &src) {
	if (points.size() == src.size()) points = src;
	return *this;
}

void HairGeo::resetIter() {
	pointIter = 0;
	strandIter = 0;
}

void HairGeo::resize(std::vector<unsigned int> &offs) {
	if (offs.size() == 0) return;
	for (auto i = 1; i < offs.size(); i++) if (offs[i] < offs[i - 1]) return;
	offsets.clear();

	offsets = offs;
	unsigned int numPoints = offsets[offsets.size() - 1];
	points.resize(numPoints, Eigen::Vector3f(0,0,0));

	resetIter();
}
void HairGeo::operator<<(const Eigen::Vector3f &p) {
	if (pointIter >= points.size()) return;
	points[pointIter] = p;
	pointIter++;
}

void HairGeo::operator>>(HairSegment &seg) {
	if (strandIter + 1 >= offsets.size()) return;

	auto nextRootId = offsets[strandIter + 1];
	while (pointIter + 1 >= nextRootId) {
		pointIter = nextRootId;
		strandIter++;
		if (strandIter + 1 >= offsets.size()) return;
		nextRootId = offsets[strandIter + 1];
	}

	auto thisRootId = offsets[strandIter];

	if ((pointIter < thisRootId) || (thisRootId >= nextRootId)) return;

	seg.a.pos = points[pointIter];
	seg.b.pos = points[pointIter + 1];

	seg.a.t = (float)(pointIter - thisRootId) / (float)(nextRootId - thisRootId);
	seg.b.t = (float)(1 + pointIter - thisRootId) / (float)(nextRootId - thisRootId);

	seg.a.id = pointIter;
	seg.b.id = 1 + pointIter;

	pointIter++;
}

void HairGeo::operator>>(HairVertex &seg) {
	if (strandIter + 1 >= offsets.size()) return;

	auto nextRootId = offsets[strandIter + 1];
	while (pointIter>= nextRootId) {
		pointIter = nextRootId;
		strandIter++;
		if (strandIter + 1 >= offsets.size()) return;
		nextRootId = offsets[strandIter + 1];
	}

	auto thisRootId = offsets[strandIter];

	if ((pointIter < thisRootId) || (thisRootId >= nextRootId)) return;

	seg.pos = points[pointIter];
	seg.t = (float)(pointIter - thisRootId) / (float)(nextRootId - thisRootId);
	seg.id = pointIter;

	pointIter++;
}

void HairGeo::operator>>(Eigen::Vector3f &p) {
	if (pointIter >= points.size()) return;
	p = points[pointIter];
	pointIter++;
}

unsigned int HairGeo::numSegments() const {
	unsigned int numSegments = 0;
	unsigned int offSize = offsets.size();
	for (auto i = 1u; i < offSize; i++) {
		auto nPoints = (offsets[i] - offsets[i - 1]);
		if (nPoints>1)
			numSegments += (nPoints-1);
	}
	return numSegments;
}

unsigned int HairGeo::numPoints() const {
	return points.size();
}