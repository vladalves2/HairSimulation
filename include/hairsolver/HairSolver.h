#pragma once

#include <Eigen/Core>
#include "HairGeo.h"

class HairDoF {
public:
	HairDoF();

	virtual void advance(float timestep, float gravity) = 0;

	Eigen::VectorXf mDof;
	Eigen::VectorXf mDofPrev;

	Eigen::VectorXi mTopology;
};

class HairDoF_Points : public HairDoF {
public:
	HairDoF_Points();

	void advance(float timestep, float gravity);

	HairDoF_Points & operator= (const HairGeo&o);
};

//class HairDoF_PointsAndQuaternions : HairDoF {
//public:
//	HairDoF_PointsAndQuaternions();
//
//	void advance(float timestep, float gravity);
//};


class HairModel {
public:

	HairModel();
	void step(HairDoF &dof) const;
	virtual void solve(HairDoF &dof) const = 0;

	float mTimestep;
	float mGravity;
	float mSegmentLength;
};

class HairModel_FollowTheLeader : public HairModel {
public:

	HairModel_FollowTheLeader();

	void solve(HairDoF &dof) const;
};