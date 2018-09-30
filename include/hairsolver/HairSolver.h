#pragma once

#include <Eigen/Core>
#include <Eigen\Geometry>
#include "HairGeo.h"

class HairDoF {
public:
	HairDoF();

	void rotateFromPrev(Eigen::Quaternionf &rot);
	void copyRootsFromHair(HairDoF &src);
	void copyRootsToHair(HairDoF &dst);

	virtual unsigned int vertexSize() const = 0;
	virtual void advance(float timestep, float gravity) = 0;
	virtual void extraInitialize() {};


	HairDoF & operator= (const HairGeo&o);
	
	Eigen::VectorXf &getDoFs() { return mDof; }
	Eigen::VectorXf &getPrevDoFs() { return mDofPrev; }
	Eigen::VectorXi &getTopology() { return mTopology; }
	Eigen::VectorXi &getPointType() { return mPointType; }

	float mHairRadius;

private:
	Eigen::VectorXf mDof;
	Eigen::VectorXf mDofPrev;

	Eigen::VectorXi mTopology;
	Eigen::VectorXi mPointType;
};

class HairDoF_Points : public HairDoF {
public:
	HairDoF_Points();
	HairDoF_Points & operator= (const HairGeo&o);

	unsigned int vertexSize() const;
	void advance(float timestep, float gravity);
};

class HairDoF_PointsAndQuaternions : public HairDoF {
public:
	HairDoF_PointsAndQuaternions();
	HairDoF_PointsAndQuaternions & operator= (const HairGeo&o);

	unsigned int vertexSize() const;
	void advance(float timestep, float gravity);
	void extraInitialize() override;
};


class HairModel {
public:

	HairModel();
	void updateRoots(HairDoF &roots);
	void step(HairDoF &dof) const;
	void reset();
	virtual void solve(HairDoF &dof) const = 0;

	float mTimestep;
	float mGravity;
	float mSegmentLength;
	unsigned int mStiffness;

	float mRotXfreq;
	float mRotYfreq;
	float mRotZfreq;
	float mRotXamp;
	float mRotYamp;
	float mRotZamp;

	float mCurrentTime;
	bool mTransform;
	Eigen::Quaternionf mRootRotation, mCurrentRootRotation;
};

class HairModel_FollowTheLeader : public HairModel {
public:

	HairModel_FollowTheLeader();

	void solve(HairDoF &dof) const;
};

class HairModel_PBD_Cosserat : public HairModel {
public:
	HairModel_PBD_Cosserat();

	void solve(HairDoF &dof) const;
	void solveStrand(HairDoF &dof, unsigned int i, float pointDisplacementScale, float quaternionDisplacementScale, float twistBendFactor) const;
};