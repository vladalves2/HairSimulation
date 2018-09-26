#include "HairSolver.h"
#include <iostream>

HairDoF::HairDoF() {}

HairDoF_Points::HairDoF_Points() {}

HairDoF_Points & HairDoF_Points::operator= (const HairGeo&o) {
	auto nPs = o.numPoints();
	auto nStrands = o.numStrands();
	mDof.resize(nPs * 3);
	mDofPrev.resize(nPs * 3);
	mTopology.resize(nStrands + 1);
	for (auto i = 0u; i <= nStrands; i++)
		mTopology[i] = o.offsets[i];
	for (auto i = 0u; i < nPs; i++) {
		Eigen::Map<Eigen::Vector3f>(mDof.data() + i * 3) = o.points[i];
	}
	mDofPrev = mDof;
	return *this;
}

void HairDoF_Points::advance(float timestep, float gravity) {
	int nPoints = mDof.size() / 3;

	Eigen::Vector3f accel(0, gravity, 0);

#pragma omp parallel for
	for (int pid = 0; pid < nPoints; pid++) {
		Eigen::Map<Eigen::Vector3f> pNow (mDof.data() + pid * 3);
		Eigen::Map<Eigen::Vector3f> pPrev (mDofPrev.data() + pid * 3);

		Eigen::Vector3f p0 = pNow;
		pNow = pNow + (pNow - pPrev) + accel * (timestep*timestep*0.5f);
		pPrev = p0;
	}
}




HairModel::HairModel() : mTimestep(0.005f), mGravity(-9.81f), mSegmentLength(0.02f){}

void HairModel::step(HairDoF &dof) const {
	dof.advance(mTimestep, mGravity);
	solve(dof);
}

HairModel_FollowTheLeader::HairModel_FollowTheLeader() : HairModel() {}

void HairModel_FollowTheLeader::solve(HairDoF &dof) const {
	int nHairs = dof.mTopology.size();
	nHairs--;

#pragma omp parallel for
	for (int hid = 0; hid < nHairs; hid++) {
		int start = dof.mTopology[hid];
		int end = dof.mTopology[hid + 1];

		{
			Eigen::Map<Eigen::Vector3f> pNow(dof.mDof.data() + start * 3);
			Eigen::Map<Eigen::Vector3f> pPrev(dof.mDofPrev.data() + start * 3);

			pNow = pPrev;
		} 

		for (int pid = start + 1; pid < end; pid++) {
			Eigen::Map<Eigen::Vector3f> A(dof.mDof.data() + (pid-1) * 3);
			Eigen::Map<Eigen::Vector3f> B(dof.mDof.data() + pid * 3);

			Eigen::Vector3f oldB = B;

			Eigen::Vector3f seg = (B - A);
			float currentLen = seg.norm();
			if (currentLen > mSegmentLength) {
				seg.normalize();
				B = A + seg * mSegmentLength;

				Eigen::Map<Eigen::Vector3f> APrev(dof.mDofPrev.data() + (pid - 1) * 3);
				APrev += (B - oldB);
			}
		}
	}
}