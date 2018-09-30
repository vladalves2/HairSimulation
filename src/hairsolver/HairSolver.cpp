#include "HairSolver.h"
#include <iostream>

HairDoF::HairDoF() : mHairRadius(1.0f){}

HairDoF & HairDoF::operator= (const HairGeo&o) {
	auto nPs = o.numPoints();
	auto nStrands = o.numStrands();
	Eigen::VectorXf& dof = getDoFs();
	Eigen::VectorXf& dofprev = getPrevDoFs();
	auto vtxSize = vertexSize();
	Eigen::VectorXi& topo = getTopology();
	Eigen::VectorXi& type = getPointType();

	dof.resize(nPs * vtxSize);
	dofprev.resize(nPs * vtxSize);
	type.resize(nPs);
	topo.resize(nStrands + 1);

	type.fill(1);

	for (auto i = 0u; i <= nStrands; i++) {
		auto off = o.offsets[i];
		topo[i] = off;
		if (i != nStrands) type[off] = 0;
		if (i != 0) type[off - 1] = 2;
	}

	for (auto i = 0u; i < nPs; i++) {
		Eigen::Map<Eigen::Vector3f>(dof.data() + i * vtxSize) = o.points[i];
	}

	extraInitialize();

	dofprev = dof;

	return *this;
}

void HairDoF::rotateFromPrev(Eigen::Quaternionf &rot) {
	Eigen::Matrix3f rotMatrix = rot.toRotationMatrix();

	Eigen::VectorXf &elements = getDoFs();
	Eigen::VectorXf &prevElements = getPrevDoFs();
	auto elementSize = vertexSize();

	int nElements = elements.size()/ elementSize;
#pragma omp parallel for
	for (int id = 0; id < nElements; id++) {
		Eigen::Map<Eigen::Vector3f> src(prevElements.data() + id * elementSize);
		Eigen::Map<Eigen::Vector3f> dst(elements.data() + id * elementSize);
		dst = rotMatrix * src;
	}
}

void HairDoF::copyRootsFromHair(HairDoF &src) {
	auto elementSize = src.vertexSize();
	if (elementSize != vertexSize()) {
		std::cout << "copyRootsFromHair error: elementSize incompatible" << std::endl;
		return;
	}

	Eigen::VectorXf &srcElements = src.getDoFs();
	Eigen::VectorXi &srcTopo = src.getTopology();

	int nHairs = srcTopo.size() - 1;

	if (nHairs < 0) nHairs = 0;

	Eigen::VectorXf &dstElements = getDoFs();
	dstElements.resize(nHairs * elementSize);

	for (int i = 0; i < nHairs; i++) {
		float * src = (srcElements.data() + srcTopo[i] * elementSize);
		float * dst = (dstElements.data() + i * elementSize);
		for (auto j = 0; j < elementSize; j++)
			dst[j] = src[j];
	}


	Eigen::VectorXf &prevDstElements = getPrevDoFs();
	prevDstElements = dstElements;
}

void HairDoF::copyRootsToHair(HairDoF &dst) {
	auto elementSize = dst.vertexSize();
	if (elementSize != vertexSize()) {
		std::cout << "copyRootsToHair error: elementSize incompatible" << std::endl;
		return;
	}

	Eigen::VectorXf &dstElements = dst.getDoFs();
	Eigen::VectorXi &dstTopo = dst.getTopology();

	int nHairs = dstTopo.size() - 1;

	if (nHairs < 0) nHairs = 0;

	Eigen::VectorXf &srcElements = getDoFs();

	for (int i = 0; i < nHairs; i++) {
		float * dst = (dstElements.data() + dstTopo[i] * elementSize);
		float * src = (srcElements.data() + i * elementSize);
		for (auto j = 0; j < elementSize; j++)
			dst[j] = src[j];
	}
}

HairDoF_PointsAndQuaternions::HairDoF_PointsAndQuaternions() {}

HairDoF_PointsAndQuaternions & HairDoF_PointsAndQuaternions::operator= (const HairGeo&o) {
	HairDoF::operator=(o);
	return *this;
}

unsigned int HairDoF_PointsAndQuaternions::vertexSize() const { return 7; }

void HairDoF_PointsAndQuaternions::extraInitialize() {
	Eigen::VectorXf& dof = getDoFs();
	auto vtxSize = vertexSize();
	Eigen::VectorXi& topo = getTopology();
	auto nStrands = topo.size() - 1;

	for (auto hid = 0u; hid < nStrands; hid++) {
		//initialize quaternions
		auto start = topo[hid];
		auto end = topo[hid + 1];

		for (auto pid = start; pid < end-1; pid++) {
			Eigen::Map<Eigen::Vector3f> a(dof.data() + pid * vtxSize);
			Eigen::Map<Eigen::Vector3f> b(dof.data() + (pid+1) * vtxSize);

			Eigen::Vector3f segment = b - a;
			segment.normalize();
			Eigen::Map<Eigen::Quaternionf>  q(dof.data() + (pid + 1) * vtxSize + 3);
			q = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitX(), segment);
		}
		Eigen::Map<Eigen::Quaternionf>(dof.data() + start * vtxSize + 3) = Eigen::Map<Eigen::Quaternionf>(dof.data() + (start + 1) * vtxSize + 3);
	}
}
void HairDoF_PointsAndQuaternions::advance(float timestep, float gravity) {
	Eigen::VectorXi &types = getPointType();
	Eigen::VectorXf& dof = getDoFs();
	Eigen::VectorXf& dofprev = getPrevDoFs();
	auto vtxSize = vertexSize();
	int nPoints = dof.size() / vtxSize;

	Eigen::Vector3f accel(0, gravity, 0);

	const float inertia = EIGEN_PI * mHairRadius * mHairRadius * mHairRadius * mHairRadius * 0.25f;
	Eigen::Matrix3f I; 
	I.setIdentity();
	I(0, 0) = inertia;
	I(1, 1) = inertia;
	I(2, 2) = 2* inertia;
	Eigen::Matrix3f Iinv = I.inverse();

#pragma omp parallel for
	for (int pid = 0; pid < nPoints; pid++) {
		if (types[pid] == 0) continue;
		////////////////
		//update positions
		////////////////
		Eigen::Map<Eigen::Vector3f> pNow(dof.data() + pid * vtxSize);
		Eigen::Map<Eigen::Vector3f> pPrev(dofprev.data() + pid * vtxSize);

		Eigen::Vector3f p0 = pNow;
		pNow = pNow + (pNow - pPrev) + accel * (timestep*timestep*0.5f);
		pPrev = p0;

		Eigen::Map<Eigen::Quaternionf> qNow(dof.data() + pid * vtxSize + 3);
		Eigen::Map<Eigen::Quaternionf> qPrev(dofprev.data() + pid * vtxSize + 3);

		////////////////
		//update quaternions
		////////////////

		//compute angular velocity from quaternions
		Eigen::Quaternionf q0 = qNow;
		Eigen::Vector3f w = ((qNow * qPrev.conjugate()).vec()) * (2 / timestep);
		const Eigen::Vector3f torque = -w.cross(I*w);
		////integrate angular velocity
		w = (w + timestep * (Iinv * torque));
		Eigen::Quaternionf wq(0.0f, w.x(), w.y(), w.z());

		Eigen::Quaternionf qTmp = (wq * qNow);
		qNow.vec() += (0.5f * timestep) * qTmp.vec();
		qNow.w() += (0.5f * timestep) * qTmp.w();

		qNow.normalize();
		qPrev = q0;
	}

}


HairDoF_Points::HairDoF_Points() {}

HairDoF_Points & HairDoF_Points::operator= (const HairGeo&o) {
	HairDoF::operator=(o);
	return *this;
}

unsigned int HairDoF_Points::vertexSize() const { return 3; }

void HairDoF_Points::advance(float timestep, float gravity) {
	Eigen::VectorXi &types = getPointType();
	Eigen::VectorXf& dof = getDoFs();
	Eigen::VectorXf& dofprev = getPrevDoFs();
	auto vtxSize = vertexSize();

	int nPoints = dof.size() / vtxSize;

	Eigen::Vector3f accel(0, gravity, 0);

#pragma omp parallel for
	for (int pid = 0; pid < nPoints; pid++) {
		if (types[pid] != 0) {
			Eigen::Map<Eigen::Vector3f> pNow(dof.data() + pid * vtxSize);
			Eigen::Map<Eigen::Vector3f> pPrev(dofprev.data() + pid * vtxSize);

			Eigen::Vector3f p0 = pNow;
			pNow = pNow + (pNow - pPrev) + accel * (timestep*timestep*0.5f);
			pPrev = p0;
		}
	}
}

void HairModel::reset() {
	mCurrentTime = 0;
	mCurrentRootRotation = Eigen::Quaternionf(0, 1, 0, 0);
	mRootRotation = Eigen::Quaternionf(0, 1, 0, 0);
}

void HairModel::updateRoots(HairDoF &roots) {
	mTransform = ((mRotXfreq * mRotXamp) != 0) || ((mRotYfreq * mRotYamp) != 0) || ((mRotZfreq * mRotZamp) != 0);

	if (mTransform) {
		float thetaX = (mRotXamp * EIGEN_PI * mRotXfreq * mCurrentTime) / 180.0f;
		float thetaY = (mRotYamp * EIGEN_PI * mRotYfreq * mCurrentTime) / 180.0f;
		float thetaZ = (mRotZamp * EIGEN_PI * mRotZfreq * mCurrentTime) / 180.0f;
		mRootRotation = Eigen::AngleAxisf(thetaX, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(thetaY, Eigen::Vector3f::UnitY())* Eigen::AngleAxisf(thetaZ, Eigen::Vector3f::UnitZ());
		mCurrentRootRotation.slerp(0.5f, mRootRotation);

		roots.rotateFromPrev(mCurrentRootRotation);		
	}

	mCurrentTime += mTimestep;
}


HairModel::HairModel() : mTimestep(0.005f), mGravity(-9.81f), mSegmentLength(0.02f), mStiffness(0), mRotXfreq(0), mRotYfreq(0), mRotXamp(0), mRotYamp(0), mCurrentTime(0) {}

void HairModel::step(HairDoF &hair) const {	
	hair.advance(mTimestep, mGravity);
	solve(hair);
}

HairModel_FollowTheLeader::HairModel_FollowTheLeader() : HairModel() {}

void HairModel_FollowTheLeader::solve(HairDoF &dof) const {
	Eigen::VectorXf& coords = dof.getDoFs();
	Eigen::VectorXf& coordsPrev = dof.getPrevDoFs();
	Eigen::VectorXi& topo = dof.getTopology();
	auto vertexSize = dof.vertexSize();

	int nHairs = topo.size();
	nHairs--;

#pragma omp parallel for
	for (int hid = 0; hid < nHairs; hid++) {
		int start = topo[hid];
		int end = topo[hid + 1];

		for (int pid = start + 1; pid < end; pid++) {
			Eigen::Map<Eigen::Vector3f> A(coords.data() + (pid-1) * vertexSize);
			Eigen::Map<Eigen::Vector3f> B(coords.data() + pid * vertexSize);

			Eigen::Vector3f oldB = B;

			Eigen::Vector3f seg = (B - A);
			float currentLen = seg.norm();
			if (currentLen > mSegmentLength) {
				seg.normalize();
				B = A + seg * mSegmentLength;

				if (pid > start + 1) {
					Eigen::Map<Eigen::Vector3f> APrev(coordsPrev.data() + (pid - 1) * vertexSize);
					APrev += (B - oldB);
				}
			}
		}
	}
}

HairModel_PBD_Cosserat::HairModel_PBD_Cosserat() : HairModel() {}

void HairModel_PBD_Cosserat::solveStrand(HairDoF &dof, unsigned int pid, float gammaScale, float quaternionDisplacementScale, float twistBendFactor) const {

	Eigen::VectorXi& type = dof.getPointType();
	Eigen::VectorXf& coords = dof.getDoFs();
	auto vertexSize = dof.vertexSize();

	if (type[pid] == 0) return;

	Eigen::Map<Eigen::Vector3f> A(coords.data() + (pid - 1) * vertexSize);
	Eigen::Map<Eigen::Vector3f> B(coords.data() + pid * vertexSize);
	Eigen::Map<Eigen::Quaternionf> qA(coords.data() + (pid - 1) * vertexSize + 3);
	Eigen::Map<Eigen::Quaternionf> qB(coords.data() + pid * vertexSize + 3);

	Eigen::Vector3f d3 = (qA * Eigen::Quaternionf(0, 1, 0, 0) * qA.conjugate()).vec();
	Eigen::Vector3f stretchShearStrain = ((B - A) / mSegmentLength - d3);
	Eigen::Vector3f pointDisp = stretchShearStrain * gammaScale;

	if (type[pid - 1] != 0) A += pointDisp;
	B -= (pointDisp);

	Eigen::Quaternionf quatDisp = Eigen::Quaternionf(0.0f, stretchShearStrain.x(), stretchShearStrain.y(), stretchShearStrain.z()) * qB * Eigen::Quaternionf(0,-1,0,0);

	Eigen::Quaternionf darboux = (qA.conjugate() * qB);
	Eigen::Quaternionf omega(0, darboux.x()* twistBendFactor, darboux.y()* twistBendFactor, darboux.z()* twistBendFactor);

	Eigen::Quaternionf qBdisp = qA * omega;

	if (type[pid - 1] != 0) {
		Eigen::Quaternionf qAdisp = qB * omega;
		qA.w() += qAdisp.w();
		qA.vec() += qAdisp.vec();
		qA.normalize();
	}

	qB.vec() += quatDisp.vec() * quaternionDisplacementScale -qBdisp.vec();
	qB.w() += quatDisp.w() * quaternionDisplacementScale - qBdisp.w();
	qB.normalize();
}

void HairModel_PBD_Cosserat::solve(HairDoF &dof) const {
	Eigen::VectorXf& coords = dof.getDoFs();
	Eigen::VectorXf& coordsPrev = dof.getPrevDoFs();
	Eigen::VectorXi& topo = dof.getTopology();
	Eigen::VectorXi& type = dof.getPointType();
	auto vertexSize = dof.vertexSize();

	int nHairs = topo.size();
	nHairs--;
	int nPoints = topo[nHairs];

	const float hairDensity = 0.0013f;
	const float radius = dof.mHairRadius;
	const float pointVolume = (mSegmentLength * EIGEN_PI * radius * radius);
	const float pointMass = pointVolume * hairDensity;
	const float invPointMass = 1.0f / pointMass;
	const float segmentInertia = EIGEN_PI * radius * radius * radius * radius * 0.25f;
	const float invSegmentInertia = 1.0f / segmentInertia;
	const float gammaScale = pointMass / ((2 * pointMass / mSegmentLength) + 4 * mSegmentLength * invSegmentInertia  + 1.0e-6f);
	const float quaternionDisplacementScale = 2 * invSegmentInertia * mSegmentLength;//(mSegmentLength * mSegmentLength * invSegmentInertia) / (invPointMass + invPointMass + 4 * invSegmentInertia * mSegmentLength * mSegmentLength);
	
	const float twistBendStiffness = 1.0f;
	const float twistBendFactor = twistBendStiffness / (2* invSegmentInertia + 1.0e-6f);

	for (auto iter = 0; iter < mStiffness; iter++) {
#pragma omp parallel for
		for (int pid = 0; pid < nPoints; pid += 2) solveStrand(dof, pid, gammaScale, quaternionDisplacementScale, twistBendFactor);

#pragma omp parallel for
		for (int pid = 1; pid < nPoints; pid += 2) solveStrand(dof, pid, gammaScale, quaternionDisplacementScale, twistBendFactor);
	}
}