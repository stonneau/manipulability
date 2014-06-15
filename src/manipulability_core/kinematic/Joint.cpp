
#include "kinematic/Joint.h"
#include "Pi.h"
#include "MatrixDefs.h"
#include "ComVisitor_ABC.h"

#include <math.h>

using namespace matrices;
using namespace factories;
using namespace manip_core;
using namespace manip_core::enums;

Joint::Joint(const Vector3& attach, const Vector3& iRotAxis, Purpose purpose, Com com, NUMBER minTheta, NUMBER maxTheta, NUMBER restAngle, rotation::eRotation rot)
	: r_			(0, 0, 0)
	, purpose_		(purpose)
	, attach_		(attach) 
	, v_			(iRotAxis) // Rotation axis when joints at zero angles
	, theta_		(0)
	, pRealparent_	(0)
	, restAngle_	(restAngle)
	, minTheta_		(minTheta)
	, maxTheta_		(maxTheta)
	, pChild_		(0)
	, rot_(rot)
	, com_(com)
{
	// NOTHING
}

Joint::Joint(const Vector3& attach, const Vector3& iRotAxis, Purpose purpose, Com com, NUMBER restAngle, rotation::eRotation rot)
	: r_			(0, 0, 0)
	, purpose_		(purpose)
	, attach_		(attach) 
	, v_			(iRotAxis) // Rotation axis when joints at zero angles
	, theta_		(0)
	, pRealparent_	(0)
	, restAngle_	(restAngle)
	, minTheta_		(-Pi)
	, maxTheta_		(Pi)
	, pChild_		(0)
	, rot_(rot)
	, com_(com)
{
	// NOTHING
}

Joint::~Joint()
{
	//NOTHING
}

NUMBER Joint::AddToTheta(const NUMBER delta)
{ 
	NUMBER newTheta = theta_ + delta;
	while (newTheta > 360 * DegreesToRadians )
	{
		newTheta -= 360 * DegreesToRadians;
	}
	while (newTheta < -360 * DegreesToRadians )
	{
		newTheta += 360 * DegreesToRadians;
	}
	if(newTheta < 0 * DegreesToRadians)
	{
		newTheta = (360 * DegreesToRadians + newTheta);
	}
	/*if(newTheta > maxTheta_)
	{
		theta_ = maxTheta_;
		return abs(abs(maxTheta_) - abs(newTheta));
	}
	else if (newTheta < minTheta_)
	{
		theta_ = minTheta_;
		return - abs(abs(minTheta_) - abs(newTheta));
	}
	else
	{
		theta_ = newTheta;
		return 0.f;
	}
	return newTheta - theta_;*/
	theta_ = newTheta;
	return 0.f;
}


// Compute the global position of a single Joint
const matrices::Vector3& Joint::ComputeS(void)
{
	Joint* y = this->pRealparent_;
	Joint* w = this;
	s_ = r_;							// Initialize to local (relative) position
	while (y) {
		Rotate(y->v_, s_, y->theta_);
		y = y->pRealparent_;
		w = w->pRealparent_;
		s_ += w->r_;
	}
	return s_;
}

// Compute the global rotation axis of a single Joint
void Joint::ComputeW(void)
{
	Joint* y = this->pRealparent_;
	w_ = v_;							// Initialize to local rotation axis
	while (y) {
		Rotate(y->v_, w_, y->theta_);
		y = y->pRealparent_;
	}
}

void Joint::InitJoint()
{
	theta_ = restAngle_;
}

void Joint::ToRest()
{
	theta_ = restAngle_;
}

const bool Joint::IsLocked() const 
{
	return minTheta_ == maxTheta_;
}


Joint* Joint::Clone() const
{
	Joint* res = new Joint(attach_, v_, purpose_, com_, minTheta_, maxTheta_, restAngle_, rot_);
	res->theta_ = theta_;
	return res;
}

void Joint::AcceptComVisitor(ComVisitor_ABC* visitor) const
{
	if( com_.weight_ != 0 )
	{
		Vector3 proximal = pRealparent_ ? pRealparent_->s_ : (Vector3(0,0,0));
		visitor->VisitCom(com_.Compute(proximal, s_), com_.weight_);
	}
}

bool Joint::IsEffector() const
{
	return purpose_==EFFECTOR;
} 

bool Joint::IsJoint() const
{
	return purpose_==JOINT;
} 

void Joint::Release()
{
	delete this;
}
double Joint::GetAngle() const
{
	return GetTheta();
}
void Joint::Offset(double * offset) const
{
	matrices::vect3ToArray(offset, r_);
}

const manip_core::enums::rotation::eRotation Joint::GetRotation() const
{
	return rot_;
}

const JointI* Joint::GetSon() const
{
	return pChild_;
}

const JointI* Joint::GetParent() const
{
	return pRealparent_;
}

