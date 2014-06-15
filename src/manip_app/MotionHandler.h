
#ifndef _CLASS_MOTIONHANDLER
#define _CLASS_MOTIONHANDLER

#include "TimerHandler.h"
#include "MatrixDefs.h"
#include "IKSolver/IKSolver.h"
#include "SplineTimeManager.h"
#include "ManipManager.h"

#include "API/TreeI.h"
#include "API/RobotI.h"

#include <map>
#include <queue>

namespace manip_core
{
	struct RobotI;
	struct TreeI;
}

class MotionAction_ABC
{
public:
	 MotionAction_ABC(){};
	~MotionAction_ABC(){};

	virtual void operator() (manip_core::RobotI& /*robot*/, const Timer::t_time /*dt*/) = 0;
};

class MotionHandler : public TimerHandled_ABC
{
public:
	explicit MotionHandler(manip_core::ManipManager& manager, const float rotationSpeed = 1.f, const float translationSpeed = 1.f);
	~MotionHandler();

private:
	MotionHandler(const MotionHandler&);
	MotionHandler& operator=(const MotionHandler&);

public:
	virtual void Update(const Timer::t_time /*t*/, const Timer::t_time /*dt*/);
	virtual void Reset();

public:
	void PushAction(MotionAction_ABC* /*action*/); // done only at update
	void Translate(const matrices::Vector3& /*direction*/); // done only at update
	void MoveBy(const matrices::Vector3& /*direction*/); // done only at update
	void Rotate(const matrices::Matrix3& /*rotation*/); // done only at update
	const matrices::Vector3& GetDirection() const{return previousDirection_;} // done only at update

public:
	const IKSolverApp& solver_;
	const NUMBER rotationSpeed_, translationSpeed_;

public:
	typedef std::map<int, spline::SplineTimeManager*>T_SplineManager;
	typedef T_SplineManager::iterator				IT_SplineManager;
	typedef T_SplineManager::const_iterator			CIT_SplineManager;

	typedef std::queue<MotionAction_ABC*> T_Action_;

private:
	manip_core::PostureManager* postureManager_;
	matrices::Vector3 previousDirection_;
	T_SplineManager splineManagers_;
	T_Action_ actions_;
};

#endif //_CLASS_MOTIONHANDLER
