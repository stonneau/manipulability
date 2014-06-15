
#ifndef _CLASS_POSTUREMANAGERIMPL
#define _CLASS_POSTUREMANAGERIMPL

#include "world/World.h"
#include "posture/PostureSolver.h"
#include "posture/Trajectory.h"
#include "SpiderGait.h"
#include "spline/exact_cubic.h"

#include "API/PostureManagerI.h"

class Robot;

namespace manip_core
{
struct RobotI;

class PostureManagerImpl : public PostureManagerI
{
public :

	 PostureManagerImpl(const World& world);
	~PostureManagerImpl();

public:

	virtual void Release();

	virtual void ResetTrajectory();

	virtual void SetJumpToTarget(const bool /*jump*/);

	virtual void AddTrajectoryPoint(const float time, const double* transform);
	/**
	Add a criteria either for raising or lowering foot.
	*/
	virtual void AddPostureCriteria(const enums::postureCriteria::ePostureCriteria /*criteria*/);
	/**
	Register a PostureCreatedListenerI that will be called whenever a new posture is created
	*/
	virtual void RegisterPostureCreatedListenerI(PostureCreatedListenerI* /*listener*/);	
	/**
	Unregister a previously created lister
	*/
	virtual void UnRegisterPostureCreatedListenerI(PostureCreatedListenerI* /*listener*/);
	/**
	Compute solution posture for given trajectory and constraints
	*/
	virtual void ComputeOnline(const RobotI* /*robot*/, int /*nbSamples*/);

	virtual T_CubicTrajectory NextPosture(RobotI* /*robot*/, double* /*dir*/, bool /*closestDistance*/);

	virtual void InitSamples(const RobotI* /*robot*/, int /*nbSamples*/);
	virtual void AcceptSampleVisitor(const RobotI* /*robot*/, const TreeI* /*tree*/,  SampleVisitorI * /*visitor*/, bool /*collide*/);

	virtual void Update(const unsigned long /*time*/);

	#ifdef PROFILE
	virtual void Log() const;
	#endif

private:
	PostureSolver pSolver_;
	Trajectory trajectory_;
	bool initialized_;
	const World& world_;
	SpiderGait* spiderGait_;
};

} // namespace manip_core
#endif //_CLASS_POSTUREMANAGERIMPL

