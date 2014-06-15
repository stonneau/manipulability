
#ifndef _CLASS_POSTUREMANAGERI
#define _CLASS_POSTUREMANAGERI

#include "Exports.h"
#include "SampleVisitorI.h"
#include "spline/curve_abc.h"

#include <vector>

namespace manip_core
{
namespace enums
{
namespace postureCriteria
{
	enum ePostureCriteria
	{
		toeOffBoundary = 0, // raise foot when off boundary
		toeOffJointLimit, // raise foot when joint limit reached
		toeOnCOM,           // anchor foot when stability is lost
		toeOffSpiderGait,
		toeOnSpiderGait,
		unknown
	};
}// namespace postureCriteria
}// namespace enums

struct RobotI;

struct MANIPCORE_API PostureCreatedListenerI
{
public:
	/** Called whenever a new posture has been created.
	*/
	virtual void OnPostureCreated(double /*time*/, const RobotI* /*pRobot*/) = 0; // TODO posture destroyed ?
};


typedef std::pair<int, spline::curve_abc<>*>  CubicTrajectory;
typedef std::vector<CubicTrajectory>		T_CubicTrajectory;
typedef T_CubicTrajectory::iterator			IT_CubicTrajectory;

struct MANIPCORE_API PostureManagerI
{
	/** Deletes the current PostureManagerI.
	*/
	virtual void Release() = 0;
	/**	Add a desired posture to the trajectory path. Parameters are time and a 4 square column-major matrix.
	 */
	virtual void AddTrajectoryPoint(const float time, const double* /*transform*/)= 0;

	virtual void SetJumpToTarget(const bool /*jump*/)= 0;

	/*in ms*/
	virtual void Update(const unsigned long time)= 0;

	virtual void ResetTrajectory()= 0;
	/**
	Add a criteria either for raising or lowering foot.
	*/
	virtual void AddPostureCriteria(const enums::postureCriteria::ePostureCriteria /*criteria*/) = 0;
	/**
	Register a PostureCreatedListenerI that will be called whenever a new posture is created
	*/
	virtual void RegisterPostureCreatedListenerI(PostureCreatedListenerI* /*listener*/) = 0;	
	/**
	Unregister a previously created lister
	*/
	virtual void UnRegisterPostureCreatedListenerI(PostureCreatedListenerI* /*listener*/) = 0;
	/**
	Compute solution posture for given trajectory and constraints
	*/
	virtual void InitSamples(const RobotI* /*robot*/, int /*nbSamples*/) = 0;
	virtual void AcceptSampleVisitor(const RobotI* /*robot*/, const TreeI* /*tree*/, SampleVisitorI * /*visitor*/, bool /*collide*/) = 0;

	virtual void ComputeOnline(const RobotI* /*robot*/, int /*nbSamples*/) = 0;

	// direction in world coordinates
	virtual T_CubicTrajectory NextPosture(RobotI* /*robot*/, double* /*dir*/, bool /*closestDistance*/) = 0;

	#ifdef PROFILE
	virtual void Log() const = 0;
	#endif
};

} // namespace manip_core
#endif //_CLASS_POSTUREMANAGERI
