
#ifndef _CLASS_ROOTTRAJECTORY
#define _CLASS_ROOTTRAJECTORY

#include "TimerHandler.h"
#include "MatrixDefs.h"
#include "spline/curve_abc.h"


class RootTrajectory : public TimerHandled_ABC
{
public:
	typedef Eigen::Vector3d point_t;
	typedef spline::curve_abc<> curve_abc_t;

	explicit RootTrajectory(curve_abc_t* /*spline*/, const matrices::Vector3& /*initDir*/);
	~RootTrajectory();

private:
	RootTrajectory(const RootTrajectory&);
	RootTrajectory& operator=(const RootTrajectory&);

public:
	virtual void Update(const Timer::t_time /*t*/, const Timer::t_time /*dt*/);
	virtual void Reset();

private:
	curve_abc_t* spline_;
	matrices::Vector3 previous_;
	const matrices::Vector3 initDir_;
};

#endif //_CLASS_ROOTTRAJECTORY
