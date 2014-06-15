
#ifndef _CLASS_TRAJECTORY
#define _CLASS_TRAJECTORY

#include "MatrixDefs.h"
//#include <Memory.h>

#include <list>

class Trajectory
{
public:
	typedef std::pair  <float, matrices::Vector3>	P_TimePosition;
    typedef std::list<P_TimePosition, Eigen::aligned_allocator<std::pair<const int, matrices::Matrix4> > > T_TimePositions;
	typedef T_TimePositions::iterator				T_TimePositionsIT;
	typedef T_TimePositions::const_iterator			T_TimePositionsCIT;

public: // constructors / destructors
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	 Trajectory(); // todo direciton / trajectory
	 Trajectory(const T_TimePositions& /*timePositions*/); // todo direciton / trajectory
	~Trajectory();

public: // actuators
	void Reset(); // false is time is alreadyt marked
	bool AddCheckPoint(float /*time*/, const matrices::Vector3& /*position*/); // false is time is alreadyt marked
	bool AddWayPoint(T_TimePositionsIT& /*position*/);
	bool ReplaceWayPoint(T_TimePositionsIT& /*position*/, const matrices::Vector3& /*newWaypoint*/);

public: // helpers
	const T_TimePositions& GetTimePositions() const;
	T_TimePositions& GetEditableTimePositions();

private:
	T_TimePositions timePositions_;
};

#endif //_CLASS_TRAJECTORY
