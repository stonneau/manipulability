
#ifndef _CLASS_FILTER_DISTANCE_OBSTACLE
#define _CLASS_FILTER_DISTANCE_OBSTACLE

#include <memory>

#include "Filter_ABC.h"
#include "MatrixDefs.h"

class Sample;
class Obstacle;
class Robot;
class Tree;

struct FilterDPImpl;

// checks that end-effector is "around" obstacle
class FilterDistanceObstacle : public Filter_ABC {

public:
	FilterDistanceObstacle(NUMBER /*treshold*/, const Tree& /*tree*/, const Obstacle& /*obstacle*/, const Robot& /*robot*/, const matrices::Vector3& /*direction*/);
	~FilterDistanceObstacle();

protected:
	virtual bool ApplyFilter(const Sample& /*sample*/) const;

private:
	std::auto_ptr<FilterDPImpl> pImpl_;
};


#endif //_CLASS_FILTER_DISTANCE_OBSTACLE