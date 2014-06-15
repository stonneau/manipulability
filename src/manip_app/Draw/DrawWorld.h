
#ifndef _CLASS_DRAWWORLD
#define _CLASS_DRAWWORLD

#include "ManipManager.h"

#include <memory>

struct PImpl;
class World;

class DrawWorld : public manip_core::ObstacleVisitor_ABC {

public:
	 DrawWorld(manip_core::ManipManager& /*manager*/);
	~DrawWorld();

	 void Draw()const;
	 virtual void OnObstacleCreated(const matrices::Vector3& /*upLeft*/, const matrices::Vector3& /*upRight*/, const matrices::Vector3& /*downRight*/, const matrices::Vector3& /*downLeft*/, float* color, const float transparency, const int texture);	
	 virtual void OnWallCreated(const matrices::Vector3& /*upLeft*/, const matrices::Vector3& /*upRight*/, const matrices::Vector3& /*downRight*/, const matrices::Vector3& /*downLeft*/, float* color, const float transparency, const int texture);	
	 virtual void OnGroundCreated(const matrices::Vector3& /*upLeft*/, const matrices::Vector3& /*upRight*/, const matrices::Vector3& /*downRight*/, const matrices::Vector3& /*downLeft*/, float* color, const float transparency, const int texture);	
	

private:
	std::auto_ptr<PImpl> pImpl_;
}; // class DrawWorld

#endif //_CLASS_DRAWWORLD