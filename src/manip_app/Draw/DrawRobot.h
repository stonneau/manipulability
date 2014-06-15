
#ifndef _CLASS_DRAWROBOT
#define _CLASS_DRAWROBOT


#include <memory>

struct PImpl;
class Robot;
class Joint;

namespace manip_core
{
	struct RobotI;
}

class DrawRobot {

public:
	 DrawRobot(const manip_core::RobotI* robot);
	~DrawRobot();

	 void ToggleSupportPolygon(bool /*onoff*/);
	 void Draw(bool transparency = false)const;
	 void DrawNoTexture()const;
	 void SetTarget(const manip_core::RobotI* robot);
	 const manip_core::RobotI* GetRobot();
	
private:
	void DrawEllipsoid(int treeId) const;

private:
	std::auto_ptr<PImpl> pImpl_;
}; // class DrawRobot

#endif //_CLASS_DRAWROBOT