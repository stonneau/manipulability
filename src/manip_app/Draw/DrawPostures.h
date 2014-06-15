
#ifndef _CLASS_DRAWPOSTURES
#define _CLASS_DRAWPOSTURES

#include "API/PostureManagerI.h"

#include <memory>

namespace manip_core
{
	struct RobotI;
}

struct DrawPosturePImpl;

class DrawPostures : public manip_core::PostureCreatedListenerI {

public:
	 DrawPostures();
	~DrawPostures();

	 void Draw(bool transparency = false)const;
	 void DrawOne(bool transparency = false) const;
	 void Clear();
	 const manip_core::RobotI* Next();
	 const manip_core::RobotI* Previous();
	
	virtual void OnPostureCreated(NUMBER time, const manip_core::RobotI* pRobot);

private:
	std::auto_ptr<DrawPosturePImpl> pImpl_;
}; // class PostureSolver

#endif //_CLASS_DRAWPOSTURES