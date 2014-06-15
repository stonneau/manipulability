
#ifndef _CLASS_XBOXMOTION
#define _CLASS_XBOXMOTION

#include "MotionHandler.h"
#ifdef _WIN32
#include "XBOXController/Controller.h"
#endif

class XboxMotion : public TimerHandled_ABC
{
public:
	explicit XboxMotion();
	~XboxMotion();

private:
	XboxMotion(const XboxMotion&);
	XboxMotion& operator=(const XboxMotion&);

public:
	virtual void Update(const Timer::t_time /*t*/, const Timer::t_time /*dt*/);
	virtual void Reset();

#ifdef _WIN32
private:
    xbox::Controller controller_;
#endif

};

#endif //_CLASS_XBOXMOTION
