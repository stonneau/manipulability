#include "TimerHandler.h"


TimerHandler::TimerHandler()
	: lastUpdateTime_(0)
{
	// NOTHING
}

TimerHandler::~TimerHandler()
{
	// NOTHING
}

void TimerHandler::Start(Timer::t_time initTime) {
	timer_.Start(initTime);
	lastUpdateTime_ = timer_.GetTime();
}

void TimerHandler::Reset(Timer::t_time initTime) {
	timer_.Reset(initTime);
	for(TimerHandled_ABC::IT_TimerHandled_ABC it = listeners_.begin(); it != listeners_.end(); ++it)
	{
		(*it)->Reset();
	}
	lastUpdateTime_ = 0;
}

void TimerHandler::Update() {
	Timer::t_time currentTime = timer_.GetTime();
	Timer::t_time dt = (currentTime - lastUpdateTime_);
	lastUpdateTime_ = currentTime;
	for(TimerHandled_ABC::IT_TimerHandled_ABC it = listeners_.begin(); it != listeners_.end(); ++it)
	{
		(*it)->Update(currentTime / 1000., dt / 1000.);
	}
}

void TimerHandler::Register(TimerHandled_ABC* listener)
{
	listeners_.push_back(listener);
}

Timer& TimerHandler::GetTimer()
{
	return timer_;
}
