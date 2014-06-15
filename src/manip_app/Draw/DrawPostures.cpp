
#include "API/PostureManagerI.h"
#include "API/RobotI.h"

#include "MatrixDefs.h"

#include "DrawPostures.h"
#include "DrawRobot.h"

#include <drawstuff/drawstuff.h> // The drawing library for ODE;
#include <vector>

using namespace std;
using namespace matrices;
using namespace manip_core;

struct DrawPosturePImpl
{
	DrawPosturePImpl()
	{
		current_ = drawRobots_.end();
	}

	~DrawPosturePImpl()
	{
		//solver_.UnRegisterPostureCreatedListenerI(*this);
		for(T_DrawRobotIT it = drawRobots_.begin(); it != drawRobots_.end(); ++it)
		{
			delete(*it);
		}
	}

	virtual void OnPostureCreated(NUMBER time, const RobotI* pRobot)
	{
		drawRobots_.push_back(new DrawRobot(pRobot));
		current_ = drawRobots_.begin();
	}

	typedef vector<DrawRobot*> T_DrawRobot;
	typedef T_DrawRobot::iterator T_DrawRobotIT;
	typedef T_DrawRobot::const_iterator T_DrawRobotCIT;
	T_DrawRobot drawRobots_;
	DrawPosturePImpl::T_DrawRobotCIT current_;
};

//TODO listener in case of adding new trees
DrawPostures::DrawPostures()
	: pImpl_(new DrawPosturePImpl)
{
	// TODO
}

DrawPostures::~DrawPostures()
{
	// TODO
}

void DrawPostures::Draw(bool transparency) const
{
	if(pImpl_->current_ != pImpl_->drawRobots_.end())
	{
		//(*(pImpl_->current_))->ToggleSupportPolygon(true);
		(*(pImpl_->current_))->Draw(transparency);
	}for(DrawPosturePImpl::T_DrawRobotCIT it = pImpl_->drawRobots_.begin(); it != pImpl_->drawRobots_.end(); ++it)
	{
		if (*it != *pImpl_->current_)
		(*it)->DrawNoTexture();
	}
}

const manip_core::RobotI* DrawPostures::Next()
{
	if(pImpl_->drawRobots_.empty())
		return 0;
	++(pImpl_->current_);
	if(pImpl_->current_ == pImpl_->drawRobots_.end())
	{
		--(pImpl_->current_);
		return 0;
	}
	return (*(pImpl_->current_))->GetRobot();
}

const manip_core::RobotI* DrawPostures::Previous()
{
	if(pImpl_->drawRobots_.empty())
		return 0;
	if(pImpl_->current_ == pImpl_->drawRobots_.begin())
	{
		return 0;
	}
	else
	{
		--(pImpl_->current_);
		return (*(pImpl_->current_))->GetRobot();
	}
}

void DrawPostures::DrawOne(bool transparency) const
{
	if(pImpl_->current_ != pImpl_->drawRobots_.end())
	{
		//(*(pImpl_->current_))->ToggleSupportPolygon(true);
		(*(pImpl_->current_))->Draw(transparency);
	}
}

void DrawPostures::Clear()
{
	pImpl_.reset(new DrawPosturePImpl);
}

void DrawPostures::OnPostureCreated(NUMBER time, const RobotI* pRobot)
{
	pImpl_->OnPostureCreated(time, pRobot);
	pImpl_->current_ = pImpl_->drawRobots_.begin();
}

