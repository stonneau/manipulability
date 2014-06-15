#include "SpiderGait.h"

SpiderGait::SpiderGait()
	: elapsed_(0)
	, phase_(0)
	, totalelapsed_(0)
	, phaseLength_(600)
{
	// NOTHING
}

SpiderGait::~SpiderGait()
{
	// NOTHINGsettr
}

void SpiderGait::Update(unsigned long time)
{
	elapsed_ += time - totalelapsed_;
	totalelapsed_ = time;
	if(elapsed_ > phaseLength_)
	{
		elapsed_ = elapsed_ % phaseLength_;
		phase_ = (phase_ + 1) % 2;
	}
}

int SpiderGait::Phase() const
{
	return phase_;
}

bool SpiderGait::MidPhase() const
{
	return elapsed_ > phaseLength_ / 10;
}
