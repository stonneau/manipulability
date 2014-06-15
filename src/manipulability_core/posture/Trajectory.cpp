#include "Trajectory.h"

using namespace matrices;

Trajectory::Trajectory()
{
	// NOTHING
}

Trajectory::Trajectory(const T_TimePositions& timePositions)
	:  timePositions_(timePositions)
{
	//NOTHING
}

Trajectory::~Trajectory()
{
	// NOTHING
}

bool Trajectory::AddCheckPoint(float time, const Vector3& position)
{
	bool inserted = false;
	for(T_TimePositionsIT it = timePositions_.begin(); it!= timePositions_.end(); ++it)
	{
		float cTime = (*it).first;
		if(cTime == time)
		{
			return false;
		}
		else if(cTime > time)
		{
			timePositions_.insert(it, P_TimePosition(time, position));
			inserted = true;
			return true;
		}
	}
	if(!inserted)
	{
		if(!timePositions_.empty())
		{
			float lastime = timePositions_.back().first;
			// interpolate trajectory as straight line
			// TODO Meilleure �chelle
			// compute speed
			Vector3 oldPosition = timePositions_.back().second;
			Vector3 speed = (position - oldPosition) / (time - lastime);
			/*for (float i = lastime + 0.1f; i < time; i = i + 0.1f)
			{
				Vector3 newVal = oldPosition + speed * ( i);
				timePositions_.push_back(P_TimePosition(i, oldPosition + speed * ( i - lastime) ));
			}*/
		}
		timePositions_.push_back(P_TimePosition(time, position));
		return true;
	}
	return false;
}

bool Trajectory::AddWayPoint(T_TimePositionsIT& position)
{
	T_TimePositionsIT next = position;
	++next;
	assert(position != timePositions_.end());
	Vector3 newWaypoint = ((*position).second + (*next).second) / 2.f;
	NUMBER distance =  ((*next).second  - newWaypoint).norm();
	if(distance < 0.2)
	{
		return false;
	}
	else
	{
		float newTime = ((*position).first + (*next).first) / 2.f;
		//The vector is extended by inserting new elements before the element at position.
		position = timePositions_.insert(next, P_TimePosition(newTime, newWaypoint));
		return true;
	}
}

bool Trajectory::ReplaceWayPoint(T_TimePositionsIT& position, const Vector3& newWaypoint)
{
	position->second = newWaypoint;
	return true;
}


const Trajectory::T_TimePositions& Trajectory::GetTimePositions() const
{
	return timePositions_;
}

Trajectory::T_TimePositions& Trajectory::GetEditableTimePositions()
{
	return timePositions_;
}

void Trajectory::Reset()
{
	timePositions_.clear();
}

