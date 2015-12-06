//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
/*
	sfPAM is a module written for SteerSuite by Cyan Kuo. The work is based off of the sfAI module and 
	implements the steering model described in I. Karamouzas et al., "A Predictive Collision Avoidance 
	Model for Pedestrian Simulation," in Motion in Games, A. Egges, R. Geraerts and M. Overmars, Eds.,
	vol. 5884, Springer, 2009, pp. 41-52.
*/



#include "SocialForcesAgent.h"
#include "SocialForcesAIModule.h"
#include "SocialForces_Parameters.h"
// #include <math.h>

// #include "util/Geometry.h"

/// @file SocialForcesAgent.cpp
/// @brief Implements the SocialForcesAgent class.

#undef min
#undef max

#define AGENT_MASS 1.0f

using namespace Util;
using namespace SocialForcesGlobals;
using namespace SteerLib;

// #define _DEBUG_ENTROPY 1

SocialForcesAgent::SocialForcesAgent()
{
	_SocialForcesParams.sf_acceleration = sf_acceleration;
	_SocialForcesParams.sf_personal_space_threshold = sf_personal_space_threshold;
	_SocialForcesParams.sf_agent_repulsion_importance = sf_agent_repulsion_importance;
	_SocialForcesParams.sf_query_radius = sf_query_radius;
	_SocialForcesParams.sf_body_force = sf_body_force;
	_SocialForcesParams.sf_agent_body_force = sf_agent_body_force;
	_SocialForcesParams.sf_sliding_friction_force = sf_sliding_friction_force;
	_SocialForcesParams.sf_agent_b = sf_agent_b;
	_SocialForcesParams.sf_agent_a = sf_agent_a;
	_SocialForcesParams.sf_wall_b = sf_wall_b;
	_SocialForcesParams.sf_wall_a = sf_wall_a;
	_SocialForcesParams.sf_max_speed = sf_max_speed;

	_enabled = false;
}

SocialForcesAgent::~SocialForcesAgent()
{
	// std::cout << this << " is being deleted" << std::endl;
	/*
	if (this->enabled())
	{
		Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
		// getSimulationEngine()->getSpatialDatabase()->removeObject( this, bounds);
	}*/
	// std::cout << "Someone is removing an agent " << std::endl;
}

SteerLib::EngineInterface * SocialForcesAgent::getSimulationEngine()
{
	return _gEngine;
}

void SocialForcesAgent::setParameters(Behaviour behave)
{
	this->_SocialForcesParams.setParameters(behave);
}

void SocialForcesAgent::disable()
{
	// DO nothing for now
	// if we tried to disable a second time, most likely we accidentally ignored that it was disabled, and should catch that error.
	assert(_enabled==true);


	//  1. remove from database
	AxisAlignedBox b = AxisAlignedBox(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	getSimulationEngine()->getSpatialDatabase()->removeObject(dynamic_cast<SpatialDatabaseItemPtr>(this), b);

	//  2. set enabled = false
	_enabled = false;


}

void SocialForcesAgent::reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo)
{
	// compute the "old" bounding box of the agent before it is reset.  its OK that it will be invalid if the agent was previously disabled
	// because the value is not used in that case.
	_waypoints.clear();
	_midTermPath.clear();

	Util::AxisAlignedBox oldBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.5f, _position.z-_radius, _position.z+_radius);


	// initialize the agent based on the initial conditions
	_position = initialConditions.position;
	_forward = normalize(initialConditions.direction);
	_radius = initialConditions.radius;
	_velocity = initialConditions.speed * _forward;
	// std::cout << "inital colour of agent " << initialConditions.color << std::endl;
	if ( initialConditions.colorSet == true )
	{
		this->_color = initialConditions.color;
	}
	else
	{
		this->_color = Util::gBlue;
	}

	// compute the "new" bounding box of the agent
	Util::AxisAlignedBox newBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.5f, _position.z-_radius, _position.z+_radius);

	if (!_enabled) {
		// if the agent was not enabled, then it does not already exist in the database, so add it.
		// std::cout
		getSimulationEngine()->getSpatialDatabase()->addObject( dynamic_cast<SpatialDatabaseItemPtr>(this), newBounds);
	}
	else {
		// if the agent was enabled, then the agent already existed in the database, so update it instead of adding it.
		getSimulationEngine()->getSpatialDatabase()->updateObject( dynamic_cast<SpatialDatabaseItemPtr>(this), oldBounds, newBounds);
	}

	_enabled = true;

	if (initialConditions.goals.size() == 0)
	{
		throw Util::GenericException("No goals were specified!\n");
	}

	while (!_goalQueue.empty())
	{
		_goalQueue.pop();
	}

	// iterate over the sequence of goals specified by the initial conditions.
	for (unsigned int i=0; i<initialConditions.goals.size(); i++) {
		if (initialConditions.goals[i].goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET ||
				initialConditions.goals[i].goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL)
		{
			if (initialConditions.goals[i].targetIsRandom)
			{
				// if the goal is random, we must randomly generate the goal.
				// std::cout << "assigning random goal" << std::endl;
				SteerLib::AgentGoalInfo _goal;
				_goal.targetLocation = getSimulationEngine()->getSpatialDatabase()->randomPositionWithoutCollisions(1.0f, true);
				_goalQueue.push(_goal);
				_currentGoal.targetLocation = _goal.targetLocation;
			}
			else
			{
				_goalQueue.push(initialConditions.goals[i]);
			}
		}
		else {
			throw Util::GenericException("Unsupported goal type; SocialForcesAgent only supports GOAL_TYPE_SEEK_STATIC_TARGET and GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL.");
		}
	}

	runLongTermPlanning(_goalQueue.front().targetLocation, dont_plan);

	// std::cout << "first waypoint: " << _waypoints.front() << " agents position: " << position() << std::endl;
	/*
	 * Must make sure that _waypoints.front() != position(). If they are equal the agent will crash.
	 * And that _waypoints is not empty
	 */
	Util::Vector goalDirection;
	if ( !_midTermPath.empty() )
	{
		this->updateLocalTarget();
		goalDirection = normalize( this->_currentLocalTarget - position());
	}
	else
	{
		goalDirection = normalize( _goalQueue.front().targetLocation - position());
	}

	_prefVelocity =
			(
				(
					(
						Util::Vector(goalDirection.x, 0.0f, goalDirection.z) *
						PREFERRED_SPEED
					)
				- velocity()
				)
				/
				_SocialForcesParams.sf_acceleration
			)
			*
			MASS;

	// _velocity = _prefVelocity;
#ifdef _DEBUG_ENTROPY
	std::cout << "goal direction is: " << goalDirection << " prefvelocity is: " << prefVelocity_ <<
			" and current velocity is: " << velocity_ << std::endl;
#endif


	assert(_forward.length()!=0.0f);
	assert(_goalQueue.size() != 0);
	assert(_radius != 0.0f);
}


void SocialForcesAgent::calcNextStep(float dt)
{

}

float timeToIntersect(const Util::Point a_pos, const Util::Point b_pos, const Util::Vector v_diff, float ab_rad) 
	{
		// intersection time is based off of the original C++ implementation by I. Karamouzas
		float time;
		Util::Vector pos_diff = b_pos - a_pos;
		float a = v_diff*v_diff;
		float b = pos_diff*v_diff;	
		float c = pos_diff*pos_diff - ab_rad*ab_rad;
   		float discr = b*b - a*c;
		if (discr > 0.0f)
		{			
			time = (b - sqrtf(discr)) / a;
			if (time < 0) 
			   time = FLT_MAX;	
		}
		else
			time = FLT_MAX;		
		
		return time;
	}

Util::Vector SocialForcesAgent::calcAgentRepulsionForce(float dt, Util::Vector wall_repulsion)
{
	// The return value
	Util::Vector agent_repulsion_force = Util::Vector(0,0,0);
	std::pair<Util::Vector,Util::Vector> agent_goalFInfo = calcGoalForce(dt,_goalQueue.front());

	Util::Vector agent_desiredvelocity;

	// Calculate the desired velocity and estimated position of agent A
	agent_desiredvelocity = agent_goalFInfo.first + wall_repulsion;

	

	// Obtain other agents within agent A's query radius
	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
	getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors,
		_position.x-(this->_radius + _SocialForcesParams.sf_query_radius),
		_position.x+(this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z-(this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z+(this->_radius + _SocialForcesParams.sf_query_radius),
				(this));

	SteerLib::AgentInterface * tmp_agent;
	std::multimap<float,SteerLib::AgentInterface *> other_agents;
	int agent_workingmemory = 0;
	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin();  
		neighbour != _neighbors.end();  neighbour++)
	{
		if ( (*neighbour)->isAgent() )
		{
			tmp_agent = dynamic_cast<SteerLib::AgentInterface *>(*neighbour);
			
			if ( id() != tmp_agent->id() )
			{
				// if the other guy is getting too close...
				if ( tmp_agent->computePenetration(this->position(), this->radius()) > 
					PERSONAL_SPACE_THRESHOLD*PERSONAL_SPACE_THRESHOLD )	
				{
				// make getting away from him the top priority -- insert time to collision and agent
				// in one of the top places in the map
				other_agents.insert(std::make_pair(0.0f, tmp_agent));
				
				}
				else
				{
				// initialize variables that will be used to compute repulsive forces
				Util::Vector relativeDir = normalize(tmp_agent->position() - this->position());
				// I have literally no idea what this next 'if' statement does
				// but it's used in Karamouzas' code and seems to make things work better
				if ( ( relativeDir * normalize(this->velocity()) ) < 
					cosf( (0.5f * M_PI * 200.f) / 180.f) ) continue; 
				float collisiontime = timeToIntersect(this->position(), tmp_agent->position(), 
					agent_desiredvelocity - tmp_agent->velocity() , 
					PERSONAL_SPACE_THRESHOLD + tmp_agent->radius());

				other_agents.insert(std::make_pair(collisiontime, tmp_agent));
				}
			}

		}

	}
	
	// iterate over agents by collision time and apply the goal forces
	for (std::multimap<float,SteerLib::AgentInterface *>::iterator it_other = other_agents.begin(); 
		((it_other != other_agents.end()) && (agent_workingmemory < AGENT_TRACKING)); 
		it_other++)

	{
		// nab the collision time, establish magnitude
		SteerLib::AgentInterface * other = it_other->second;
		float ct = it_other->first;
		Util::Vector away_direction = Util::Vector(0,0,0);
		float mag = 0.0;

		// obtain the distance to collision and the direction away
		Util::Point otherCollPos = other->position() + ct * other->velocity();
		Util::Point agent_CollPos = this->position() + ct * agent_desiredvelocity;
		away_direction = (agent_CollPos - otherCollPos)/(agent_CollPos - otherCollPos).length();
		float distToCollision = ((agent_CollPos - this->position()).length()) + 
			((agent_CollPos - otherCollPos).length() - this->radius() - other->radius());
		float D = std::max(agent_desiredvelocity.length() * ct + distToCollision, 0.0001f); // this differs from the paper
		// magnitude of force
		if ( D < D_MIN ) {
			mag = AGGRESSION * D_MIN / D;
		} else if ( D < D_MID ) {
			mag = AGGRESSION;
		} else if ( D < D_MAX ) {
			mag = AGGRESSION * (D_MAX - D)/(D_MAX - D_MID);
		} else {
			continue;	// magnitude is zero
		}
		// calculate evasive force, apply it and see if this causes collisions with other agents
		agent_workingmemory += 1;
		mag *= powf((ct == 0.0f?1.0f:AGENT_REPULSION_IMPORTANCE), agent_workingmemory);
		agent_repulsion_force += mag * away_direction;
		// use this value to determine whether or not other collisions occur
		agent_desiredvelocity += agent_repulsion_force;
	}


	return agent_repulsion_force;
}



std::pair<float, Util::Point> minimum_distance(Util::Point l1, Util::Point l2, Util::Point p)
{
  // Return minimum distance between line segment vw and point p
  float lSq = (l1 - l2).lengthSquared();  // i.e. |l2-l1|^2 -  avoid a sqrt
  if (lSq == 0.0)
	  return std::make_pair((p - l2).length(),l1 );   // l1 == l2 case

  // Consider the line extending the segment, parameterized as l1 + t (l2 - l1).
  // We find projection of point p onto the line.
  // It falls where t = [(p-l1) . (l2-l1)] / |l2-l1|^2
  const float t = dot(p - l1, l2 - l1) / lSq;
  if (t < 0.0)
  {
	  return std::make_pair((p - l1).length(), l1);       // Beyond the 'l1' end of the segment
  }
  else if (t > 1.0)
  {
	  return std::make_pair((p - l2).length(), l2);  // Beyond the 'l2' end of the segment
  }
  const Util::Point projection = l1 + t * (l2 - l1);  // Projection falls on the segment
  return std::make_pair((p - projection).length(), projection) ;
}

Util::Vector SocialForcesAgent::calcRepulsionForce(float dt)
{
#ifdef _DEBUG_
	std::cout << "wall repulsion; " << calcWallRepulsionForce(dt) << " agent repulsion " <<
			(_SocialForcesParams.sf_agent_repulsion_importance * calcAgentRepulsionForce(dt)) << std::endl;
#endif
	Util::Vector wall_repulsion = calcWallRepulsionForce(dt);
	return wall_repulsion + (calcAgentRepulsionForce(dt, wall_repulsion));
}


Util::Vector SocialForcesAgent::calcWallRepulsionForce(float dt)
{
	// this is the return value: the repulsion force of the wall
	Util::Vector wall_repulsion_force = Util::Vector(0,0,0);

	// get all neighbours in range
	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
		getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors,
				_position.x-(this->_radius + _SocialForcesParams.sf_query_radius),
				_position.x+(this->_radius + _SocialForcesParams.sf_query_radius),
				_position.z-(this->_radius + _SocialForcesParams.sf_query_radius),
				_position.z+(this->_radius + _SocialForcesParams.sf_query_radius),
				dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	SteerLib::ObstacleInterface * tmp_ob;

	// for all neighbours that are walls
	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin();  
		neighbour != _neighbors.end();  neighbour++)
	{
		if ( !(*neighbour)->isAgent() )
		{

			// cast object as obstacle, calculate normal, obtain closest point wall and its distance
			tmp_ob = dynamic_cast<SteerLib::ObstacleInterface *>(*neighbour);
			Util::Vector wall_normal = calcWallNormal(tmp_ob);
			std::pair<Util::Point,Util::Point> line = calcWallPointsFromNormal(tmp_ob, wall_normal);
			std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position());

			// as per Karamouzas paper: if shortest distance to wall minus radius is smaller than
			// preferred distance to wall, calculate repulsion force
			if (powf(min_stuff.first, 2.0) < powf(PREFERRED_WALL_DISTANCE, 2.0))
			{
				// basically scales back the force if the collision isn't imminent
				if (min_stuff.first > 0.0)
					wall_normal /= min_stuff.first;
				 float distMinRadius = (min_stuff.first - radius()) < 0.0001f? 0.0001f : min_stuff.first - radius();

			// as per Karamouzas paper
			wall_repulsion_force +=
				wall_normal * (PREFERRED_WALL_DISTANCE + (radius() - min_stuff.first)) / powf( (distMinRadius) , 0.001f);
			}
		}
		else
		{
			continue;
		}

	}
	return wall_repulsion_force;
}

std::pair<Util::Point, Util::Point> SocialForcesAgent::calcWallPointsFromNormal(SteerLib::ObstacleInterface* obs, Util::Vector normal)
{
	Util::AxisAlignedBox box = obs->getBounds();
	if ( normal.z == 1)
	{
		return std::make_pair(Util::Point(box.xmin,0,box.zmax), Util::Point(box.xmax,0,box.zmax));
		// Ended here;
	}
	else if ( normal.z == -1 )
	{
		return std::make_pair(Util::Point(box.xmin,0,box.zmin), Util::Point(box.xmax,0,box.zmin));
	}
	else if ( normal.x == 1)
	{
		return std::make_pair(Util::Point(box.xmax,0,box.zmin), Util::Point(box.xmax,0,box.zmax));
	}
	else // normal.x == -1
	{
		return std::make_pair(Util::Point(box.xmin,0,box.zmin), Util::Point(box.xmin,0,box.zmax));
	}
}

/**
 * Basically What side of the obstacle is the agent on use that as the normal
 * DOES NOT SUPPORT non-axis-aligned boxes
 *
 *
 * 			   \		   /
 * 				\		  /
 * 				 \	 a	 /
 *				  \		/
 * 					 _
 * 			a		| |       a
 * 					 -
 * 				  /     \
 * 				 /   a   \
 * 				/	      \
 * 			   /	       \
 *
 *
 */
Util::Vector SocialForcesAgent::calcWallNormal(SteerLib::ObstacleInterface* obs)
{
	Util::AxisAlignedBox box = obs->getBounds();
	if ( position().x > box.xmax )
	{
		if ( position().z > box.zmax)
		{
			if ( abs(position().z - box.zmax ) >
				abs( position().x - box.xmax) )
			{
				return Util::Vector(0, 0, 1);
			}
			else
			{
				return Util::Vector(1, 0, 0);
			}

		}
		else if ( position().z < box.zmin )
		{
			if ( abs(position().z - box.zmin ) >
				abs( position().x - box.xmax) )
			{
				return Util::Vector(0, 0, -1);
			}
			else
			{
				return Util::Vector(1, 0, 0);
			}

		}
		else
		{ // in between zmin and zmax
			return Util::Vector(1, 0, 0);
		}

	}
	else if ( position().x < box.xmin )
	{
		if ( position().z > box.zmax )
		{
			if ( abs(position().z - box.zmax ) >
				abs( position().x - box.xmin) )
			{
				return Util::Vector(0, 0, 1);
			}
			else
			{
				return Util::Vector(-1, 0, 0);
			}

		}
		else if ( position().z < box.zmin )
		{
			if ( abs(position().z - box.zmin ) >
				abs( position().x - box.xmin) )
			{
				return Util::Vector(0, 0, -1);
			}
			else
			{
				return Util::Vector(-1, 0, 0);
			}

		}
		else
		{ // in between zmin and zmax
			return Util::Vector(-1, 0, 0);
		}
	}
	else // between xmin and xmax
	{
		if ( position().z > box.zmax )
		{
			return Util::Vector(0, 0, 1);
		}
		else if ( position().z < box.zmin)
		{
			return Util::Vector(0, 0, -1);
		}
		else
		{ // What do we do if the agent is inside the wall?? Lazy Normal
			return calcObsNormal( obs );
		}
	}

}

/**
 * Treats Obstacles as a circle and calculates normal
 */
Util::Vector SocialForcesAgent::calcObsNormal(SteerLib::ObstacleInterface* obs)
{
	Util::AxisAlignedBox box = obs->getBounds();
	Util::Point obs_centre = Util::Point((box.xmax+box.xmin)/2, (box.ymax+box.ymin)/2,
			(box.zmax+box.zmin)/2);
	return normalize(position() - obs_centre);
}

std::pair<Util::Vector, Util::Vector> SocialForcesAgent::calcGoalForce(float dt, SteerLib::AgentGoalInfo goalInfo)
{
	// get goal direction and force (this function is used repeatedly)
	Util::Vector goalDirection;
	// given next goal, calculate the goal direction
	if ( ! _midTermPath.empty() && (!this->hasLineOfSightTo(goalInfo.targetLocation)) )
	{
		// if this is a mid-way goal, calculate accordingly
		if (reachedCurrentWaypoint())
		{
			this->updateMidTermPath();
		}

		this->updateLocalTarget();
		
		goalDirection = normalize(_currentLocalTarget - position());

	}
	else
	{
		// likewise if its the final goal
		goalDirection = normalize(goalInfo.targetLocation - position());
	}

	// calculate goal force
	Util::Vector goalForce = (((goalDirection * PREFERRED_SPEED) - velocity()) /
		(_SocialForcesParams.sf_acceleration/dt)) + 
		velocity();

	// send both back to whoever called
	return std::make_pair(goalDirection,goalForce);
}

void SocialForcesAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{
	Util::AutomaticFunctionProfiler profileThisFunction( &SocialForcesGlobals::gPhaseProfilers->aiProfiler );
	if (!enabled())
	{
		return;
	}

	Util::AxisAlignedBox oldBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);

	SteerLib::AgentGoalInfo goalInfo = _goalQueue.front();
	
	

	std::pair<Util::Vector, Util::Vector> goalDirForce = calcGoalForce(dt, goalInfo);
	Util::Vector goalDirection = goalDirForce.first;
	Util::Vector goalForce = goalDirForce.second;
	Util::Vector repulsionForce = calcRepulsionForce(dt);

// #define _DEBUG_ 1
#ifdef _DEBUG_
	std::cout << "agent" << id() << " repulsion force " << repulsionForce << std::endl;
	std::cout << "agent" << id() << " proximity force " << proximityForce << std::endl;
	std::cout << "agent" << id() << " pref force " << prefForce << std::endl;
#endif
	int alpha=1;
	if ( repulsionForce.length() > 0.0)
	{
		alpha=0;
	}

	// the updated velocity value
	_velocity = (goalForce) + repulsionForce;

	// Karamouzas et al implemented a noise function to account for random actions
	float angle = std::rand() * 2.0f * 3.14 / RAND_MAX;
	float dist = std::rand() * 0.001f / RAND_MAX;
    _velocity += dist* Util::Vector(cos(angle), sin(angle), dist); 


	_velocity = clamp(velocity(), _SocialForcesParams.sf_max_speed);
	_velocity.y=0.0f;

#ifdef _DEBUG_
	std::cout << "agent" << id() << " speed is " << velocity().length() << std::endl;
#endif
	_position = position() + (velocity() * dt);
	// A grid database update should always be done right after the new position of the agent is calculated
	/*
	 * Or when the agent is removed for example its true location will not reflect its location in the grid database.
	 * Not only that but this error will appear random depending on how well the agent lines up with the grid database
	 * boundaries when removed.
	 */

	Util::AxisAlignedBox newBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	getSimulationEngine()->getSpatialDatabase()->updateObject( this, oldBounds, newBounds);

	/*
	 * Now do the conversion from SocialForcesAgent into the SteerSuite coordinates
	 */

	if ((goalInfo.targetLocation - position()).length() < radius()*GOAL_THRESHOLD_MULTIPLIER ||
			(goalInfo.goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL &&
					Util::boxOverlapsCircle2D(goalInfo.targetRegion.xmin, goalInfo.targetRegion.xmax,
							goalInfo.targetRegion.zmin, goalInfo.targetRegion.zmax, this->position(), this->radius())))
	{
		_goalQueue.pop();
		if (_goalQueue.size() != 0)
		{
			// in this case, there are still more goals, so start steering to the next goal.
			goalDirection = _goalQueue.front().targetLocation - _position;
			_prefVelocity = Util::Vector(goalDirection.x, 0.0f, goalDirection.z);
		}
		else
		{
			// in this case, there are no more goals, so disable the agent and remove it from the spatial database.
			disable();
			return;
		}
	}

	// Hear the 2D solution from RVO is converted into the 3D used by SteerSuite
	// _velocity = Vector(velocity().x, 0.0f, velocity().z);
	if ( velocity().lengthSquared() > 0.0 )
	{
		// Only assign forward direction if agent is moving
		// Otherwise keep last forward
		_forward = normalize(_velocity);
	}

}


void SocialForcesAgent::draw()
{
#ifdef ENABLE_GUI
	AgentInterface::draw();
	// if the agent is selected, do some annotations just for demonstration

#ifdef DRAW_COLLISIONS
	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
	getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors, _position.x-(this->_radius * 3), _position.x+(this->_radius * 3),
			_position.z-(this->_radius * 3), _position.z+(this->_radius * 3), dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin();  neighbor != _neighbors.end();  neighbor++)
	{
		if ( (*neighbor)->isAgent() && (*neighbor)->computePenetration(this->position(), this->_radius) > 0.00001f)
		{
			Util::DrawLib::drawStar(
					this->position()
					+
					(
						(
							dynamic_cast<AgentInterface*>(*neighbor)->position()
							-
							this->position()
						)
					/2), Util::Vector(1,0,0), 0.8f, gRed);
			// Util::DrawLib::drawStar(this->position(), Util::Vector(1,0,0), 1.14f, gRed);
		}
	}
#endif
#ifdef DRAW_HISTORIES
	__oldPositions.push_back(position());
	int points = 0;
	float mostPoints = 100.0f;
	while ( __oldPositions.size() > mostPoints )
	{
		__oldPositions.pop_front();
	}
	for (int q = __oldPositions.size()-1 ; q > 0 && __oldPositions.size() > 1; q--)
	{
		DrawLib::drawLineAlpha(__oldPositions.at(q), __oldPositions.at(q-1),gBlack, q/(float)__oldPositions.size());
	}

#endif

#ifdef DRAW_ANNOTATIONS

	for (int i=0; ( _waypoints.size() > 1 ) && (i < (_waypoints.size() - 1)); i++)
	{
		if ( _gEngine->isAgentSelected(this) )
		{
			DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i+1), gYellow);
		}
		else
		{
			//DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i+1), gBlack);
		}
	}

	for (int i=0; i < (_waypoints.size()); i++)
	{
		DrawLib::drawStar(_waypoints.at(i), Util::Vector(1,0,0), 0.34f, gBlue);
	}

	for (int i=0; ( _midTermPath.size() > 1 ) && (i < (_midTermPath.size() - 1)); i++)
	{
		if ( _gEngine->isAgentSelected(this) )
		{
			DrawLib::drawLine(_midTermPath.at(i), _midTermPath.at(i+1), gMagenta);
		}
		else
		{
			// DrawLib::drawLine(_midTermPath.at(i), _midTermPath.at(i+1), gCyan);
		}
	}

	DrawLib::drawLine(position(), this->_currentLocalTarget, gGray10);
	DrawLib::drawStar(this->_currentLocalTarget+Util::Vector(0,0.001,0), Util::Vector(1,0,0), 0.24f, gGray10);

	/*
	// draw normals and closest points on walls
	std::set<SteerLib::ObstacleInterface * > tmp_obs = gEngine->getObstacles();
	for (std::set<SteerLib::ObstacleInterface * >::iterator tmp_o = tmp_obs.begin();  tmp_o != tmp_obs.end();  tmp_o++)
	{
		Util::Vector normal = calcWallNormal( *tmp_o );
		std::pair<Util::Point,Util::Point> line = calcWallPointsFromNormal(* tmp_o, normal);
		Util::Point midpoint = Util::Point((line.first.x+line.second.x)/2, ((line.first.y+line.second.y)/2)+1,
				(line.first.z+line.second.z)/2);
		DrawLib::drawLine(midpoint, midpoint+normal, gGreen);
		// Draw the closes point as well
		std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position());
		DrawLib::drawStar(min_stuff.second, Util::Vector(1,0,0), 0.34f, gGreen);
	}
	*/

#endif

#endif
}