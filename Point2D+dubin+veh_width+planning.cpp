/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/util/PPM.h>
#include <string>
#include <ompl/config.h>
#include <../tests/resources/config.h>

#include <boost/filesystem.hpp>
#include <iostream>

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
// For ompl::msg::setLogLevel
#include "ompl/util/Console.h"


// For boost program options
#include <boost/program_options.hpp>
// For string comparison (boost::iequals)
#include <boost/algorithm/string.hpp>
// For std::make_shared
#include <memory>
#include <cmath>
#include <fstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class Plane2DEnvironment
{
public:

	Plane2DEnvironment(const char *ppm_file)
	{
		bool ok = false;
		try
		{
			ppm_.loadFile(ppm_file);
			ok = true;
		}
		catch (ompl::Exception &ex)
		{
			OMPL_ERROR("Unable to load %s.\n%s", ppm_file, ex.what());
		}
		if (ok)
		{
			maxWidth_ = ppm_.getWidth() - 1;
			maxHeight_ = ppm_.getHeight() - 1;

		}
	}

	void plan(unsigned int start_row, unsigned int start_col, double start_yaw, unsigned int goal_row, unsigned int goal_col, double goal_yaw, double width, double runtime, const std::string& outputFile)
	{
		// Dubins state space
		ob::StateSpacePtr space(std::make_shared<ob::ReedsSheppStateSpace>());
		double turningRadius = 10.0;
		veh_width = width;
		std::cout << veh_width << std::endl;
		space = std::make_shared<ob::DubinsStateSpace>(turningRadius, false);

		ob::RealVectorBounds bounds(2);
		bounds.setLow(0);
		bounds.setHigh(0, ppm_.getWidth());
		bounds.setHigh(1, ppm_.getHeight());
		space->as<ob::SE2StateSpace>()->setBounds(bounds);

		// define a simple setup class
		ss = std::make_shared<og::SimpleSetup>(space);
		ob::ScopedState<> start(space), goal(space);
		start[0] = start_row;
		start[1] = start_col;
		start[2] = start_yaw;
		goal[0] = goal_row;
		goal[1] = goal_col;
		goal[2] = goal_yaw;
		// set state validity checking for this space
		ss->setStateValidityChecker([this](const ob::State *state) { return isStateValid(state); });

		ss->setStartAndGoalStates(start, goal);
		og::RRT *rrt = new og::RRT(ss->getSpaceInformation());
		rrt->setRange(10.0);
		rrt->setGoalBias(0.05);
		ss->setPlanner(ob::PlannerPtr(rrt));
		// this call is optional, but we put it in to get more output information
		space->setup();
		ss->getSpaceInformation()->setStateValidityCheckingResolution(0.001);

		// attempt to solve the problem within 30 seconds of planning time
		ob::PlannerStatus solved = ss->solve(runtime);

		if (solved)
		{
			std::cout << "Found solution:" << std::endl;
			ss->simplifySolution();
			og::PathGeometric &p = ss->getSolutionPath();
			ss->getPathSimplifier()->simplifyMax(p);
			ss->getPathSimplifier()->smoothBSpline(p);

			//path.printAsMatrix(std::cout);
		}
		else
			std::cout << "No solution found" << std::endl;
	}

	void recordSolution()
	{
		og::PathGeometric &p = ss->getSolutionPath();
		p.interpolate();
		for (std::size_t i = 0; i < p.getStateCount(); ++i)
		{
			const int w = std::min(maxWidth_, (int)p.getState(i)->as<ob::SE2StateSpace::StateType>()->getX());
			const int h = std::min(maxHeight_, (int)p.getState(i)->as<ob::SE2StateSpace::StateType>()->getY());
			std::cout << p.getState(i)->as<ob::SE2StateSpace::StateType>()->getX() << " " << p.getState(i)->as<ob::SE2StateSpace::StateType>()->getY() << std::endl;
			ompl::PPM::Color &c = ppm_.getPixel(h, w);
			c.red = 255;
			c.green = 0;
			c.blue = 0;
		}
	}

	void save(const char *filename)
	{
		ppm_.saveFile(filename);
	}

private:

	bool isStateValid(const ob::State *state) const
	{
		/*const int w1 = std::min((int)state->as<ob::SE2StateSpace::StateType>()->getX(), maxWidth_);
		const int h1 = std::min((int)state->as<ob::SE2StateSpace::StateType>()->getY(), maxHeight_);*/
		const double x = state->as<ob::SE2StateSpace::StateType>()->getX();
		const double y = state->as<ob::SE2StateSpace::StateType>()->getY();
		const double yaw = state->as<ob::SE2StateSpace::StateType>()->getYaw();
		const double left_x = x + veh_width * sin(yaw);
		const double left_y = y - veh_width * cos(yaw);
		const double right_x = x - veh_width * sin(yaw);
		const double right_y = y + veh_width * cos(yaw);
		const ompl::PPM::Color &left_c = ppm_.getPixel(round(left_y), round(left_x));
		const ompl::PPM::Color &right_c = ppm_.getPixel(round(right_y), round(right_x));
		return left_c.red > 127 && left_c.green > 127 && left_c.blue > 127 && right_c.red > 127 && right_c.green > 127 && right_c.blue > 127;
	}

	og::SimpleSetupPtr ss;
	int maxWidth_;
	int maxHeight_;
	ompl::PPM ppm_;
	double veh_width;

};

int main()
{	
	int length = 20;
	for (int i = 0; i < length; i++)
	{
		Plane2DEnvironment env("C:/Users/TJZXH/Desktop/test.ppm");
		double runTime = 100.0;
		const std::string& outputFile = "J:/results.txt";
		env.plan(85, 626, -3.14/4, 521, 12, -3.14/2, 10.0, runTime, outputFile);
		env.recordSolution();
		std::string env_path = "C:/Users/TJZXH/Desktop/OMPL-LEARN/veh_width_results/veh_width_demo" + std::to_string(i) + ".ppm";
		env.save(env_path.c_str());
	}	
	return 0;
}