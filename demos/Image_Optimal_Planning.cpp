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

//#include <ompl/base/spaces/RealVectorStateSpace.h>
//#include <ompl/geometric/SimpleSetup.h>
//#include <ompl/geometric/planners/rrt/RRTstar.h>
//#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/PPM.h>

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

// The supported optimal planners, in alphabetical order
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>


// For boost program options
#include <boost/program_options.hpp>
// For string comparison (boost::iequals)
#include <boost/algorithm/string.hpp>
// For std::make_shared
#include <memory>

#include <fstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// An enum of supported optimal planners, alphabetical order
enum optimalPlanner
{
	PLANNER_BFMTSTAR,
	PLANNER_BITSTAR,
	PLANNER_CFOREST,
	PLANNER_FMTSTAR,
	PLANNER_INF_RRTSTAR,
	PLANNER_PRMSTAR,
	PLANNER_RRTSTAR,
	PLANNER_SORRTSTAR,
};

// An enum of the supported optimization objectives, alphabetical order
enum planningObjective
{
	OBJECTIVE_PATHCLEARANCE,
	OBJECTIVE_PATHLENGTH,
	OBJECTIVE_THRESHOLDPATHLENGTH,
	OBJECTIVE_WEIGHTEDCOMBO
};

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getBalancedObjective1(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getBalancedObjective2(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si);

ob::PlannerPtr allocatePlanner(ob::SpaceInformationPtr si, optimalPlanner plannerType)
{
	switch (plannerType)
	{
	case PLANNER_BFMTSTAR:
	{
		return std::make_shared<og::BFMT>(si);
		break;
	}
	case PLANNER_BITSTAR:
	{
		return std::make_shared<og::BITstar>(si);
		break;
	}
	case PLANNER_CFOREST:
	{
		return std::make_shared<og::CForest>(si);
		break;
	}
	case PLANNER_FMTSTAR:
	{
		return std::make_shared<og::FMT>(si);
		break;
	}
	case PLANNER_INF_RRTSTAR:
	{
		og::InformedRRTstar *irrt = new og::InformedRRTstar(si);
		irrt->setRange(10.0);
		irrt->setGoalBias(0.05);
		return ob::PlannerPtr(irrt);
		/*return std::make_shared<og::InformedRRTstar>(si);*/
		break;
	}
	case PLANNER_PRMSTAR:
	{
		return std::make_shared<og::PRMstar>(si);
		break;
	}
	case PLANNER_RRTSTAR:
	{
		og::RRTstar *rrt = new og::RRTstar(si);
		rrt->setRange(10.0);
		rrt->setGoalBias(0.05);
		return ob::PlannerPtr(rrt);
		/*return std::make_shared<og::RRTstar>(si);*/
		break;
	}
	case PLANNER_SORRTSTAR:
	{
		return std::make_shared<og::SORRTstar>(si);
		break;
	}
	default:
	{
		OMPL_ERROR("Planner-type enum is not implemented in allocation function.");
		return ob::PlannerPtr(); // Address compiler warning re: no return value.
		break;
	}
	}
}

ob::OptimizationObjectivePtr allocateObjective(const ob::SpaceInformationPtr& si, planningObjective objectiveType)
{
	switch (objectiveType)
	{
	case OBJECTIVE_PATHCLEARANCE:
		return getClearanceObjective(si);
		break;
	case OBJECTIVE_PATHLENGTH:
		return getPathLengthObjective(si);
		break;
	case OBJECTIVE_THRESHOLDPATHLENGTH:
		return getThresholdPathLengthObj(si);
		break;
	case OBJECTIVE_WEIGHTEDCOMBO:
		return getBalancedObjective1(si);
		break;
	default:
		OMPL_ERROR("Optimization-objective enum is not implemented in allocation function.");
		return ob::OptimizationObjectivePtr();
		break;
	}
}

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

	void plan(unsigned int start_row, unsigned int start_col, unsigned int goal_row, unsigned int goal_col, double runTime, optimalPlanner plannerType, planningObjective objectiveType, const std::string& outputFile)
	{
		// Dubins state space
		ob::StateSpacePtr space(std::make_shared<ob::ReedsSheppStateSpace>());
		double turningRadius = 10.0;
		space = std::make_shared<ob::DubinsStateSpace>(turningRadius, true);

		ob::RealVectorBounds bounds(2);
		bounds.setLow(0);
		bounds.setHigh(0, ppm_.getWidth());
		bounds.setHigh(1, ppm_.getHeight());
		space->as<ob::SE2StateSpace>()->setBounds(bounds);

		ob::ScopedState<> start(space), goal(space);
		start[0] = start_row;
		start[1] = start_col;
		start[2] = 0;
		goal[0] = goal_row;
		goal[1] = goal_col;
		goal[2] = 0;

		// Construct a space information instance for this state space
		auto si(std::make_shared<ob::SpaceInformation>(space));

		// Set the object used to check which states in the space are valid
		si->setStateValidityChecker([this](const ob::State *state) { return isStateValid(state); });
		/*si->setStateValidityCheckingResolution(0.001);*/
		si->setup();
		// Create a problem instance
		auto pdef(std::make_shared<ob::ProblemDefinition>(si));

		// Set the start and goal states
		pdef->setStartAndGoalStates(start, goal);
		std::cout << "start to plan!" << std::endl;
		// Create the optimization objective specified by our command-line argument.
		// This helper function is simply a switch statement.
		pdef->setOptimizationObjective(allocateObjective(si, objectiveType));

		// Construct the optimal planner specified by our command line argument.
		// This helper function is simply a switch statement.	
		ob::PlannerPtr optimizingPlanner = allocatePlanner(si, plannerType);

		// Set the problem instance for our planner to solve
		optimizingPlanner->setProblemDefinition(pdef);
		optimizingPlanner->setup();

		// attempt to solve the planning problem in the given runtime
		bool solved = optimizingPlanner->solve(runTime);

		if (solved)
		{
			// Output the length of the path found
			std::cout
				<< optimizingPlanner->getName()
				<< " found a solution of length "
				<< pdef->getSolutionPath()->length()
				<< " with an optimization objective value of "
				<< pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()) << std::endl;
			/*pdef->getSolutionPath()->print(std::cout);*/
			/*pdef->getSolutionPath()->as<og::PathGeometric>;*/

			// If a filename was specified, output the path as a matrix to
			// that file for visualization
			if (!outputFile.empty())
			{
				std::shared_ptr<og::PathGeometric> path = std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
				path->interpolate();
				//path->printAsMatrix(std::cout);
				std::ofstream outFile(outputFile.c_str());
				path->printAsMatrix(outFile);
				outFile.close();
			}
		}
		else
			std::cout << "No solution found." << std::endl;
	}

	void recordSolution()
	{
		std::shared_ptr<og::PathGeometric> p = std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
		p->interpolate();
		for (std::size_t i = 0; i < p->getStateCount(); ++i)
		{
			const int w = std::min(maxWidth_, (int)p->getState(i)->as<ob::SE2StateSpace::StateType>()->getX());
			const int h = std::min(maxHeight_, (int)p->getState(i)->as<ob::SE2StateSpace::StateType>()->getY());
			std::cout << p->getState(i)->as<ob::SE2StateSpace::StateType>()->getX() << " " << p->getState(i)->as<ob::SE2StateSpace::StateType>()->getY() << std::endl;
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
		const int w = std::min((int)state->as<ob::SE2StateSpace::StateType>()->getX(), maxWidth_);
		const int h = std::min((int)state->as<ob::SE2StateSpace::StateType>()->getY(), maxHeight_);

		const ompl::PPM::Color &c = ppm_.getPixel(h, w);
		return c.red > 127 && c.green > 127 && c.blue > 127;
	}

	ob::ProblemDefinitionPtr pdef;
	int maxWidth_;
	int maxHeight_;
	ompl::PPM ppm_;

};

int main()
{
	Plane2DEnvironment env("C:/Users/TJZXH/Desktop/test.ppm");
	double runTime = 100.0;
	optimalPlanner plannerType = PLANNER_BFMTSTAR;
	planningObjective objectiveType = OBJECTIVE_PATHLENGTH;
	const std::string& outputFile = "J:/results.txt";
	env.plan(43, 643, 520, 3, runTime, plannerType, objectiveType, outputFile);
	env.recordSolution();
	env.save("C:/Users/TJZXH/Desktop/result_demo.ppm");

	return 0;
}

/** Returns a structure representing the optimization objective to use
for optimal motion planning. This method returns an objective
which attempts to minimize the length in configuration space of
computed paths. */
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
	return std::make_shared<ob::PathLengthOptimizationObjective>(si);
}

/** Returns an optimization objective which attempts to minimize path
length that is satisfied when a path of length shorter than 1.51
is found. */
ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
	auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
	obj->setCostThreshold(ob::Cost(1.51));
	return obj;
}

/** Defines an optimization objective which attempts to steer the
robot away from obstacles. To formulate this objective as a
minimization of path cost, we can define the cost of a path as a
summation of the costs of each of the states along the path, where
each state cost is a function of that state's clearance from
obstacles.

The class StateCostIntegralObjective represents objectives as
summations of state costs, just like we require. All we need to do
then is inherit from that base class and define our specific state
cost function by overriding the stateCost() method.
*/
class ClearanceObjective : public ob::StateCostIntegralObjective
{
public:
	ClearanceObjective(const ob::SpaceInformationPtr& si) :
		ob::StateCostIntegralObjective(si, true)
	{
	}

	// Our requirement is to maximize path clearance from obstacles,
	// but we want to represent the objective as a path cost
	// minimization. Therefore, we set each state's cost to be the
	// reciprocal of its clearance, so that as state clearance
	// increases, the state cost decreases.
	ob::Cost stateCost(const ob::State* s) const override
	{
		return ob::Cost(1 / (si_->getStateValidityChecker()->clearance(s) +
			std::numeric_limits<double>::min()));
	}
};

/** Return an optimization objective which attempts to steer the robot
away from obstacles. */
ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si)
{
	return std::make_shared<ClearanceObjective>(si);
}

/** Create an optimization objective which attempts to optimize both
path length and clearance. We do this by defining our individual
objectives, then adding them to a MultiOptimizationObjective
object. This results in an optimization objective where path cost
is equivalent to adding up each of the individual objectives' path
costs.

When adding objectives, we can also optionally specify each
objective's weighting factor to signify how important it is in
optimal planning. If no weight is specified, the weight defaults to
1.0.
*/
ob::OptimizationObjectivePtr getBalancedObjective1(const ob::SpaceInformationPtr& si)
{
	auto lengthObj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
	auto clearObj(std::make_shared<ClearanceObjective>(si));
	auto opt(std::make_shared<ob::MultiOptimizationObjective>(si));
	opt->addObjective(lengthObj, 10.0);
	opt->addObjective(clearObj, 1.0);

	return ob::OptimizationObjectivePtr(opt);
}

/** Create an optimization objective equivalent to the one returned by
getBalancedObjective1(), but use an alternate syntax.
*/
ob::OptimizationObjectivePtr getBalancedObjective2(const ob::SpaceInformationPtr& si)
{
	auto lengthObj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
	auto clearObj(std::make_shared<ClearanceObjective>(si));

	return 10.0*lengthObj + clearObj;
}

/** Create an optimization objective for minimizing path length, and
specify a cost-to-go heuristic suitable for this optimal planning
problem. */
ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
{
	auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
	obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
	return obj;
}