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

/* Author: Jonathan Sobieski, Mark Moll */



#ifndef OMPL_GEOMETRIC_PLANNERS_PDST_PDST_
#define OMPL_GEOMETRIC_PLANNERS_PDST_PDST_

#include "ompl/base/Planner.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/base/PlannerData.h"
#include "ompl/datastructures/BinaryHeap.h"


namespace ompl
{

    namespace geometric
    {

         /**
            @anchor gPDST
            @par Short description
            PDST is a tree-based motion planner that attempts to detect
            the less explored area of the space through the use of a
            binary space partition of a projection of the state space.
            Exploration is biased towards large cells with few path
            segments. Unlike most tree-based planners which expand from
            a randomly select endpoint of a path segment, PDST expands
            from a randomly selected point along a deterministically
            selected path segment. It is important to set the projection
            the algorithm uses (setProjectionEvaluator() function). If
            no projection is set, the planner will attempt to use the
            default projection associated to the state space. An
            exception is thrown if no default projection is available
            either.
            @par External documentation
            A.M. Ladd and L.E. Kavraki, Motion planning in the presence
            of drift, underactuation and discrete system changes, in
            <em>Robotics: Science and Systems I</em>, pp. 233–241, MIT
            Press, June 2005.
            <a href="http://www.roboticsproceedings.org/rss01/p31.pdf">[PDF]</a>
         */

        /// \brief Path-Directed Subdivision Tree
        class PDST : public base::Planner
        {
        public:

            PDST(const base::SpaceInformationPtr &si);

            virtual ~PDST(void);

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);
            virtual void clear(void);
            virtual void setup(void);

            /// Extracts the planner data from the priority queue into data.
            virtual void getPlannerData(base::PlannerData &data) const;

            /// Set the projection evaluator. This class is able to compute the projection of a given state.
            void setProjectionEvaluator(const base::ProjectionEvaluatorPtr &projectionEvaluator)
            {
                projectionEvaluator_ = projectionEvaluator;
            }

            /// Set the projection evaluator (select one from the ones registered with the state space).
            void setProjectionEvaluator(const std::string &name)
            {
                projectionEvaluator_ = si_->getStateSpace()->getProjection(name);
            }

            /// Get the projection evaluator
            const base::ProjectionEvaluatorPtr& getProjectionEvaluator(void) const
            {
                return projectionEvaluator_;
            }

            /// \brief In the process of randomly selecting states in
            /// the state space to attempt to go towards, the
            /// algorithm may in fact choose the actual goal state, if
            /// it knows it, with some probability. This probability
            /// is a real number between 0.0 and 1.0; its value should
            /// usually be around 0.05 and should not be too large. It
            /// is probably a good idea to use the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }
            /// Get the goal bias the planner is using */
            double getGoalBias(void) const
            {
                return goalBias_;
            }

        protected:
            struct Cell;
            struct Motion;

            /// Comparator used to order motions in the priority queue
            struct MotionCompare
            {
                /// returns true if m1 is lower priority than m2
                bool operator() (Motion* p1, Motion* p2) const
                {
                    // lowest priority means highest score
                    return p1->score() < p2->score();
                }
            };

            /// Class representing the tree of motions exploring the state space
            struct Motion
            {
            public:
                Motion(base::State *state, Motion *parent, base::EuclideanProjection::size_type projectionDim,
                    double priority, Cell* cell)
                    : priority_(priority), cell_(cell), parent_(parent), state_(state),
                    projection_(projectionDim), heapElement_(NULL)
                {
                }
                Motion(base::State *state, Motion *parent, base::EuclideanProjection projection,
                    double priority, Cell* cell)
                    : priority_(priority), cell_(cell), parent_(parent), state_(state),
                    projection_(projection), heapElement_(NULL)
                {
                }
                Motion(base::State *state, const ompl::base::ProjectionEvaluatorPtr& pe)
                    : priority_(0.), cell_(NULL), parent_(NULL), state_(state),
                    projection_(pe->getDimension()), heapElement_(NULL)
                {
                    pe->project(state_, projection_);
                }

                double score(void) const
                {
                    return priority_ / cell_->volume_;
                }
                void updatePriority(void)
                {
                    priority_ = priority_ * 2.0 + 1.0;
                }

                /// \brief Split a motion into two parts. The first part is
                /// duration steps long (and is returned), the second part is the
                /// remaing part of the motion and is kept in this object. It
                /// is assumed the motion is contained within one cell.
                Motion* split(const base::SpaceInformationPtr& si, double length,
                    const ompl::base::ProjectionEvaluatorPtr& pe)
                {
                    Motion* motion = new Motion(si->allocState(), parent_,
                        pe->getDimension(), priority_, cell_);
                    si->getStateSpace()->interpolate(parent_->state_, state_, length, motion->state_);
                    pe->project(motion->state_, motion->projection_);
                    cell_->motions_.push_back(motion);
                    parent_ = motion;
                    return motion;
                }

                /// Priority for selecting this path to extend from in the future
                double                           priority_;

                /// pointer to the cell that contains this path
                Cell*                            cell_;

                /// Parent motion from which this one started
                Motion*                          parent_;

                /// The state achieved by this motion
                ompl::base::State*               state_;

                /// The projection for this Motion
                ompl::base::EuclideanProjection  projection_;

                /// Handle to the element of the priority queue for this Motion
                ompl::BinaryHeap<Motion *, MotionCompare>::Element *heapElement_;
            };

            /// Cell is a Binary Space Partition
            struct Cell
            {
                Cell(double volume, const ompl::base::RealVectorBounds& bounds,
                    unsigned int splitDimension = 0)
                    : volume_(volume), splitDimension_(splitDimension), splitValue_(0.0),
                    left_(NULL), right_(NULL), bounds_(bounds)
                {
                }

                ~Cell()
                {
                    if (left_)
                        delete left_;
                    if (right_)
                        delete right_;
                }

                /// Subdivides this cell
                void subdivide(unsigned int spaceDimension);

                /// Locates the cell that this motion begins in
                Cell* stab(const ompl::base::EuclideanProjection& projection) const
                {
                    Cell *containingCell = const_cast<Cell*>(this);
                    while (containingCell->left_ != NULL)
                    {
                        if (projection[containingCell->splitDimension_] <= containingCell->splitValue_)
                            containingCell = containingCell->left_;
                        else
                            containingCell = containingCell->right_;
                    }
                    return containingCell;
                }
                /// Add a motion
                void addMotion(Motion *motion)
                {
                    motions_.push_back(motion);
                    motion->cell_ = this;
                }

                /// Volume of the cell
                double                       volume_;
                /// Dimension along which the cell is split into smaller cells
                unsigned int                 splitDimension_;
                /// The midpoint between the bounds_ at the splitDimension_
                double                       splitValue_;
                /// The left child cell (NULL for a leaf cell)
                Cell*                        left_;
                /// The right child cell (NULL for a leaf cell)
                Cell*                        right_;
                /// A bounding box for this cell
                ompl::base::RealVectorBounds bounds_;
                /// The motions contained in this cell. Motions are stored only in leaf nodes.
                std::vector<Motion*>         motions_;
            };


            /// Inserts the motion into the appropriate cell
            void insertSampleIntoBsp(Motion *motion, Cell *bsp,base::State* state,
                base::EuclideanProjection& projection);

            /// \brief Select a state along motion and propagate a new motion from there.
            /// Return NULL if no valid motion could be generated starting at the
            /// selected state.
            Motion* propagateFrom(Motion* motion, base::State* state, base::EuclideanProjection& projection);

            /// \brief Split motion into segments such that each segment is contained in a cell.
            Cell* split(const Cell* bsp, Cell* startCell, Motion* motion, int& numSegments,
                base::State* state, base::EuclideanProjection& projection);

            void freeMotion(Motion* m)
            {
                if (m->state_)
                    si_->freeState(m->state_);
                delete m;
            }

            void freeMemory(void);

            /// State sampler
            ompl::base::StateSamplerPtr              sampler_;

            // Random number generator
            RNG rng_;

            /// \brief Vector holding all of the start states supplied for the problem
            /// Each start motion is the root of its own tree of motions.
            std::vector<Motion*>                     startMotions_;

            /// Priority queue of motions
            ompl::BinaryHeap<Motion*, MotionCompare> priorityQueue_;

            /// Binary Space Partition
            Cell *bsp_;

            /// Projection evaluator for the problem
            ompl::base::ProjectionEvaluatorPtr       projectionEvaluator_;

            /// Number between 0 and 1 specifying the probability with which the goal should be sampled
            double goalBias_;

            /// Objected used to sample the goal
            ompl::base::GoalSampleableRegion*        goalSampler_;

            /// Iteration number and priority of the next Motion that will be generated
            unsigned int                             iteration_;

            /// Closest motion to the goal
            Motion*                                  lastGoalMotion_;

        };
    }
}

#endif
