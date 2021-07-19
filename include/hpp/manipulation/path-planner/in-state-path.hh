// Copyright (c) 2021, 
// Authors: 
//
// This file is part of hpp-manipulation.
// hpp-manipulation is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation. If not, see <http://www.gnu.org/licenses/>.

#ifndef HPP_MANIPULATION_PATH_PLANNER_IN_STATE_PATH_HH
# define HPP_MANIPULATION_PATH_PLANNER_IN_STATE_PATH_HH

# include <hpp/core/path-planner.hh>
# include <hpp/core/config-projector.hh>
# include <hpp/core/collision-validation.hh>
# include <hpp/core/joint-bound-validation.hh>

# include <hpp/manipulation/config.hh>
# include <hpp/manipulation/fwd.hh>
# include <hpp/manipulation/problem.hh>
# include <hpp/manipulation/steering-method/graph.hh>

namespace hpp {
  namespace manipulation {
    namespace pathPlanner {
      /// \addtogroup path_planner
      /// \{

      /// 
      ///
      /// #### Sketch of the method
      ///
      /// #### Problem resolution
      ///
      /// #### Current status
      ///
      class HPP_MANIPULATION_DLLAPI InStatePath : public core::PathPlanner
      {
        public:
          virtual ~InStatePath()
          {}

          static InStatePathPtr_t create (const ProblemConstPtr_t& problem);

          InStatePathPtr_t copy () const;

          void setEdge (const graph::EdgePtr_t& edge);
          void setInit (const ConfigurationPtr_t& q);
          void setGoal (const ConfigurationPtr_t& q);

          virtual void oneStep() {}
          virtual core::PathVectorPtr_t solve();

          int maxIterPathPlanning = 0;
          double timeOutPathPlanning = 0;
          std::string plannerType = "BiRRT*";
          std::vector<std::string> optimizerTypes;
          bool resetRoadmap = true;

        protected:
          InStatePath (const ProblemConstPtr_t& problem) :
            PathPlanner(problem),
            problem_ (problem),
            roadmap_(), constraints_(), weak_()
          {
            const core::DevicePtr_t& robot = problem_->robot();
            cproblem_ = core::Problem::create(robot);
            cproblem_->setParameter
              ("kPRM*/numberOfNodes", core::Parameter((size_type) 2000));
            cproblem_->clearConfigValidations();
            cproblem_->addConfigValidation(core::CollisionValidation::create(robot));
            cproblem_->addConfigValidation(core::JointBoundValidation::create(robot));
            for (const core::CollisionObjectPtr_t & obs: problem_->collisionObstacles()) {
              cproblem_->addObstacle(obs);
            }
          }

          InStatePath (const InStatePath& other) :
            PathPlanner(other.problem_),
            problem_(other.problem_), cproblem_(other.cproblem_),
            roadmap_ (other.roadmap_),
            constraints_(other.constraints_),
            weak_ ()
          {}

          void init (InStatePathWkPtr_t weak)
          {
            weak_ = weak;
          }

        private:

          // a pointer to the problem used to create the InStatePath instance
          ProblemConstPtr_t problem_;
          // a new problem created for this InStatePath instance
          core::ProblemPtr_t cproblem_;
          core::RoadmapPtr_t roadmap_;
          ConstraintSetPtr_t constraints_;
          /// Weak pointer to itself
          InStatePathWkPtr_t weak_;

      }; // class InStatePath
      /// \}

    } // namespace pathPlanner
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_STEERING_METHOD_CROSS_STATE_OPTIMIZATION_HH
