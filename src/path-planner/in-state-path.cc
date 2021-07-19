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
#define HPP_DEBUG
#include <hpp/manipulation/path-planner/in-state-path.hh>

#include <map>
#include <queue>
#include <vector>

#include <hpp/util/exception-factory.hh>

#include <pinocchio/multibody/model.hpp>

#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/joint-collection.hh>

#include <hpp/core/path-vector.hh>
#include <hpp/core/roadmap.hh>

#include <hpp/core/bi-rrt-planner.hh>
#include <hpp/core/path-planner/k-prm-star.hh>
#include <hpp/core/diffusing-planner.hh>

#include <hpp/core/path-optimizer.hh>
#include <hpp/core/path-optimization/random-shortcut.hh>
#include <hpp/core/path-optimization/simple-shortcut.hh>
#include <hpp/core/path-optimization/partial-shortcut.hh>
#include <hpp/core/path-optimization/simple-time-parameterization.hh>

#include <hpp/manipulation/graph/edge.hh>


namespace hpp {
  namespace manipulation {
    namespace pathPlanner {

      InStatePathPtr_t InStatePath::create (const ProblemConstPtr_t &problem)
      {
        InStatePathPtr_t shPtr(new InStatePath(problem));
        shPtr->init(shPtr);
        return shPtr;
      }

      InStatePathPtr_t InStatePath::copy () const
      {
        InStatePath *ptr = new InStatePath(*this);
        InStatePathPtr_t shPtr(ptr);
        ptr->init(shPtr);
        return shPtr;
      }

      void InStatePath::setEdge (const graph::EdgePtr_t& edge)
      {
        constraints_ = edge->pathConstraint();
        try {
          cproblem_->pathValidation(edge->pathValidation());
          hppDout(info, "a");
        } catch(const std::runtime_error&(e)) {
          hppDout(info, "b");
          //TODO1 comprendre pourquoi ce truc des pools
          //TODO2 comprendre aussi les Failed to apply constraints in StraightPath::extract
          assert ((std::string) (e.what()) == "Cannot free pool when some objects are in use.");
        }
        cproblem_->constraints(constraints_);
        cproblem_->steeringMethod(edge->steeringMethod());
        try {
          cproblem_->filterCollisionPairs();
          hppDout(info, "c");
        } catch(const std::runtime_error&(e)) {
          hppDout(info, "d");
          assert ((std::string) (e.what()) == "Cannot free pool when some objects are in use.");
        }
      }

      void InStatePath::setInit (const ConfigurationPtr_t& qinit)
      {
        assert(constraints_);
        constraints_->configProjector()->rightHandSideFromConfig(*qinit);
        cproblem_->initConfig(qinit);
        cproblem_->resetGoalConfigs();
      }

      void InStatePath::setGoal (const ConfigurationPtr_t& qgoal)
      {
        assert(constraints_);
        ConfigurationPtr_t qgoalc (new Configuration_t (*qgoal));
        constraints_->apply(*qgoalc);
        assert((*qgoal-*qgoalc).isZero());
        cproblem_->resetGoalConfigs();
        cproblem_->addGoalConfig(qgoal);
      }

      core::PathVectorPtr_t InStatePath::solve() {

        if (resetRoadmap || !roadmap_)
          roadmap_ = core::Roadmap::create (cproblem_->distance(), problem_->robot());
        
        core::PathPlannerPtr_t planner_;
        if (plannerType == "kPRM*")
          planner_ = core::pathPlanner::kPrmStar::createWithRoadmap(cproblem_, roadmap_);
        else if (plannerType == "DiffusingPlanner")
          planner_ = core::DiffusingPlanner::createWithRoadmap(cproblem_, roadmap_);
        else if (plannerType == "BiRRT*")
          planner_ = core::BiRRTPlanner::createWithRoadmap(cproblem_, roadmap_);
        else
          planner_ = core::BiRRTPlanner::createWithRoadmap(cproblem_, roadmap_);
        if (maxIterPathPlanning)
            planner_->maxIterations(maxIterPathPlanning);
        if (timeOutPathPlanning)
            planner_->timeOut(timeOutPathPlanning);
        if (!maxIterPathPlanning && !timeOutPathPlanning)
            planner_->stopWhenProblemIsSolved(true);
        core::PathVectorPtr_t path = planner_->solve();
        
        for (const std::string& optType: optimizerTypes) {
          using namespace core::pathOptimization;
          PathOptimizerPtr_t optimizer;
          if (optType == "RandomShortcut")
            optimizer = RandomShortcut::create(problem_);
          else if (optType == "SimpleShortcut")
            optimizer = SimpleShortcut::create(problem_);
          else if (optType == "PartialShortcut")
            optimizer = PartialShortcut::create(problem_);
          else if (optType == "SimpleTimeParameterization")
            optimizer = SimpleTimeParameterization::create(problem_);
          else
            continue;
          try {
            path = optimizer->optimize(path);
          } catch (const hpp::Exception& e) {
            hppDout(info, "could not optimize " << e.what());
          }
        }
        return path;
      }
      
    } // namespace pathPlanner
  } // namespace manipulation
} // namespace hpp