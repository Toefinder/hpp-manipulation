// Copyright (c) 2021, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr),
//          Florent Lamiraux (florent.lamiraux@laas.fr)
//          Alexandre Thiault (athiault@laas.fr)
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

#include <hpp/manipulation/path-planner/in-state-path.hh>

#include <hpp/util/exception-factory.hh>

#include <hpp/pinocchio/configuration.hh>

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
#include <hpp/manipulation/path-optimization/random-shortcut.hh>

#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/roadmap.hh>


namespace hpp {
  namespace manipulation {
    namespace pathPlanner {

      InStatePathPtr_t InStatePath::create (
          const core::ProblemConstPtr_t& problem)
      {
        InStatePath* ptr;
        RoadmapPtr_t r = Roadmap::create(problem->distance(), problem->robot());
        try {
          ProblemConstPtr_t p(HPP_DYNAMIC_PTR_CAST(const Problem, problem));
          ptr = new InStatePath (p, r);
        } catch (std::exception&) {
          throw std::invalid_argument
            ("The problem must be of type hpp::manipulation::Problem.");
        }
        InStatePathPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      InStatePathPtr_t InStatePath::createWithRoadmap (
          const core::ProblemConstPtr_t& problem,
          const core::RoadmapPtr_t& roadmap)
      {
        InStatePath* ptr;
        core::RoadmapPtr_t r2 = roadmap;
        RoadmapPtr_t r;
        try {
          ProblemConstPtr_t p(HPP_DYNAMIC_PTR_CAST(const Problem, problem));
          r = HPP_DYNAMIC_PTR_CAST (Roadmap, r2);
          ptr = new InStatePath (p, r);
        } catch (std::exception&) {
          if (!r)
            throw std::invalid_argument
              ("The roadmap must be of type hpp::manipulation::Roadmap.");
          else
            throw std::invalid_argument
              ("The problem must be of type hpp::manipulation::Problem.");
        }
        InStatePathPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      InStatePathPtr_t InStatePath::copy () const
      {
        InStatePath *ptr = new InStatePath(*this);
        InStatePathPtr_t shPtr(ptr);
        ptr->init(shPtr);
        return shPtr;
      }

      void InStatePath::maxIterPlanner(const unsigned long& maxiter)
      {
        maxIterPathPlanning_ = maxiter;
      }

      void InStatePath::timeOutPlanner(const double& timeout)
      {
        timeOutPathPlanning_ = timeout;
      }

      void InStatePath::resetRoadmap(const bool& resetroadmap)
      {
        resetRoadmap_ = resetroadmap;
      }

      void InStatePath::plannerType(const std::string& plannertype)
      {
        plannerType_ = plannertype;
      }
      
      void InStatePath::addOptimizerType(const std::string& opttype)
      {
        optimizerTypes_.push_back(opttype);
      }
      
      void InStatePath::resetOptimizerTypes()
      {
        optimizerTypes_.clear();
      }

      void InStatePath::setEdge (const graph::EdgePtr_t& edge)
      {
        constraints_ = edge->pathConstraint();
        cproblem_->pathValidation(edge->pathValidation());
        cproblem_->constraints(constraints_);
        cproblem_->steeringMethod(edge->steeringMethod());
      }

      void InStatePath::setInit (const ConfigurationPtr_t& qinit)
      {
        if (!constraints_)
          throw std::logic_error("Use setEdge before setInit and setGoal");
        constraints_->configProjector()->rightHandSideFromConfig(*qinit);
        cproblem_->initConfig(qinit);
        cproblem_->resetGoalConfigs();
      }

      void InStatePath::setGoal (const ConfigurationPtr_t& qgoal)
      {
        if (!constraints_)
          throw std::logic_error("Use setEdge before setInit and setGoal");
        ConfigurationPtr_t qgoalc (new Configuration_t (*qgoal));
        constraints_->apply(*qgoalc);
        assert((*qgoal-*qgoalc).isZero());
        cproblem_->resetGoalConfigs();
        cproblem_->addGoalConfig(qgoal);
      }

      core::PathVectorPtr_t InStatePath::solve()
      {
        if (!constraints_)
          throw std::logic_error("Use setEdge, setInit and setGoal before solve");
        if (resetRoadmap_ || !roadmap_)
          roadmap_ = core::Roadmap::create (cproblem_->distance(), problem_->robot());
        
        core::PathPlannerPtr_t planner;
        // TODO: BiRRT* does not work properly:
        //     - discontinuities due to an algorithmic mistake involving qProj_
        //     - not using path projectors, it should
        if (plannerType_ == "kPRM*")
          planner = core::pathPlanner::kPrmStar::createWithRoadmap(cproblem_, roadmap_);
        else if (plannerType_ == "DiffusingPlanner")
          planner = core::DiffusingPlanner::createWithRoadmap(cproblem_, roadmap_);
        else if (plannerType_ == "BiRRT*")
          planner = core::BiRRTPlanner::createWithRoadmap(cproblem_, roadmap_);
        else {
          hppDout(warning, "Unknown planner type specified. Setting to default DiffusingPlanner");
          planner = core::DiffusingPlanner::createWithRoadmap(cproblem_, roadmap_);
        }
        if (maxIterPathPlanning_)
            planner->maxIterations(maxIterPathPlanning_);
        if (timeOutPathPlanning_)
          planner->timeOut(timeOutPathPlanning_);
        if (!maxIterPathPlanning_ && !timeOutPathPlanning_)
            planner->stopWhenProblemIsSolved(true);
        core::PathVectorPtr_t path = planner->solve();
        
        for (const std::string& optType: optimizerTypes_) {
          namespace manipOpt = pathOptimization;
          namespace coreOpt = core::pathOptimization;
          PathOptimizerPtr_t optimizer;
          if (optType == "RandomShortcut")
            optimizer = coreOpt::RandomShortcut::create(problem_);
          else if (optType == "SimpleShortcut")
            optimizer = coreOpt::SimpleShortcut::create(problem_);
          else if (optType == "PartialShortcut")
            optimizer = coreOpt::PartialShortcut::create(problem_);
          else if (optType == "SimpleTimeParameterization")
            optimizer = coreOpt::SimpleTimeParameterization::create(problem_);
          else if (optType == "Graph-RandomShortcut")
            optimizer = manipOpt::RandomShortcut::create(problem_);
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