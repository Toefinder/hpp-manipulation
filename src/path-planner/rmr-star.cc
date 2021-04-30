// Copyright 2018 (c) CNRS
// Authors: Margaux Sebal, Florent Lamiraux

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:

// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following
// disclaimer in the documentation and/or other materials provided
// with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.
#define HPP_DEBUG
#include "hpp/manipulation/path-planner/rmr-star.hh"

#include <hpp/pinocchio/configuration.hh>

#include <hpp/constraints/solver/by-substitution.hh>

#include <hpp/core/configuration-shooter.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/path-planner/k-prm-star.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/path-validation-report.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/roadmap.hh>


#include "hpp/manipulation/problem.hh"
#include "hpp/manipulation/roadmap.hh"

namespace hpp {
  namespace manipulation {
    namespace pathPlanner {

      typedef rmrStar::ContactState ContactState;
      typedef core::PathPtr_t PathPtr_t;
      typedef core::PathVector PathVector;
      typedef core::PathVectorPtr_t PathVectorPtr_t;
      typedef core::Nodes_t Nodes_t;
      typedef core::NodePtr_t NodePtr_t;
      typedef core::Edges_t Edges_t;
      typedef constraints::solver::HierarchicalIterative HierarchicalIterative;
      typedef constraints::solver::BySubstitution BySubstitution;
      typedef constraints::NumericalConstraints_t NumericalConstraints_t;
      using pinocchio::displayConfig;

      RMRStarPtr_t RMRStar::create (const core::ProblemConstPtr_t& problem,
                                    const core::RoadmapPtr_t& roadmap)
      {
        RMRStar* ptr;
        core::RoadmapPtr_t r2 = roadmap;
        RoadmapPtr_t r;
        try {
          ProblemConstPtr_t p(HPP_DYNAMIC_PTR_CAST(const Problem, problem));
          r = HPP_DYNAMIC_PTR_CAST (Roadmap, r2);
          ptr = new RMRStar (p, r);
        } catch (std::exception&) {
          if (!r)
            throw std::invalid_argument
              ("The roadmap must be of type hpp::manipulation::Roadmap.");
          else
            throw std::invalid_argument
              ("The problem must be of type hpp::manipulation::Problem.");
        }
        RMRStarPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      void RMRStar::computeStatesFromGraph ()
      {
        const std::size_t nbComp = graph_->nbComponents();

        for (std::size_t i=0; i<nbComp; ++i) {
          const graph::GraphComponentPtr_t graphComp (graph_->get(i));

          graph::EdgePtr_t edge (HPP_DYNAMIC_PTR_CAST(graph::Edge, graphComp));
          graph::StatePtr_t state (HPP_DYNAMIC_PTR_CAST(graph::State, graphComp));

          if (edge) {
            //create an indexed table with a node as key and the loop edge as value
            if (edge->stateFrom() == edge->stateTo()) {
              loopTransition_[edge->stateFrom()] = edge;
              statesWithLoops_.push_back(edge->stateTo());
            }
          }
          else if (state) {
            //create a vector of all the states, and a reverse map to get the index
            //in the vector of a state
            index_[state] = allStates_.size();
            allStates_.push_back(state);
          }
        }
      }

      ////////////////////////////////////////////////////////////////////////////

      void RMRStar::updateStatesToSample()
      {
        statesToSample_ = statesWithLoops_; return;
        do {
          ilpIt_->nextPath(ilpCurrentPath_);
#ifdef HPP_DEBUG
          std::string str = "";
          for (const graph::StatePtr_t& s: ilpCurrentPath_) {
            str += "\n\t";
            str += s->name();
            str += ";";
          }
          hppDout(info, "ilpIt next Path: " << str);
#endif
        } while (!ilpIt_->seemsWorkable(ilpCurrentPath_));
        statesToSample_ = ilpIt_->uniqueStatesIn(ilpCurrentPath_);
#ifdef HPP_DEBUG
          std::string str = "";
          for (const graph::StatePtr_t& s: ilpCurrentPath_) {
            str += "\n\t";
            str += s->name();
            str += ";";
          }
          hppDout(info, "ilpIt next working Path: " << str);
#endif
      }

      ContactState RMRStar::sampleContact ()
      {
        size_type maxNbTry =
          problem()->getParameter("RMR*/nbTryRandomConfig").intValue();
        ConfigurationShooterPtr_t shooter
          (manipulationProblem_->configurationShooter ());

        // Sample random state of the graph
        if (ilpCounter_ == ilpNextFreq_*statesToSample_.size())
          updateStatesToSample();

        std::size_t i_rand = rand() % (statesToSample_.size ());
        const graph::StatePtr_t s_rand = statesToSample_[i_rand];

        core::ValidationReportPtr_t validationReport;
        bool stateValid = false;

        while (!stateValid) {
          core::ConfigValidationsPtr_t configValidations
            (problem ()->configValidations ());

          ConstraintSetPtr_t stateConstraints =
            graph_->targetConstraint (loopTransition_[s_rand]);
          ConfigProjectorPtr_t configProjector =
            stateConstraints->configProjector ();
          NumericalConstraints_t numConstraints =
            configProjector->numericalConstraints();
          constraints::solver::BySubstitution solver
            (configProjector->solver ());

          // Sample a random constraint among the numerical constraints of
          // the random state selected.
          std::size_t randomSkip = rand() % numConstraints.size();

          // Loop over the constraints of the solver,
          // once every setRhsFreq_ times, generate a valid configuration
          // in the randomly selected state and set right hand side of
          // the randomly selected constraint from this configuration in
          // the solver.
          //
          // If the randomly selected constraint has already been used
          // (i.e. if it is stored in member rightHandSides_), randomly
          // select an already used right hand side.
          //
          // Note that first loop is useless if the randomly selected constraint
          // has already been used.
          for (std::size_t i = 0; i < numConstraints.size() ;i++)
          {
            if ((counter_% setRhsFreq_ == 0 && counter_>0) && i == randomSkip)
            {
              ConfigurationPtr_t q_rand;
              for (size_type j = 0; j < maxNbTry; j++)
              {
                // shoot random configuration
                q_rand = shooter->shoot();

                // project it on state
                bool applySuccess = s_rand->configConstraint()->apply(*q_rand);
                if (!applySuccess)
                  continue;

                // until it is valid
                bool validRhs =
                  configValidations->validate (*q_rand, validationReport);
                //hppDout(info, "q_rand rhs random" << displayConfig (*q_rand));
                if (validRhs)
                  break;
              } // end for j

              // set right hand side of randomly selected constraint with
              // this configuration
              bool rhsSet =
                solver.rightHandSideFromConfig (numConstraints[i], *q_rand);
              assert (rhsSet);
              continue;
            } // end if 

            // retrieve right hand sides already used for randomly selected
            // constraint.
            std::vector <constraints::vector_t> rhsOfTheFunctionAlreadyVisited;
            const std::string& constraintFnName =
              numConstraints[i]->functionPtr ()->name ();
            for (RightHandSides_t::const_iterator it = rightHandSides_.begin();
                 it != rightHandSides_.end(); ++it) {
              if (it->first->functionPtr ()->name () == constraintFnName)
                rhsOfTheFunctionAlreadyVisited.push_back (it->second);
            }

            if (!rhsOfTheFunctionAlreadyVisited.empty()) {
              // Randomly sample an already used right hand side for the
              // randomly selected constraint
              std::size_t indice_rand = rand() %
                rhsOfTheFunctionAlreadyVisited.size();
              constraints::vector_t rhs_rand
                (rhsOfTheFunctionAlreadyVisited [indice_rand]);
              assert (!rhs_rand.hasNaN());
              // set right hand side of the randomly selected constraint
              // to this value in the current solver.
              bool success = solver.rightHandSide (numConstraints[i], rhs_rand);
              assert(success);
            }
          } // end for i
          hppDout(info, "solver: " << solver);

          // Try to generate a valid configuration in the leaf selected
          // by the above loop.
          for (size_type i = 0; i < maxNbTry; i++) {
            //Shoot random configuration
            q_rand_ = shooter->shoot();

            HierarchicalIterative::Status constraintApplied =
              solver.solve (*q_rand_);
            assert(!q_rand_->hasNaN ());
            hppDout(info, "constraintApply: " << constraintApplied);
            if (constraintApplied != HierarchicalIterative::SUCCESS)
              continue;

            bool configValid =
              configValidations->validate (*q_rand_, validationReport);
            hppDout(info, "q_rand sampleContact: " << displayConfig(*q_rand_));

            stateValid = configValid;
            if (stateValid)
              break;
          }

          if (!stateValid) {
            hppDout(info, "fail to find a random configuration in state "
                     << s_rand->name ());
          }
        } // end while (!stateValid)

        //Get loopEdge constraints
        graph::EdgePtr_t loopEdge = loopTransition_[s_rand];
        core::ConstraintSetPtr_t edgeConstraints = loopEdge->pathConstraint();

        // get right hand side of constraint from q_rand
        ContactState contactState (s_rand, *q_rand_, edgeConstraints);

        return contactState;
      }

      void RMRStar::updateCounters() {
        if (counter_ == setRhsFreq_)
          counter_ = 0;
        counter_++;
        if (ilpCounter_ == ilpNextFreq_*statesToSample_.size())
          ilpCounter_ = 0;
        ilpCounter_++;
      }

      ////////////////////////////////////////////////////////////////////////////

      void RMRStar::buildLatestLeafRoadmap ()
      {
        using core::pathPlanner::kPrmStar;
        hppDout (info,"build state "<<latestLeaf_.state()->name());
        hppDout	(info, "latest leaf solver " <<latestLeaf_.solver ());

        //copy the problem and pass the edge contraints
        core::ProblemPtr_t p (core::Problem::createCopy(problem()));
        p->initConfig(q_rand_);
        p->steeringMethod (edgeSteeringMethod_->copy());
        p->constraints
          (core::ConstraintSet::createCopy(latestLeaf_.constraints ()));
        p->resetGoalConfigs();

        latestRoadmap_->clear();
        kPrm_ = kPrmStar::createWithRoadmap(p, latestRoadmap_);

        hppDout (info,"kPRM start Solve ");
        kPrm_->startSolve();

        kPrmStar::STATE kPrmState;

        do {
          kPrmState= kPrm_->getComputationState();
          kPrm_->oneStep();

        }	while (kPrmState != kPrmStar::CONNECT_INIT_GOAL);

      }
      ////////////////////////////////////////////////////////////////////////////

      void RMRStar::copyLatestRoadmapIntoGlobal ()
      {
        const core::RoadmapPtr_t& r = latestRoadmap_;
        const Edges_t& edges = r->edges ();
        hppDout (info, "number of edges: " << edges.size ());
        NodePtr_t node1;
        NodePtr_t node2;

        for (Edges_t::const_iterator itedge = edges.begin();
              itedge != edges.end(); itedge++){
          node1=roadmap_->addNode
            (ConfigurationPtr_t
             (new Configuration_t(*(*itedge)->from()->configuration())));
          node2=roadmap_->addNode
            (ConfigurationPtr_t
             (new Configuration_t(*(*itedge)->to()->configuration())));
          HPP_STATIC_PTR_CAST (core::Roadmap, roadmap_)->addEdge
            (node1, node2,(*itedge)->path());
        }

        Nodes_t nodes (r->nodes ());
        for (Nodes_t::const_iterator it = nodes.begin(); it != nodes.end(); ++it) {
          hppDout (info, "node = " << displayConfig (*(*it)->configuration ()));
        }
      }

      ////////////////////////////////////////////////////////////////////////////

      void RMRStar::connectLatestRoadmap ()
      {
        // Iterate through all pre-existing leaf roadmaps and try to connect
        // them to the latestLeaf_ roadmap.
        for (RMRStar::LeafRoadmaps_t::const_iterator itstate(leafRoadmaps_.begin());
          itstate != leafRoadmaps_.end(); ++itstate)
        {
          // We use the adjective "current" to denote objects related to
          // the loop iterator "itstate".

          // Current contact state
          const ContactState&  currentLeaf (itstate->first);
          graph::StatePtr_t currentState = currentLeaf.state ();
          // Current mapping of right hand sides
          ContactState::RightHandSides_t currentLeafRhs
            (currentLeaf.rightHandSides ());
          // current roadmap
          const core::RoadmapPtr_t& currentRoadmap (itstate->second.second);
          Nodes_t nodes = currentRoadmap->nodes ();

          // check if the states are different and neighboors or are similar
          graph::Edges_t connectedEdges =
            graph_->getEdges(latestLeaf_.state (), currentState);

          assert(connectedEdges.size() <= 1);

          // Whether states of latest leaf and current leaf are neighbors
          // and different
          bool neighborStates = (!connectedEdges.empty ()) &&
            (latestLeaf_.state () != currentState);

          bool rhsEqual = true;
          for (ContactState::RightHandSides_t::const_iterator it
                (latestLeaf_.rightHandSides ().begin());
            it != latestLeaf_.rightHandSides ().end(); ++it) {
            for (ContactState::RightHandSides_t::const_iterator i
                (currentLeafRhs.begin());
              i != currentLeafRhs.end(); ++i) {
              if (it->first == i->first && it->second != i->second ) {
                //both functions are equal but not the right hand sides
                rhsEqual = false;
                break;
              }
            }
            if (!rhsEqual)
              break;
          }

          // Get solver projecting on latest leaf explored
          constraints::solver::BySubstitution latestLeafSolver
            (latestLeaf_.solver ());

          // Get solver projecting on current leaf
          constraints::solver::BySubstitution currentLeafSolver
            (currentLeaf.solver ());

          pinocchio::value_type errorThreshold
            (biggestThreshold (latestLeafSolver, currentLeafSolver));

          // Test whether latest roadmap and current roadmap are in neighbor
          // states and accessible, or
          // are on he same leaf of the same state.
          if ((neighborStates && rhsEqual) ||
              (latestLeaf_.state () == currentState &&
               latestLeaf_.rightHandSide ().isApprox
               (currentLeaf.rightHandSide (), errorThreshold))) {
            if (latestLeaf_.state () == currentState &&
                latestLeaf_.rightHandSide ().isApprox
                (currentLeaf.rightHandSide (), errorThreshold)) {
              hppDout (info, displayConfig (latestLeaf_.rightHandSide ()));
              hppDout (info, displayConfig (currentLeaf.rightHandSide ()));
            }

            graph::WaypointEdgePtr_t waypointEdge
              (HPP_DYNAMIC_PTR_CAST(graph::WaypointEdge, connectedEdges[0]));
            core:: ValidationReportPtr_t validationReport;
            if (waypointEdge) {
              assert(latestLeaf_.state () != currentState);
              hppDout(info, "Problème avec Waypoints");
              hppDout(info, "From " << connectedEdges[0]->stateFrom()->name() <<
                            " to " <<connectedEdges[0]->stateTo()->name() <<
                            " through\n" << waypointEdge->waypoint(0)->stateTo()->name() <<
                            " and\n" << waypointEdge->waypoint(1)->stateTo()->name() <<
                            " and\n" << waypointEdge->waypoint(2)->stateTo()->name());

              connectStatesByWaypoints
                (waypointEdge, currentLeaf, latestLeaf_,
                 validationReport, currentRoadmap, currentState);
            }
            else {
              hppDout(info, "Problème sans Waypoints");
              hppDout(info, "From " << connectedEdges[0]->stateFrom()->name() <<
                            " to " << connectedEdges[0]->stateTo()->name());

              connectDirectStates
                (currentLeaf, latestLeaf_, validationReport,
                 currentState, currentRoadmap);
            }
          }
        }
        storeLatestLeafRoadmap();
      }

      void RMRStar::connectStatesByWaypoints
      (const graph::WaypointEdgePtr_t& waypointEdge,
       const ContactState& currentLeaf, const ContactState& latestLeaf,
       core::ValidationReportPtr_t& validationReport,
       const core::RoadmapPtr_t& roadmap,
       const graph::StatePtr_t& currentState)
      {
        assert (latestLeaf.state () == waypointEdge->stateFrom ());
        assert (currentLeaf.state () == waypointEdge->stateTo ());
        assert (currentState == waypointEdge->stateTo ());
        // Current mapping of right hand sides
        ContactState::RightHandSides_t currentLeafRhs
          (currentLeaf.rightHandSides ());
        // Configuration on current leaf
        core::ConfigValidationsPtr_t configValidations
          (manipulationProblem_->configValidations ());

        //get waypoints constraints
        std::size_t nWaypoints = waypointEdge->nbWaypoints();
        // log waypoint edge
        for (std::size_t j = 0; j < nWaypoints; ++j) {
          hppDout (info, "waypoint " << j);
          hppDout (info, "  from: "
                   << waypointEdge->waypoint (j)->stateFrom ()->name ());
          hppDout (info, "  to  : "
                   << waypointEdge->waypoint (j)->stateTo ()->name ());
        }
        ConfigurationPtr_t q_waypointInter;
        PathPtr_t path, projPath, validPath;
        hppDout(info, "nWaypoints = " << nWaypoints);
        Configuration_t q_prev, q_next;

        // Generate configuration at intersection of both leaves
        q_waypointInter = createInterStateConfig
          (currentLeaf, latestLeaf, configValidations, validationReport,
           currentState);
        // Detect on which waypoint this configuration lies
        if (!q_waypointInter) {
          hppDout (info, "Failed to generate configuration at intersection of "
                   " contact states " << currentState->name () <<
                   " and " << latestLeaf.state ()->name ());
          return;
        }
        std::size_t wp;
        bool success = false;
        // Detect on which waypoint state q_waypointInter lies
        for (wp = 0; wp < nWaypoints; ++wp) {
          graph::StatePtr_t targetState (waypointEdge->waypoint(wp)->stateTo ());

          if (targetState->configConstraint ()->configProjector ()->solver ().
              isSatisfied (*q_waypointInter)) {
            hppDout (info, displayConfig (*q_waypointInter) << " in state "
                     << targetState->name ());
            success = true;
            break;
          }
          hppDout (info, "error threshold: "
                   << targetState->configConstraint ()->configProjector ()->
                   solver ().errorThreshold ());
        }
        // Configuration should be in a waypoint state
        assert (success);

        // Project intersection configuration backward along intermediate
        // transitions
        std::vector <PathPtr_t> paths (nWaypoints);
        size_type i;
        Configuration_t q (*q_waypointInter);
        size_type max_iter (20);
        // q_waypointInter lies on waypoint [wp]->to ()
        for (i = (size_type) wp; i >= 0; --i) {
          graph::EdgePtr_t edge (waypointEdge->waypoint(i));
          graph::StatePtr_t initState (edge->stateFrom ());
          // project q on initState -> q_prev
          q_prev = q;
          // build solver for initial state of edge
          BySubstitution solver (edge->pathConstraint ()->configProjector ()->
                                 solver ());
          solver.rightHandSideFromConfig (q);
          const BySubstitution& ssolver (initState->configConstraint ()->
                                         configProjector ()->solver ());
          for (NumericalConstraints_t::const_iterator it
                 (ssolver.numericalConstraints ().begin ()); it !=
                 ssolver.numericalConstraints ().end (); ++it) {
            solver.add (*it);
          }
          solver.maxIterations (max_iter);
          // Compute q_prev
          HierarchicalIterative::Status status = solver.solve (q_prev);
          if (status != HierarchicalIterative::SUCCESS) {
            hppDout(info, "Failed to compute waypoint backward (" << i << ").");
            return;
          }
          hppDout(info, displayConfig(q_prev) << " in state " << initState->name());
          // build path between q_prev and q
          success = edge->build (path, q_prev, q);
          if (!success) {
            hppDout (info, "Failed to a build path between q_prev = "
                     << displayConfig (q_prev));
            hppDout(info, "and q = " << displayConfig (q));
            hppDout(info, "along edge " << edge->name ());
            return;
          }
          // project path - note that constraints are stored in path
          PathProjectorPtr_t pathProjector
            (manipulationProblem_->pathProjector ());
          projPath = path;
          if (pathProjector && !pathProjector->apply (path, projPath)) {
            hppDout (info, "Failed to project path between q_prev = "
                     << displayConfig (q_prev));
            hppDout (info, "and q = " << displayConfig (q));
            return;
          }
          // validate path
          PathValidationPtr_t pathValidation (edge->pathValidation());
          PathValidationReportPtr_t report;
          if (!pathValidation->validate (projPath, false, validPath,
                                         report)) {
            hppDout (info, "Failed to validate path between q_prev = "
                     << displayConfig (q_prev));
            hppDout (info, "and q = " << displayConfig (q));
            return;
          }
          // store path to build a path vector afterward
          paths [i] = projPath;
          q = q_prev;
        }

        // Project intersection configuration forward along intermediate
        // transitions
        q = *q_waypointInter;
        for (i = (size_type) wp + 1; i < (size_type) nWaypoints; ++i) {
          graph::EdgePtr_t edge (waypointEdge->waypoint(i));
          graph::StatePtr_t goalState (edge->stateTo ());
          // project q on initState -> q_next
          q_next = q;
          // build solver for goal state of edge
          BySubstitution solver (edge->pathConstraint ()->configProjector ()->
                                 solver ());
          solver.rightHandSideFromConfig (q);
          const BySubstitution& ssolver (goalState->configConstraint ()->
                                         configProjector ()->solver ());
          for (NumericalConstraints_t::const_iterator it
                 (ssolver.numericalConstraints ().begin ()); it !=
                 ssolver.numericalConstraints ().end (); ++it) {
            solver.add (*it);
          }
          solver.maxIterations (max_iter);
          // Compute q_next
          HierarchicalIterative::Status status = solver.solve (q_next);
          if (status != HierarchicalIterative::SUCCESS) {
            hppDout (info, "Failed to compute waypoint forward (" << i << ").");
            return;
          }
          hppDout(info, displayConfig(q_next) << " in state " << goalState->name());
          // build path between q and q_next
          success = edge->build (path, q, q_next);
          if (!success) {
            hppDout (info, "Failed to a build path between q = "
                     << displayConfig (q));
            hppDout(info, "and q_next = " << displayConfig (q_next));
            hppDout(info, "along edge " << edge->name ());
            return;
          }
          // project path - note that constraints are stored in path
          PathProjectorPtr_t pathProjector
            (manipulationProblem_->pathProjector ());
          projPath = path;
          if (pathProjector && !pathProjector->apply (path, projPath)) {
            hppDout (info, "Failed to project path between q = "
                     << displayConfig (q));
            hppDout (info, "and q_next = " << displayConfig (q_next));
            return;
          }
          // validate path
          PathValidationPtr_t pathValidation (edge->pathValidation());
          PathValidationReportPtr_t report;
          if (!pathValidation->validate (projPath, false, validPath,
                                         report)) {
            hppDout (info, "Failed to validate path between q = "
                     << displayConfig (q));
            hppDout (info, "and q_next = " << displayConfig (q_next));
            return;
          }
          // store path to build a path vector afterward
          paths [i] = projPath;
          q = q_next;
        }
        // At this point, q_prev and q_next are respectively on latestLeaf
        // and currentLeaf

        // Add these configurations to the global roadmap
        ConfigurationPtr_t pQprev (new Configuration_t (q_prev));
        NodePtr_t node1 (roadmap_->addNode (pQprev));
        ConfigurationPtr_t pQnext (new Configuration_t (q_next));
        NodePtr_t node2 (roadmap_->addNode (pQnext));
        // Create path vector with vector of waypoint paths
        PathVectorPtr_t pv (PathVector::create
                            (problem ()->robot ()->configSize (),
                             problem ()->robot ()->numberDof ()));
        for (std::vector <PathPtr_t>::const_iterator itPath (paths.begin ());
             itPath != paths.end (); ++itPath) {
          pv->appendPath (*itPath);
        }
        // Add path vector as an edge in the roadmap
        roadmap_->addEdges (node1, node2, pv);
        // Try to connect q_prev with nearest neighbors of latestLeaf
        size_type k = problem()->getParameter("RMR*/numberOfConnectNodes").intValue();
        Nodes_t nearNodes (latestRoadmap_->nearestNodes (pQprev, k));
        for (Nodes_t::const_iterator itNode (nearNodes.begin ());
             itNode != nearNodes.end (); ++itNode) {
          connectConfigToNode (loopTransition_ [latestLeaf.state ()],
                               (*itNode)->configuration (), pQprev);
        }
        // Try to connect q_next with nearest neighbors of currentLeaf
        nearNodes = roadmap->nearestNodes (pQnext, k);
        for (Nodes_t::const_iterator itNode (nearNodes.begin ());
             itNode != nearNodes.end (); ++itNode) {
          connectConfigToNode (loopTransition_ [currentLeaf.state ()],
                               pQnext, (*itNode)->configuration ());
        }
      }

      ////////////////////////////////////////////////////////////////////////

      void RMRStar::connectDirectStates
      (const ContactState& currentLeaf, const ContactState& latestLeaf,
       core::ValidationReportPtr_t& validationReport,
       const graph::StatePtr_t& currentState, const core::RoadmapPtr_t& roadmap)
      {
        core::ConfigValidationsPtr_t configValidations
          (manipulationProblem_->configValidations ());
        ConfigurationPtr_t q_inter = createInterStateConfig
          (currentLeaf, latestLeaf, configValidations, validationReport,
           currentState);
        if (q_inter) {
          size_type k = problem()->getParameter("RMR*/numberOfConnectNodes").intValue();
          graph::EdgePtr_t edgeTransit = loopTransition_[latestLeaf_.state()];
          Nodes_t nearNodes = latestRoadmap_->nearestNodes(q_inter, k);

          for (Nodes_t::const_iterator itnode = nearNodes.begin();
            itnode !=nearNodes.end(); itnode++) {
            ConfigurationPtr_t nodeConfig = (*itnode)->configuration();
            connectConfigToNode (edgeTransit,q_inter,nodeConfig);
          }

          edgeTransit = loopTransition_[currentState];
          nearNodes = roadmap->nearestNodes(q_inter, k);

          for (Nodes_t::const_iterator itnode = nearNodes.begin();
            itnode !=nearNodes.end(); itnode++) {
            ConfigurationPtr_t nodeConfig = (*itnode)->configuration();
            connectConfigToNode (edgeTransit,q_inter,nodeConfig);
          }
        }
      }

      ///////////////////////////////////////////////////////////////////////

      ConfigurationPtr_t RMRStar::createInterStateConfig
      (const ContactState& currentLeaf, const ContactState& latestLeaf,
       const core::ConfigValidationsPtr_t& configValidations,
       core::ValidationReportPtr_t& validationReport,
       const graph::StatePtr_t& currentState)
      {

        const NumericalConstraints_t& currentLeafNumericalConstraints
          (currentLeaf.solver ().numericalConstraints());
        // Get the numerical constraints of latest leaf
        const NumericalConstraints_t& latestLeafNumericalConstraints
          (latestLeaf.solver ().numericalConstraints());
        ConfigurationShooterPtr_t shooter
          (manipulationProblem_->configurationShooter ());
        HierarchicalIterative::Status constraintApplied
          (HierarchicalIterative::INFEASIBLE);
        hppDout(info, "currentState " << currentState->name());
        hppDout(info, "latestLeaf_ " << latestLeaf_.state()->name());

        constraints::solver::BySubstitution solver (latestLeaf_.solver ());

        assert(latestLeaf_.constraints ());

        //Copy the constraints and the right hand side in the new solver
        for (std::size_t j=0; j<currentLeafNumericalConstraints.size(); j++)
          solver.add (currentLeafNumericalConstraints[j]);

        for (std::size_t j=0; j<latestLeafNumericalConstraints.size(); j++)
          solver.rightHandSideFromConfig (latestLeafNumericalConstraints[j],
                                          latestLeaf_.config());
        for (std::size_t j=0; j<currentLeafNumericalConstraints.size(); j++)
          solver.rightHandSideFromConfig (currentLeafNumericalConstraints[j],
                                          currentLeaf.config ());
        hppDout(info, "solver inter state " << solver);
        hppDout(info, "error threshold " << solver.errorThreshold ());

        ConfigurationPtr_t q_inter;
        size_type i = 0;
        size_type max_iter (20);
        bool valid = false;
        for (; i < max_iter; i++)
        {
          //Shoot random configuration
          q_inter = shooter->shoot();
          constraintApplied = solver.solve(*q_inter);
          if (constraintApplied != HierarchicalIterative::SUCCESS)
            continue;
          valid = configValidations->validate (*q_inter, validationReport);
          hppDout (info, "q_inter=" << displayConfig (*q_inter));
          if (valid)
            break;
        }
        hppDout(info, "i=" << i);
        hppDout(info, "connect states " << latestLeaf_.state()->name() <<
                 "  currentState " << currentState->name());
        hppDout(info, "q_inter=" << displayConfig (*q_inter));
        hppDout(info, "constraintApplied=" << constraintApplied);

        //test constraints have been applied to q_inter
        if (!valid) {
          assert(i == max_iter);
          hppDout(info, "i reached max_iter, not any connect node have been found");
          q_inter = ConfigurationPtr_t ();
        }

        return q_inter;
      }
      ////////////////////////////////////////////////////////////////////////

      void RMRStar::connectConfigToNode
      (const graph::EdgePtr_t& edge, const ConfigurationPtr_t& q1,
       const ConfigurationPtr_t& configuration)
      {
        PathProjectorPtr_t pathProjector
          (manipulationProblem_->pathProjector ());
        //connect the interRoadmap to the nodeInter
        PathPtr_t validPath, path, projpath;
        NodePtr_t nodeConnect=
          roadmap_->addNode (ConfigurationPtr_t (new Configuration_t(*q1)));

        hppDout (info, " node1 " << displayConfig(*configuration));
        hppDout (info, " node2 " <<  displayConfig(*q1));

        core::SteeringMethodPtr_t sm = edge->steeringMethod();

        hppDout (info, displayConfig (*q1) << " in state "
                 << edge->stateFrom ()->name ());
        assert(edge->stateFrom()->contains(*q1));
        hppDout (info, displayConfig (*q1) << " in state "
                 << edge->state ()->name ());
        assert(edge->state()->contains(*q1));
        hppDout (info, displayConfig (*configuration) << " in state "
                 << edge->stateTo ()->name ());
        assert(edge->stateTo()->contains(*configuration));
        hppDout (info, displayConfig (*configuration) << " in state "
                 << edge->state ()->name ());
        assert(edge->state()->contains(*configuration));

        path =(*sm)(*q1, *configuration);

        if (path == NULL)
          hppDout (info, "path=NULL");
        else {
          projpath = path;
          if ((!pathProjector) || (pathProjector->apply(path,projpath))) {

            PathValidationPtr_t pathValidation ( edge->pathValidation());
            PathValidationReportPtr_t report;
            bool valid
              (pathValidation->validate (projpath, false, validPath, report));

            if (valid){
              NodePtr_t node
                (roadmap_->addNode(ConfigurationPtr_t
                                  (new Configuration_t(*configuration))));

              roadmap_->addEdges (nodeConnect, node, projpath);
              roadmap_->addEdges (node, nodeConnect, projpath->reverse ());
              assert (projpath->initial () == *(nodeConnect->configuration()));
              assert (projpath->end () == *(node->configuration()));
              hppDout (info,"edge created ");
            } else {
              hppDout(info, "path not valid");
            }
          }  else {
            hppDout(info, "path not OK");}
        }
      }

      ///////////////////////////////////////////////////////////////////////////

      static bool rhsNull (const vector_t& rhs)
      {
        for (size_type i = 0; i < rhs.size(); ++i )
          if (rhs[i] != 0)
            return false;
        return true;
      }

      void RMRStar::storeLatestLeafRhs ()
      {
        const constraints::solver::BySubstitution& solver (latestLeaf_.solver ());
        core::NumericalConstraints_t constraints = solver.numericalConstraints();

        for (std::size_t i = 0; i < constraints.size(); i++)
        {
          constraints::ImplicitPtr_t function = constraints[i];
          //constraints::vectorOut_t rhs = function->rightHandSide(); // DEPRECATED
          vector_t rhs (function->function ().outputSpace ()->nq ());
          bool success = solver.getRightHandSide(function, rhs);
          assert(!rhs.hasNaN ());
          assert(success);

          if (!rhsNull(rhs))
          {
            bool alreadyInMap = false;
            for (RightHandSides_t::const_iterator it = rightHandSides_.begin();
              it!= rightHandSides_.end(); ++it)
            {
              if ((function == it->first) &&
                  rhs.isApprox(it->second, solver.errorThreshold()))
              {
                alreadyInMap = true;
                break;
              }
            }

            if (!alreadyInMap)
              rightHandSides_.insert(
                std::pair<constraints::ImplicitPtr_t,constraints::vectorIn_t>
                  (function, rhs)
              );
          }
        }
      }

      //////////////////////////////////////////////////////////////////////////
      void RMRStar::storeLatestLeafRoadmap ()
      {
        //copy the problem and pass the edge contraints
        core::ProblemPtr_t p (core::Problem::createCopy(problem()));
        p->initConfig(q_rand_);
        p->steeringMethod (edgeSteeringMethod_->copy ());
        p->constraints
          (core::ConstraintSet::createCopy(latestLeaf_.constraints ()));
        p->pathValidation(loopTransition_[latestLeaf_.state()]->pathValidation());
        core::RoadmapPtr_t r = core::Roadmap::create(p->distance(),p->robot());
        r->clear();

        // Create a copy of the nodes of interRoadmap that we store in the
        // leafRoadmaps_
        for (Nodes_t::const_iterator itnode =latestRoadmap_->nodes().begin();
             itnode != latestRoadmap_->nodes().end(); itnode++ )
        {
          r->addNode(ConfigurationPtr_t
                    (new Configuration_t(*(*itnode)->configuration())));	
        }

        ProblemAndRoadmap_t pbRoadmap (p,r) ;

        //complete the map with the association ContactState/ProblemAndRoadmap
        leafRoadmaps_.insert(
          std::pair<ContactState,ProblemAndRoadmap_t>(
            latestLeaf_, pbRoadmap
          )
        );
      }
      ///////////////////////////////////////////////////////////////////////////

      pinocchio::value_type RMRStar::biggestThreshold
      ( const BySubstitution& solver1, const BySubstitution& solver2)
      {
        pinocchio::value_type threshold1 = solver1.errorThreshold();
        pinocchio::value_type threshold2 = solver2.errorThreshold();

        if (threshold1 < threshold2)
          return threshold2;
        else
          return threshold1;
      }
      ///////////////////////////////////////////////////////////////////////////

      void RMRStar::startSolve ()
      {
        PathPlanner::startSolve ();
        assert(manipulationProblem_.get() != 0);
        graph_ = manipulationProblem_->constraintGraph ();
        
        computeStatesFromGraph ();
        setRhsFreq_ = problem()->getParameter("RMR*/SetRhsFreq").intValue();
        counter_ = 0;

        ilpIt_ = IncreasingLengthPathsPtr_t
          (new rmrStar::IncreasingLengthPaths(graph_));
        //ilpIt_->initialize();
        ilpNextFreq_ = problem()->getParameter("RMR*/IlpNextFreq").intValue();
        ilpCounter_ = 0;

        //Set parameter and create interRoadmap
        latestRoadmap_ = core::Roadmap::create
          (manipulationProblem_->distance(),manipulationProblem_->robot());
        latestRoadmap_->clear();
        q_rand_ = roadmap()->initNode ()->configuration();

        //Set init ContactState
        graph::StatePtr_t stateInit = graph_->getState(*(q_rand_));
        graph::EdgePtr_t loop_edge = loopTransition_[stateInit];
        core::ConstraintSetPtr_t edgeConstraints = loop_edge->pathConstraint();
        edgeSteeringMethod_ = loop_edge -> steeringMethod();
        ContactState contactStateInit (stateInit, *q_rand_, edgeConstraints);
        //ilpIt_->setInitState(q_rand_);
        hppDout(info, "Set initial state to ilpIt: " << stateInit->name());

        latestLeaf_ = contactStateInit;
        buildLatestLeafRoadmap();
        copyLatestRoadmapIntoGlobal();
        storeLatestLeafRhs();
        connectLatestRoadmap();
        //leafRoadmap_ is empty so connectRoadmap() just does storeLeafRoadmap()

        for (core::NodeVector_t::const_iterator itn
               (roadmap_->goalNodes ().begin ());
             itn != roadmap_->goalNodes ().end (); ++itn) {
          q_rand_=(*itn)->configuration();

          graph::StatePtr_t stateGoal = graph_->getState(*q_rand_);
        }
        for (core::NodeVector_t::const_iterator itn
               (roadmap_->goalNodes ().begin ());
             itn != roadmap_->goalNodes ().end (); ++itn) {

          latestRoadmap_ -> clear();

          //Set final ContactState
          q_rand_=(*itn)->configuration();

          graph::StatePtr_t stateGoal = graph_->getState(*q_rand_);
          loop_edge = loopTransition_[stateGoal];
          edgeConstraints = loop_edge->pathConstraint();
          ContactState contactStateGoal (stateGoal,*q_rand_, edgeConstraints);
          //ilpIt_->addGoalState(q_rand_);
          hppDout(info, "Added goal state to ilpIt: " << stateGoal->name());

          latestLeaf_ = contactStateGoal;
          buildLatestLeafRoadmap();
          copyLatestRoadmapIntoGlobal();
          storeLatestLeafRhs();
          connectLatestRoadmap();
        }
        step_=BUILD_ROADMAP;
        updateStatesToSample();
      }

      ////////////////////////////////////////////////////////////////////////////
      void RMRStar::oneStep ()
      {
        // todo : modifier samplecontact pour qu'il prenne que dans les états du 
        // ILP actuel. Créer un 2e compteur qui une fois un seuil atteint passe
        // au ILP suivant
        latestLeaf_ = sampleContact();
        buildLatestLeafRoadmap();
        copyLatestRoadmapIntoGlobal();
        storeLatestLeafRhs();
        connectLatestRoadmap();

        updateCounters();

        /*
        switch (step_)
        {

        case BUILD_ROADMAP:

          latestLeaf_ = sampleContact();
          buildLatestLeafRoadmap();
          copyLatestRoadmapIntoGlobal();
          storeLatestLeafRhs();

          updateCounters();

          step_=CONNECT_ROADMAPS;
          break;

        case CONNECT_ROADMAPS:

          connectLatestRoadmap();
          step_= BUILD_ROADMAP;

          break;
        }
        */
      }
      ////////////////////////////////////////////////////////////////////////////

      RMRStar::RMRStar (const core::ProblemConstPtr_t& problem,
                        const core::RoadmapPtr_t& roadmap) :
        core::PathPlanner (problem, roadmap),
        loopTransition_ (),
        manipulationProblem_ (HPP_STATIC_PTR_CAST(const manipulation::Problem, problem)),
        roadmap_ (HPP_STATIC_PTR_CAST (manipulation::Roadmap, roadmap))
      {
        assert(HPP_DYNAMIC_PTR_CAST(const Problem, problem));
        assert(HPP_DYNAMIC_PTR_CAST(manipulation::Roadmap, roadmap));
      }
      ////////////////////////////////////////////////////////////////////////////
      // ----------- Declare parameters ------------------------------------- //
      using core::Parameter;
      using core::ParameterDescription;
      HPP_START_PARAMETER_DECLARATION(RMRStar)
      core::Problem::declareParameter(ParameterDescription
        (Parameter::INT,
        "RMR*/numberOfConnectNodes",
        "The desired number of the nodes we try to connect "
        "between the intersection node and the roadmaps.",
        Parameter((size_type)3)));

      core::Problem::declareParameter(ParameterDescription
        (Parameter::INT,
        "RMR*/SetRhsFreq",
        "The desired number of the frequency of set configuration "
        "map build. If it's 0 it's never manually set, "
        "if it's N>0 it's 100(1-1/N)% of the time manually set.",
        Parameter((size_type)100)));

      core::Problem::declareParameter(ParameterDescription
        (Parameter::INT,
        "RMR*/IlpNextFreq",
        "The desired number of steps between each change in "
        "ilpCurrentPath_.",
        Parameter((size_type)5)));

      core::Problem::declareParameter(ParameterDescription
        (Parameter::INT,
        "RMR*/nbTryRandomConfig",
        "The number of attempt to sample a "
        "valid random configuration in a given state.",
        Parameter ((size_type)100)));
      HPP_END_PARAMETER_DECLARATION(RMRStar)
    } // namespace pathPlanner
  } // namespace manipulation
} // namespace hpp
