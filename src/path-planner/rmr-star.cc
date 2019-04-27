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
      typedef core::ConfigValidationsPtr_t ConfigValidationsPtr_t;
      typedef core::PathPtr_t PathPtr_t;
      typedef core::PathVector PathVector;
      typedef core::PathVectorPtr_t PathVectorPtr_t;
      typedef core::Nodes_t Nodes_t;
      typedef core::NodePtr_t NodePtr_t;
      typedef core::Edges_t Edges_t;
      typedef core::ValidationReportPtr_t ValidationReportPtr_t;
      typedef constraints::solver::HierarchicalIterative HierarchicalIterative;
      typedef constraints::solver::BySubstitution BySubstitution;
      typedef constraints::NumericalConstraints_t NumericalConstraints_t;
      using pinocchio::displayConfig;

      RMRStarPtr_t RMRStar::create (const core::Problem& problem,
                                    const core::RoadmapPtr_t& roadmap)
      {
        RMRStar* ptr;
        core::RoadmapPtr_t r2 = roadmap;
        RoadmapPtr_t r;
        try {
          const Problem& p = dynamic_cast < const Problem& > (problem);
          RoadmapPtr_t r = HPP_DYNAMIC_PTR_CAST (Roadmap, r2);
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

      void RMRStar::computeTransitionMap ()
      {
        // Access to the constraint graph

        const std::size_t nbComp = graph_->nbComponents();

        //create an indexed table with a node as key and the loop edge as value
        for (std::size_t i=0; i<nbComp; ++i)
          {
            const graph::GraphComponentPtr_t graphComp (graph_->get(i));

            graph::EdgePtr_t edge (HPP_DYNAMIC_PTR_CAST(graph::Edge, graphComp));

            if(edge)
              {
                if (edge->from()==edge->to())

                  {
                    transition_[edge->from()]=edge;
                  }
              }
          }
      }

      ////////////////////////////////////////////////////////////////////////////

      std::vector<graph::StatePtr_t> RMRStar::extract_keys
      (RMRStar::TransitionMap_t input_map) {

        //create a vector with all the input_map keys

        std::vector<graph::StatePtr_t> retval;

        for (TransitionMap_t::const_iterator i=input_map.begin() ;
             i!= input_map.end() ; ++i)  {

          retval.push_back(i->first);
        }
        return retval;
      }

      ////////////////////////////////////////////////////////////////////////////

      bool sampleInState (const graph::StatePtr_t& state,
                          const ConfigurationShooterPtr_t& shooter,
                          const ConfigValidationsPtr_t& configValidations,
                          size_type maxNbTry, ConfigurationPtr_t& q)
      {
        bool valid (false); size_type j=0;
        while (!valid && j <= maxNbTry) {
          // shoot random configuration
          q = shooter->shoot();
          // project it on state
          bool success = state->configConstraint()->apply(*q);
          if (success) {
            // until it is valid
            ValidationReportPtr_t validationReport;
            valid = configValidations->validate (*q, validationReport);
            ++j;
          }
        } // while (!valid && j <= maxNbTry)
        return valid;
      }

      bool sampleValidConfiguration
      (const BySubstitution& solver, const ConfigurationShooterPtr_t& shooter,
       const ConfigValidationsPtr_t& configValidations, size_type maxNbTry,
       ConfigurationPtr_t& q)
      {
        bool valid (false); size_type j (0);
        while (!valid && j <= maxNbTry) {
          // shoot random configuration
          q = shooter->shoot();
          // project it on state
          HierarchicalIterative::Status status = solver.solve (*q);
          if (status == HierarchicalIterative::SUCCESS) {
            // until it is valid
            ValidationReportPtr_t validationReport;
            valid = configValidations->validate (*q, validationReport);
            ++j;
          }
        } // while (!valid && j <= maxNbTry)
        return valid;
      }

      ContactState RMRStar::sampleContact ()
      {
        size_type maxNbTry (problem ().getParameter
                            ("RMR*/nbTryRandomConfig").intValue ());
        ConfigurationShooterPtr_t shooter
          (manipulationProblem_.configurationShooter ());
        std::size_t nbStates (transition_.size ());
        // Shoot random number
        std::size_t i_rand=rand() % nbStates;

        // Sample random state of the graph
        const graph::StatePtr_t s_rand =
          RMRStar::extract_keys(transition_)[i_rand];

        bool stateValid = false;
        ConfigurationPtr_t q;

        while (!stateValid) {
          ValidationReportPtr_t validationReport;
          ConfigValidationsPtr_t configValidations
            (problem ().configValidations ());

          ConstraintSetPtr_t stateConfig =
            graph_->configConstraint (transition_[s_rand]);
          NumericalConstraints_t numConstraints =
            stateConfig->configProjector ()->numericalConstraints();

          BySubstitution solver
            (stateConfig->configProjector ()->solver ());

          // Check whether right hand side of each constraint has already
          // been instantiated. If not, a random configuration needs to be
          // sampled in this state.
          bool needToSample (false);
          std::vector < std::vector <constraints::vector_t> >
            rhsOfConstraint (numConstraints.size ());
          for (std::size_t i = 0; i < numConstraints.size (); ++i) {
            // retrieve right hand sides already used for randomly selected
            // constraint.
            for (RightHandSides_t::const_iterator it=rightHandSides_.begin ();
                 it!= rightHandSides_.end () ; ++it) {
              if (it->first->functionPtr ()->name () ==
                  numConstraints [i]->functionPtr ()->name ()) {
                rhsOfConstraint [i].push_back (it->second);
              }
            }
            if (rhsOfConstraint [i].empty ()) {
              needToSample = true;
            }
          }

          // If at least one of the constraints has never been instantiated,
          // sample a valid configuration in state
          if (needToSample) {
            if (!sampleInState (s_rand, shooter, configValidations, maxNbTry,
                                q)) {
              continue;
            }
          }

          // Instantiate right hand side of each constraint
          for (std::size_t i = 0; i < numConstraints.size (); ++i) {
            if (rhsOfConstraint [i].empty ()) {
              // Initialize right hand side with random configuration
#ifndef NDEBUG
              bool success =
#endif
                solver.rightHandSideFromConfig (numConstraints [i], *q);
              assert (success);
            } else {
              // Sample right hand side among already instantiated
              std::size_t indice_rand = rand() % rhsOfConstraint [i].size();
#ifndef NDEBUG
              bool success =
#endif
                solver.rightHandSide (numConstraints [i],
                                      rhsOfConstraint [i] [indice_rand]);
              assert (success);
            }
          }
          // At this point, the right hand side of the solver is set.
          // We now sample a valid configuration with solver
          stateValid = sampleValidConfiguration
            (solver, shooter, configValidations, maxNbTry, q);
        } // while (!stateValid)

        //Get loop_edge constraints
        graph::EdgePtr_t loop_edge = transition_ [s_rand];
        core::ConstraintSetPtr_t edgeConstraints = loop_edge->pathConstraint();

        // retrieved the edge steering method
        edgeSteeringMethod_ = loop_edge -> steeringMethod();

        // get right hand side of constraint from q_rand
        ContactState contactState (s_rand, *q, edgeConstraints);
        return contactState;
      }

      ////////////////////////////////////////////////////////////////////////////

      void RMRStar::buildRoadmap ()
      {
        using core::pathPlanner::kPrmStar;
        hppDout (info,"build state "<<latestLeaf_.state()->name());
        hppDout	(info, "latest leaf solver " <<latestLeaf_.solver ());

        //copy the problem and pass the edge contraints
        core::Problem p (problem ());
        p.initConfig (ConfigurationPtr_t
                      (new Configuration_t (latestLeaf_.config ())));
        p.steeringMethod (edgeSteeringMethod_);
        p.constraints(latestLeaf_.constraints ());
        p.resetGoalConfigs();

        latestRoadmap_->clear();
        kPrm_=kPrmStar::createWithRoadmap(p,latestRoadmap_);

        hppDout (info,"kPRM start Solve ");
        kPrm_->startSolve();

        kPrmStar::STATE kPrmState;

        do {
          kPrmState= kPrm_->getComputationState();
          kPrm_->oneStep();

        }	while (kPrmState != kPrmStar::CONNECT_INIT_GOAL);

      }
      ////////////////////////////////////////////////////////////////////////////

      void RMRStar::copyRoadmapIntoGlobal (const core::RoadmapPtr_t& r)
      {
        const Edges_t& edges = r->edges ();
        hppDout (info, "number of edges: " << edges.size ());
        NodePtr_t node1;
        NodePtr_t node2;

        size_type i = 0;
        for  (Edges_t::const_iterator itedge = edges.begin();
              itedge != edges.end(); itedge++){
          node1=roadmap_->addNode
            (ConfigurationPtr_t
             (new Configuration_t(*(*itedge)->from()->configuration())));
          node2=roadmap_->addNode
            (ConfigurationPtr_t
             (new Configuration_t(*(*itedge)->to()->configuration())));
          HPP_STATIC_PTR_CAST (core::Roadmap, roadmap_)->addEdge
            (node1, node2,(*itedge)->path());
          ++i;
        }
        Nodes_t nodes (r->nodes ());
        for (Nodes_t::const_iterator it = nodes.begin (); it != nodes.end ();
             ++it) {
          hppDout (info, "node = " << displayConfig (*(*it)->configuration ()));
        }
      }

      ////////////////////////////////////////////////////////////////////////////

      void RMRStar::connectRoadmap ()
      {
        ValidationReportPtr_t validationReport;
        size_type k=problem().getParameter("RMR*/numberOfConnectNodes").intValue();

        // Iterate through all pre-existing leaf roadmaps and try to connect
        // them to the latest leaf roadmap.
        for (RMRStar::LeafRoadmaps_t::const_iterator itstate
               (leafRoadmaps_.begin());itstate != leafRoadmaps_.end();
             ++itstate) {
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
          Nodes_t nodes= currentRoadmap->nodes ();

          // check if the states are different and neighboors or are similar
          graph::Edges_t connectedEdges =
            graph_->getEdges(latestLeaf_.state (),currentState);

          assert (connectedEdges.size()<=1);

          // Whether states of latest leaf and current leaf are neighbors
          // and different
          bool neighborStates = (!connectedEdges.empty ()) &&
            (latestLeaf_.state () !=currentState);
          bool valid=false;

          // Get solver projecting on latest leaf explored
          BySubstitution latestLeafSolver
            (latestLeaf_.solver ());

          // Get solver projecting on current leaf
          BySubstitution currentLeafSolver
            (currentLeaf.solver ());

          bool rhsEqual=true;
          pinocchio::value_type errorThreshold
            (biggestThreshold (latestLeafSolver, currentLeafSolver));

          for (ContactState::RightHandSides_t::const_iterator it
                 (latestLeaf_.rightHandSides ().begin());
               it!=latestLeaf_.rightHandSides ().end(); ++it) {
            for (ContactState::RightHandSides_t::const_iterator i=currentLeafRhs.begin();
                 i!=currentLeafRhs.end(); ++i) {
              if (it->first == i->first && it->second!=i->second ) {
                //les fonctions sont egale mais pas les rhs
                rhsEqual=false;
                break;
              }
            }
            if (rhsEqual==false){break;
            }
          }

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
              (HPP_DYNAMIC_PTR_CAST(graph::WaypointEdge,connectedEdges[0]));

            if (waypointEdge) {
              assert (latestLeaf_.state ()!=currentState);
              hppDout(info, "Problème avec Waypoints");

              connectStatesByWaypoints
                (waypointEdge, currentLeaf, latestLeaf_,
                 k, validationReport, valid, currentRoadmap, currentState);
            }
            else {
              hppDout(info, "Problème sans Waypoints");

              connectDirectStates
                (currentLeaf, latestLeaf_, validationReport, k, valid,
                 currentState, currentRoadmap);
            }
          }
        }
        storeLeafRoadmap();
      }

      void RMRStar::connectStatesByWaypoints
      (const graph::WaypointEdgePtr_t& waypointEdge,
       const ContactState& currentLeaf, const ContactState& latestLeaf,
       size_type k,
       ValidationReportPtr_t& validationReport, bool valid,
       const core::RoadmapPtr_t& roadmap,
       const graph::StatePtr_t& currentState)
      {
        assert (latestLeaf.state () == waypointEdge->from ());
        assert (currentLeaf.state () == waypointEdge->to ());
        size_type max_iter (20);
        assert (currentState == waypointEdge->to ());
        // Current mapping of right hand sides
        ContactState::RightHandSides_t currentLeafRhs
          (currentLeaf.rightHandSides ());
        // Configuration on current leaf
        ConfigValidationsPtr_t configValidations
          (manipulationProblem_.configValidations ());

        //get waypoints constraints
        std::size_t nWaypoints = waypointEdge->nbWaypoints();
        // log waypoint edge
        hppDout(info,"nWaypoints = "<<nWaypoints);
        for (std::size_t j = 0; j < nWaypoints; ++j) {
          hppDout (info, "waypoint " << j);
          hppDout (info, "  from: "
                   << waypointEdge->waypoint (j)->from ()->name ());
          hppDout (info, "  to  : "
                   << waypointEdge->waypoint (j)->to ()->name ());
        }
        ConfigurationPtr_t q_waypointInter;
        PathPtr_t path, projPath, validPath;
        bool success=false;
        Configuration_t q_prev, q_next;

        // Generate configuration at intersection of both leaves
        q_waypointInter = createInterStateConfig
          (currentLeaf, latestLeaf, configValidations, validationReport,
           valid, currentState);
        if (!q_waypointInter) {
          hppDout (info, "Failed to generate configuration at intersection of "
                   " contact states " << currentState->name () <<
                   " and " << latestLeaf.state ()->name ());
          return;
        }
        size_type wp;
        // Detect on which waypoint state q_waypointInter lies
        for (wp = -1; wp < (size_type) nWaypoints; ++wp) {
          graph::StatePtr_t targetState;
          if (wp == -1) {
            targetState = waypointEdge->waypoint(0)->from ();
          } else {
            targetState = waypointEdge->waypoint(wp)->to ();
          }

          if (targetState->configConstraint ()->configProjector ()->solver ().
              isSatisfied (*q_waypointInter)) {
            hppDout (info, displayConfig (*q_waypointInter) << " in state "
                     << targetState->name ());
            success = true;
            break;
          }
        }
        // Configuration should be in a waypoint state
        assert (success);

        // Project intersection configuration backward along intermediate
        // transitions
        std::vector <PathPtr_t> paths ((std::size_t) nWaypoints);
        size_type i;
        Configuration_t q (*q_waypointInter);
        // q_waypointInter lies on waypoint [wp]->to ()
        for (i = wp; i>=0; --i) {
          graph::EdgePtr_t edge (waypointEdge->waypoint(i));
          graph::StatePtr_t initState (edge->from ());
          //
          // project q on initState -> q_prev
          //
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
            hppDout (info, "Failed to compute waypoint backward ("
                     << i << ").");
            return;
          }
          hppDout (info, displayConfig (q_prev)
                   << " in state " << initState->name ());
          // build path between q_prev and q
          success = edge->build (path, q_prev, q);
          if (!success) {
            hppDout (info, "Failed to a build path between q_prev = "
                     << displayConfig (q_prev));
            hppDout (info, "and q = " << displayConfig (q));
            hppDout (info, "along edge " << edge->name ());
            return;
          }
          // project path - note that constraints are stored in path
          PathProjectorPtr_t pathProjector
            (manipulationProblem_.pathProjector ());
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
        for (i = wp + 1; i < (size_type) nWaypoints; ++i) {
          graph::EdgePtr_t edge (waypointEdge->waypoint(i));
          graph::StatePtr_t goalState (edge->to ());
          //
          // project q on initState -> q_next
          //
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
          hppDout (info, displayConfig (q_next)
                   << " in state " << goalState->name ());
          // build path between q and q_next
          success = edge->build (path, q, q_next);
          if (!success) {
            hppDout (info, "Failed to a build path between q = "
                     << displayConfig (q));
            hppDout (info, "and q_next = " << displayConfig (q_next));
            hppDout (info, "along edge " << edge->name ());
            return;
          }
          // project path - note that constraints are stored in path
          PathProjectorPtr_t pathProjector
            (manipulationProblem_.pathProjector ());
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
                            (problem ().robot ()->configSize (),
                             problem ().robot ()->numberDof ()));
        for (std::vector <PathPtr_t>::const_iterator itPath (paths.begin ());
             itPath != paths.end (); ++itPath) {
          pv->appendPath (*itPath);
        }
        // Add path vector as an edge in the roadmap
        roadmap_->addEdges (node1, node2, pv);
        // Try to connect q_prev with nearest neighbors of latestLeaf
        Nodes_t nearNodes (latestRoadmap_->nearestNodes (pQprev, k));
        for (Nodes_t::const_iterator itNode (nearNodes.begin ());
             itNode != nearNodes.end (); ++itNode) {
          connectConfigToNode (transition_ [latestLeaf.state ()],
                               (*itNode)->configuration (), pQprev);
        }
        // Try to connect q_next with nearest neighbors of currentLeaf
        nearNodes = roadmap->nearestNodes (pQnext, k);
        for (Nodes_t::const_iterator itNode (nearNodes.begin ());
             itNode != nearNodes.end (); ++itNode) {
          connectConfigToNode (transition_ [currentLeaf.state ()],
                               pQnext, (*itNode)->configuration ());
        }
      }

      ////////////////////////////////////////////////////////////////////////

      void RMRStar::connectDirectStates
      (const ContactState& currentLeaf, const ContactState& latestLeaf,
       ValidationReportPtr_t& validationReport, size_type k, bool valid,
       const graph::StatePtr_t& currentState, const core::RoadmapPtr_t& roadmap)
      {
        ConfigValidationsPtr_t configValidations
          (manipulationProblem_.configValidations ());
        ConfigurationPtr_t q_inter = createInterStateConfig
          (currentLeaf, latestLeaf, configValidations, validationReport,
           valid, currentState);
        if (q_inter) {
          graph::EdgePtr_t edgeTransit =transition_[latestLeaf_.state()];
          Nodes_t nearNodes = latestRoadmap_->nearestNodes(q_inter,k);
          core::ConstraintSetPtr_t constraintTRansit =
            edgeTransit->configConstraint();

          for (Nodes_t :: const_iterator itnode = nearNodes.
                 begin(); itnode !=nearNodes.end(); itnode++ ) {
            ConfigurationPtr_t nodeConfig = (*itnode)->configuration();
            connectConfigToNode (edgeTransit,q_inter,nodeConfig);
          }
          graph::EdgePtr_t edge =transition_[currentState];
          Nodes_t nearestNodes = roadmap->nearestNodes(q_inter,k);
          core::ConstraintSetPtr_t constraint =  edge->configConstraint();

          for (Nodes_t :: const_iterator it = nearestNodes.
                 begin(); it != nearestNodes.end(); it++ ) {
            ConfigurationPtr_t nodeConfig = (*it)->configuration();
            connectConfigToNode (edge,q_inter,nodeConfig);
          }
        }
      }

      ///////////////////////////////////////////////////////////////////////

      ConfigurationPtr_t RMRStar::createInterStateConfig
      (const ContactState& currentLeaf, const ContactState& latestLeaf,
       const ConfigValidationsPtr_t& configValidations,
       ValidationReportPtr_t& validationReport, bool valid,
       const graph::StatePtr_t& currentState)
      {
        const NumericalConstraints_t& currentLeafNumericalConstraints
          (currentLeaf.solver ().numericalConstraints());
        // Get the numerical constraints of latest leaf
        const NumericalConstraints_t& latestLeafNumericalConstraints
          (latestLeaf.solver ().numericalConstraints());
        ConfigurationShooterPtr_t shooter
          (manipulationProblem_.configurationShooter ());
        HierarchicalIterative::Status constraintApplied
          (HierarchicalIterative::INFEASIBLE);
        ConfigurationPtr_t q_inter;
        size_type i= 0;
        hppDout(info,"currentState "<<currentState->name());
        hppDout(info,"latestLeaf_ "<<latestLeaf_.state()->name());

        BySubstitution solver (latestLeaf_.solver ());

        assert (latestLeaf_.constraints ());

        //Copy the constraints and the right hand side in the new solver
        for (std::size_t j=0; j<currentLeafNumericalConstraints.size(); j++){
          solver.add (currentLeafNumericalConstraints[j]);
        }
        for (std::size_t j=0; j<latestLeafNumericalConstraints.size(); j++){
          solver.rightHandSideFromConfig (latestLeafNumericalConstraints[j],
                                          latestLeaf_.config());
        }
        for (std::size_t j=0; j<currentLeafNumericalConstraints.size(); j++){
          solver.rightHandSideFromConfig (currentLeafNumericalConstraints[j],
                                          currentLeaf.config ());
        }
        hppDout(info, "solver inter state " << solver);
        hppDout(info, "error threshold " << solver.errorThreshold ());
        size_type max_iter (20);
        while ((valid==false) && i < max_iter)
          {
            //Shoot random configuration
            q_inter= shooter->shoot();
            constraintApplied = solver.solve(*q_inter);
            if (constraintApplied != HierarchicalIterative::SUCCESS) {
              valid=false;
              ++i;
              continue;
            }
            valid = configValidations->validate (*q_inter,
                                                 validationReport);
            hppDout (info,"q_inter="<< displayConfig (*q_inter));
            i++;
          }
        hppDout (info,"i="<<i);
        hppDout (info,"connect states "<<latestLeaf_.state()->name()<<
                 "  currentState "<<currentState->name());
        hppDout (info,"q_inter="<< displayConfig (*q_inter));
        hppDout (info,"constraintApplied="<<constraintApplied);

        //test constraints have been applied to q_inter
        if (!valid) {
          assert (i == max_iter);
          hppDout (info,"i reached max_iter, not any connect node have been found");
          q_inter = ConfigurationPtr_t ();
          //	assert (i!=max_iter);
        }

        return q_inter;
      }
      ////////////////////////////////////////////////////////////////////////

      void RMRStar::connectConfigToNode
      (const graph::EdgePtr_t& edge, const ConfigurationPtr_t& q1,
       const ConfigurationPtr_t& configuration)
      {
        PathProjectorPtr_t pathProjector
          (manipulationProblem_.pathProjector ());
        //connect the interRoadmap to the nodeInter
        PathPtr_t validPath, path, projpath;
        NodePtr_t nodeConnect=
          roadmap_->addNode (ConfigurationPtr_t (new Configuration_t(*q1)));

        hppDout (info, " node1 " << displayConfig(*configuration));
        hppDout (info, " node2 " <<  displayConfig(*q1));

        core::SteeringMethodPtr_t sm= edge->steeringMethod();

        hppDout (info, displayConfig (*q1) << " in state "
                 << edge->from ()->name ());
        assert(edge->from()->contains(*q1));
        hppDout (info, displayConfig (*q1) << " in state "
                 << edge->state ()->name ());
        assert(edge->state()->contains(*q1));
        hppDout (info, displayConfig (*configuration) << " in state "
                 << edge->to ()->name ());
        assert(edge->to()->contains(*configuration));
        hppDout (info, displayConfig (*configuration) << " in state "
                 << edge->state ()->name ());
        assert(edge->state()->contains(*configuration));

        path =(*sm)(*q1,*configuration);

        if (path==NULL){
          hppDout (info,"path=NULL");
        }
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

      void RMRStar::storeRhs ()
      {
        const BySubstitution& solver
          (latestLeaf_.solver ());
        core::NumericalConstraints_t constraints =solver. numericalConstraints();

        for (std::size_t i=0 ; i<constraints.size() ; i++)
          {
            constraints::ImplicitPtr_t function = constraints[i];
            constraints::vectorOut_t rhs= function->nonConstRightHandSide();
            bool success = false;
            success=solver.getRightHandSide(function,rhs);
            assert (!rhs.hasNaN ());
            assert (success);

            bool null=rhsNull(rhs);
            bool alreadyInMap=false;

            if (!null){
              for (RightHandSides_t::const_iterator it=rightHandSides_.begin() ;
                   it!= rightHandSides_.end() ; ++it)
                {
                  if (function==(it->first) &&
                      rhs.isApprox(it->second,solver.errorThreshold()))
                    {
                      alreadyInMap=true;
                    }
                }

              if(alreadyInMap==false){
                rightHandSides_.insert
                  (std::pair<constraints::ImplicitPtr_t,constraints::vectorIn_t>
                   (function,rhs));
              }
            }
          }
      }
      //////////////////////////////////////////////////////////////////////////
      bool RMRStar::rhsNull (vector_t rhs)
      {
        bool null = true;

        for (size_type i=0; i<rhs.size(); ++i )
          {
            if (rhs[i]!=0)
              {
                null=false;
              }
          }
        return null;
      }

      //////////////////////////////////////////////////////////////////////////
      void RMRStar::storeLeafRoadmap ()
      {

        //copy the problem and pass the edge contraints
        core::Problem p (problem ());
        p.initConfig (ConfigurationPtr_t (new Configuration_t
                                          (latestLeaf_.config ())));
        p.steeringMethod (edgeSteeringMethod_->copy ());
        p.constraints
          (core::ConstraintSet::createCopy(latestLeaf_.constraints ()));
        p.pathValidation(transition_[latestLeaf_.state()]->pathValidation());
        core::RoadmapPtr_t r = core::Roadmap::create(p.distance(),p.robot());
        r->clear();

        // Create a copy of the nodes of interRoadmap that we store in the
        // leafRoadmaps_
        for (Nodes_t::const_iterator itnode =latestRoadmap_->nodes().begin();
             itnode !=latestRoadmap_->nodes().end(); itnode++ )
          {
            r->addNode(ConfigurationPtr_t
                       (new Configuration_t(*(*itnode)->configuration())));	}

        RMRStar::ProblemAndRoadmap_t pbRoadmap (p,r) ;

        //complete the map with the association ContactState/ProblemAndRoadmap
        leafRoadmaps_.insert
          (std::pair<ContactState,RMRStar::ProblemAndRoadmap_t>
           (latestLeaf_,pbRoadmap));
      }
      ///////////////////////////////////////////////////////////////////////////

      pinocchio::value_type RMRStar::biggestThreshold
      ( const BySubstitution& solver1, const BySubstitution& solver2)
      {
        pinocchio::value_type threshold1 = solver1.errorThreshold();
        pinocchio::value_type threshold2 = solver2.errorThreshold();

        if (threshold1 < threshold2)
          {
            return threshold2;
          }

        if (threshold2 < threshold1)
          {
            return threshold1;
          }
        else
          {
            return threshold1;
          }
      }
      ///////////////////////////////////////////////////////////////////////////

      void RMRStar::startSolve ()
      {
        PathPlanner::startSolve ();
        graph_ =manipulationProblem_.constraintGraph ();
        computeTransitionMap ();
        setRhsFreq_=problem().getParameter("RMR*/SetRhsFreq").intValue();
        counter_=0;

        //Set parameter and create interRoadmap
        latestRoadmap_ = core::Roadmap::create
          (manipulationProblem_.distance(),manipulationProblem_.robot());
        latestRoadmap_->clear();
        ConfigurationPtr_t q (roadmap()->initNode ()->configuration());

        //Set init ContactState
        graph::StatePtr_t stateInit=
          graph_->getState(*(q));
        graph::EdgePtr_t loop_edge = transition_[stateInit];
        core::ConstraintSetPtr_t edgeConstraints =loop_edge->pathConstraint();
        ContactState contactStateInit
          (stateInit, *q, edgeConstraints);
        edgeSteeringMethod_ = loop_edge -> steeringMethod();
        latestLeaf_ = contactStateInit;

        buildRoadmap();
        copyRoadmapIntoGlobal (latestRoadmap_);
        storeLeafRoadmap();
        storeRhs();

        for (core::NodeVector_t::const_iterator itn
               (roadmap_->goalNodes ().begin ());
             itn != roadmap_->goalNodes ().end (); ++itn) {

          latestRoadmap_ -> clear();

          //Set final ContactState
          q=(*itn)->configuration();

          graph::StatePtr_t stateGoal=graph_->getState(*q);
          loop_edge = transition_[stateGoal];
          edgeConstraints =loop_edge->pathConstraint();

          ContactState contactStateGoal
            (stateGoal,*q, edgeConstraints);

          latestLeaf_=contactStateGoal;

          buildRoadmap();
          copyRoadmapIntoGlobal (latestRoadmap_);
          connectRoadmap ();
          storeRhs();
        }
        step_=BUILD_ROADMAP;
      }

      ////////////////////////////////////////////////////////////////////////////
      void RMRStar::oneStep ()
      {
        switch (step_)
          {

          case BUILD_ROADMAP:

            latestLeaf_ = sampleContact();
            buildRoadmap();
            copyRoadmapIntoGlobal (latestRoadmap_);
            storeRhs();

            step_=CONNECT_ROADMAPS;
            if (counter_==setRhsFreq_)
              {
                counter_=0;
              }
            counter_++;
            break;

          case CONNECT_ROADMAPS:

            connectRoadmap();
            step_= BUILD_ROADMAP;

            break;
          }
      }
      ////////////////////////////////////////////////////////////////////////////

      RMRStar::RMRStar (const core::Problem& problem,
                        const core::RoadmapPtr_t& roadmap) :
        core::PathPlanner (problem, roadmap),
        transition_ (), manipulationProblem_
        (static_cast <const manipulation::Problem& > (problem)),
        roadmap_ (HPP_STATIC_PTR_CAST (manipulation::Roadmap, roadmap))
      {
#ifndef NDEBUG
        dynamic_cast <const manipulation::Problem& > (problem);
        assert (HPP_DYNAMIC_PTR_CAST (manipulation::Roadmap, roadmap));

#endif
      }
      ////////////////////////////////////////////////////////////////////////////
      // ----------- Declare parameters ------------------------------------- //
      using core::Parameter;
      using core::ParameterDescription;
      HPP_START_PARAMETER_DECLARATION(RMRStar)
      core::Problem::declareParameter
      (ParameterDescription (Parameter::INT, "RMR*/numberOfConnectNodes",			  "The desired number of the nodes we try to connect between the intersection node and the roadmaps.", Parameter((size_type)3)));

      core::Problem::declareParameter(ParameterDescription (Parameter::INT,
                                                            "RMR*/SetRhsFreq",
                                                            "The desired number of the frequency of set configuration map build.If it's 0 it's never mannually set, if it's 100 it's 99% of time mannually set", Parameter((size_type)100)));
      core::Problem::declareParameter (ParameterDescription
                                       (Parameter::INT,
                                        "RMR*/nbTryRandomConfig",
                                        "The number of attempt to sample a "
                                        "valid random configuration in a given "
                                        "state.", Parameter ((size_type)100)));
      HPP_END_PARAMETER_DECLARATION(RMRStar)
    } // namespace pathPlanner
  } // namespace manipulation
} // namespace hpp
