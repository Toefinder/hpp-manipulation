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
#include <hpp/core/roadmap.hh>


#include "hpp/manipulation/problem.hh"
#include "hpp/manipulation/roadmap.hh"

namespace hpp {
  namespace manipulation {
    namespace pathPlanner {

      typedef rmrStar::ContactState ContactState;
      typedef core::Nodes_t Nodes_t;
      typedef core::NodePtr_t NodePtr_t;
      typedef core::Edges_t Edges_t;
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

      ContactState RMRStar::sampleContact ()
      {
        bool valid =false;
        ConfigurationShooterPtr_t shooter
          (manipulationProblem_.configurationShooter ());
        std::size_t nbStates (transition_.size ());
        // Shoot random number
        std::size_t i_rand=rand() % nbStates;

        // Grap random state of the graph
        const graph::StatePtr_t s_rand =
          RMRStar::extract_keys(transition_)[i_rand];

        bool stateValid = false;
        bool validRhs=false;
        ConfigurationPtr_t q_rhs;

        while (!stateValid){
          core:: ValidationReportPtr_t validationReport;
          core::ConfigValidationsPtr_t configValidations
            (problem ().configValidations ());
          int i=0;
          int j=0;
          std::vector <constraints::vector_t> rhsOfTheFunctionAlreadyVisited;

          HierarchicalIterative::Status constraintApplied=
            HierarchicalIterative::INFEASIBLE;

          ConstraintSetPtr_t stateConfig =
            graph_->configConstraint(transition_[s_rand]);
          NumericalConstraints_t numConstraints =
            stateConfig->configProjector () -> numericalConstraints();

          constraints::solver::BySubstitution solver
            (stateConfig->configProjector ()->solver ());
          std::size_t randomSkip = rand() % numConstraints.size();

          //Try to detect the function already visited and get their
          //Right Hand side to set it to the new state

          for (std::size_t i=0; i<numConstraints.size() ;i++)
            {
              if (counter_==setRhsFreq_ && i==randomSkip)
                {
                  while (validRhs==false && j<=100)
                    {
                      //Shoot random configuration
                      q_rhs= shooter->shoot();
                      bool constApply =s_rand->configConstraint()->apply(*q_rhs);

                      if (!constApply) {
                        validRhs=false;
                        j++;
                        continue;
                      }
                      validRhs =
                        configValidations->validate (*q_rhs, validationReport);
                      j++;
                      hppDout(info,"q_rand rhs random"<<displayConfig(*q_rhs));
                    }

#ifndef NDEBUG
                  bool set=
#endif
                  solver.rightHandSideFromConfig (numConstraints[i],*q_rhs);
                  assert (set);
                  continue;
                }

              for (RightHandSides_t::const_iterator it=rightHandSides_.begin() ;
                   it!= rightHandSides_.end() ; ++it)
                {
                  if (it->first->functionPtr()->name()==
                      numConstraints[i]->functionPtr()->name()){

                    rhsOfTheFunctionAlreadyVisited.push_back(it->second);
                  }
                }

              if (!rhsOfTheFunctionAlreadyVisited.empty()){

                std::size_t indice_rand=rand()%rhsOfTheFunctionAlreadyVisited.size();
                constraints::vector_t Rhs_rand=rhsOfTheFunctionAlreadyVisited[indice_rand];
                bool success=solver.rightHandSide(numConstraints[i],Rhs_rand);
                assert(success);
                rhsOfTheFunctionAlreadyVisited.clear();
              }

            }
          hppDout(info,"solver"<<solver);

          while (valid==false && i<=100)
            {
              //Shoot random configuration
              q_rand_= shooter->shoot();
              constraintApplied =solver.solve(*q_rand_);
              hppDout(info, "constraintApply"<<constraintApplied);

              if (constraintApplied != HierarchicalIterative::SUCCESS) {
                valid=false;
                i++;
                continue;
              }
              valid = configValidations->validate (*q_rand_, validationReport);
              i++;
              hppDout(info,"q_rand sampleContact"<<displayConfig(*q_rand_));
            }

          if (i==101){
            hppDout (info,"fail to find a random configuration in state "
                     << s_rand->name ());
            stateValid=false;
            i=0;
          }

          else {stateValid=true;}

        }
        //Get loop_edge constraints
        graph::EdgePtr_t loop_edge = transition_[s_rand];
        core::ConstraintSetPtr_t edgeConstraints =loop_edge->pathConstraint();

        // recovery of the edge steering method
        edgeSteeringMethod_ = loop_edge -> steeringMethod();

        // get right hand side of constraint from q_rand
        ContactState contactState (s_rand, *q_rand_, edgeConstraints);

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
        p.initConfig(q_rand_);
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
        core::PathPtr_t projpath;
        core:: ValidationReportPtr_t validationReport;
        size_type k=problem().getParameter("RMR*/numberOfConnectNodes").intValue();

        core::PathPtr_t path;

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
          constraints::solver::BySubstitution latestLeafSolver
            (latestLeaf_.solver ());

          // Get solver projecting on current leaf
          constraints::solver::BySubstitution currentLeafSolver
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
                 k, validationReport, valid, projpath, currentRoadmap,
                 currentState);
            }
            else {
              hppDout(info, "Problème sans Waypoints");

              connectDirectStates
                (currentLeaf, latestLeaf_, validationReport, k,
                 path, projpath, valid, currentState,
                 currentRoadmap);
            }
          }
        }
        storeLeafRoadmap();
      }

      void RMRStar::connectStatesByWaypoints
      (const graph::WaypointEdgePtr_t& waypointEdge,
       const ContactState& currentLeaf, const ContactState& latestLeaf,
       size_type k,
       core::ValidationReportPtr_t& validationReport, bool valid,
       core::PathPtr_t& projpath,
       const core::RoadmapPtr_t& roadmap,
       const graph::StatePtr_t& currentState)
      {
        size_type max_iter (20);
        assert (currentState == waypointEdge->to ());
        // Current mapping of right hand sides
        ContactState::RightHandSides_t currentLeafRhs
          (currentLeaf.rightHandSides ());
        HierarchicalIterative::Status constraintApplied
          (HierarchicalIterative::INFEASIBLE);
        // Configuration on current leaf
        core::ConfigValidationsPtr_t configValidations
          (manipulationProblem_.configValidations ());

        //get waypoints constraints
        std::size_t nbOfWaypoints = waypointEdge -> nbWaypoints();
        ConfigurationPtr_t q_waypointInter;
        core::PathPtr_t validPath;
        core::PathPtr_t path;
        int i=0;
        hppDout(info,"nbOfWaypoints = "<<nbOfWaypoints);
        bool succeed=false;

        //////////////////////////////////////////////////////////////
        //Find intersec waypoint
        //////////////////////////////////////////////////////////////

        for (size_type Waypoint=0; Waypoint < (size_type) nbOfWaypoints;
             Waypoint++) {
          const char *intersec ="intersec";
          const char *waypointName=
            (waypointEdge->waypoint(Waypoint)->from()->name()).c_str();
          const char * waypointIntersec= std::strstr (waypointName,intersec);

          if (waypointIntersec) {
            while (succeed ==false) {
              valid=false;
              q_waypointInter = createInterStateNode
                (currentLeaf, latestLeaf, configValidations, validationReport,
                 valid, currentState);

              if (q_waypointInter) {
                //Copy of  waypointInter
                ConfigurationPtr_t q_waypoint (new Configuration_t
                                               (*q_waypointInter));

                ConfigurationPtr_t q_nextWaypoint
                  (new Configuration_t(*q_waypointInter));

                ////////////////////////////////////////////////////////
                ///Connect the waypoints from intersec to the first one
                ////////////////////////////////////////////////////////

                for (std::size_t w=(std::size_t) (Waypoint-1); w>=1; --w) {
                  hppDout(info,"w= "<<w);
                  graph::EdgePtr_t forwardEdge =
                    waypointEdge->waypoint(w);
                  graph::Edges_t edges =
                    graph_->getEdges(currentState, latestLeaf_.state());
                  assert(edges.size()==1);

                  graph::WaypointEdgePtr_t backwardWaypointEdge =
                    HPP_DYNAMIC_PTR_CAST(graph::WaypointEdge, edges[0]);
                  assert(backwardWaypointEdge);

                  graph::EdgePtr_t Edge =
                    backwardWaypointEdge->waypoint(nbOfWaypoints-w);
                  core::ConstraintSetPtr_t constraintEdge=
                    Edge->configConstraint();
                  constraints::solver::BySubstitution solver
                    (constraintEdge->configProjector ()->solver ());

                  for (std::size_t j=0;
                       j<solver.numericalConstraints().size(); j++) {
                    for (ContactState::RightHandSides_t::const_iterator it
                           (latestLeaf_.rightHandSides().begin());
                         it!=latestLeaf_.rightHandSides().end() ; ++it) {
                      if (solver.numericalConstraints()[j] == it->first) {
                        solver.rightHandSide (solver.numericalConstraints()[j],
                                              it->second);
                      }
                    }
                    for (ContactState::RightHandSides_t::const_iterator i=currentLeafRhs.begin();
                         i!=currentLeafRhs.end(); ++i) {
                      if (solver.numericalConstraints()[j]== i->first) {
                        solver.rightHandSide
                          (solver.numericalConstraints()[j], i->second);
                      }
                    }
                  }
                  i=0;
                  valid=false;

                  while ((valid==false) && i<max_iter) {
                    constraintApplied = solver.solve(*q_nextWaypoint);

                    if (constraintApplied != HierarchicalIterative::SUCCESS) {
                      valid=false;
                      ++i;
                      continue;
                    }
                    valid = configValidations->validate
                      (*q_nextWaypoint,validationReport);
                    i++;
                  }

                  if (i==max_iter) {
                    hppDout (info, "i reached max_iter, not any connect node "
                             "have been found");
                    hppDout(info, "i = "<<i);
                    hppDout(info, "success = "<<succeed);
                  } else {
                    succeed=true;
                    connectConfigToNode (forwardEdge, path,
                                         projpath, q_nextWaypoint, q_waypoint);

                    *q_waypoint=*q_nextWaypoint;
                  }
                }

                ////////////////////////////////////////////////////////
                ///Connect first Waypoint to the interRoadmap
                ///////////////////////////////////////////////////////

                if (i!=max_iter) {
                  Nodes_t nearNodes =
                    latestRoadmap_->nearestNodes(q_waypoint,k);

                  for (Nodes_t :: const_iterator itnode = nearNodes.
                         begin(); itnode !=nearNodes.end(); itnode++) {
                    ConfigurationPtr_t nodeConfig =
                      (*itnode)->configuration();

                    connectConfigToNode (waypointEdge->waypoint(0), path, projpath, nodeConfig,
                                         q_waypoint);
                  }
                }
                //////////////////////////////////////////////////////
                //Connect the waypoints from intersec to the last one
                ///////////////////////////////////////////////////////
                *q_waypoint=*q_waypointInter;
                *q_nextWaypoint=*q_waypointInter;

                for (std::size_t w=Waypoint; w<nbOfWaypoints; w++) {
                  hppDout(info,"w= "<<w);
                  graph::EdgePtr_t Edge = waypointEdge->waypoint(w);
                  core::ConstraintSetPtr_t constraintEdge
                    (Edge->configConstraint());
                  constraints::solver::BySubstitution solver
                    (constraintEdge->configProjector ()->solver ());

                  for (std::size_t j=0; j<solver.numericalConstraints().size();
                       j++) {
                    for (ContactState::RightHandSides_t::const_iterator it
                           (latestLeaf_.rightHandSides().begin());
                         it!=latestLeaf_.rightHandSides().end() ; ++it) {
                      if (solver.numericalConstraints()[j] == it->first) {
                        solver.rightHandSide (solver.numericalConstraints()[j],
                                              it->second);
                      }
                    }
                    for (ContactState::RightHandSides_t::const_iterator i=currentLeafRhs.begin();
                         i!=currentLeafRhs.end(); ++i) {
                      if (solver.numericalConstraints()[j] == i->first) {
                        solver.rightHandSide (solver.numericalConstraints()[j],
                                              i->second);
                      }
                    }
                  }

                  i=0;
                  valid=false;

                  while ((valid==false) && i<max_iter) {
                    constraintApplied = solver.solve(*q_nextWaypoint);

                    if (constraintApplied != HierarchicalIterative::SUCCESS) {
                      valid=false;
                      ++i;
                      continue;
                    }
                    valid = configValidations->validate (*q_nextWaypoint,
                                                         validationReport);
                    i++;
                  }

                  if (i==max_iter||!succeed) {
                    hppDout (info,"i reached max_iter, not any connect node "
                             "have been found");
                    hppDout(info, "i = "<<i);
                    hppDout(info, "succeed = "<<succeed);

                  } else {
                    connectConfigToNode (Edge, path, projpath,
                                         q_waypoint, q_nextWaypoint);

                    *q_waypoint=*q_nextWaypoint;
                  }
                }

                ///////////////////////////////////////////////////////
                ///Connect the last waypoint to the Roadmap in the table
                ////////////////////////////////////////////////////////

                if (i!=max_iter && succeed) {
                  Nodes_t nearestNodes (roadmap->nearestNodes(q_waypoint,k));

                  for (Nodes_t :: const_iterator itnode (nearestNodes.begin());
                       itnode!=nearestNodes.end(); ++itnode) {
                    ConfigurationPtr_t nodeConfig ((*itnode)->configuration());

                    connectConfigToNode (waypointEdge->waypoint(nbOfWaypoints),
                                         path, projpath,
                                         q_waypoint,nodeConfig);
                  }
                }

              }//end if q_inter=q_init
              else {
                succeed=true;
              }
            }//end while not succeed

            break;
          }//end if waypoint intersec
        }//end for waypoint
      }

      ////////////////////////////////////////////////////////////////////////

      void RMRStar::connectDirectStates
      (const ContactState& currentLeaf, const ContactState& latestLeaf,
       core::ValidationReportPtr_t& validationReport, size_type k,
       core::PathPtr_t& path, core::PathPtr_t& projpath, bool valid,
       const graph::StatePtr_t& currentState, const core::RoadmapPtr_t& roadmap)
      {
        core::ConfigValidationsPtr_t configValidations
          (manipulationProblem_.configValidations ());
        ConfigurationPtr_t q_inter = createInterStateNode
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
            connectConfigToNode (edgeTransit,path,projpath,q_inter,nodeConfig);
          }
          graph::EdgePtr_t edge =transition_[currentState];
          Nodes_t nearestNodes = roadmap->nearestNodes(q_inter,k);
          core::ConstraintSetPtr_t constraint =  edge->configConstraint();

          for (Nodes_t :: const_iterator it = nearestNodes.
                 begin(); it !=nearestNodes.end(); it++ ) {
            ConfigurationPtr_t nodeConfig = (*it)->configuration();
            connectConfigToNode (edge,path,projpath,q_inter,nodeConfig);
          }
        }
      }

      ///////////////////////////////////////////////////////////////////////

      ConfigurationPtr_t RMRStar::createInterStateNode
      (const ContactState& currentLeaf, const ContactState& latestLeaf,
       const core::ConfigValidationsPtr_t& configValidations,
       core::ValidationReportPtr_t& validationReport, bool valid,
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

        constraints::solver::BySubstitution solver (latestLeaf_.solver ());

        assert (latestLeaf_.constraints ());

        //Copy the constraints and the right hand side in the new solver
        for (std::size_t j=0; j<currentLeafNumericalConstraints.size(); j++){
          solver.add(currentLeafNumericalConstraints[j]);
        }
        for (std::size_t j=0; j<latestLeafNumericalConstraints.size(); j++){
          solver.rightHandSideFromConfig (latestLeafNumericalConstraints[j],
                                          latestLeaf_.config());
        }
        for (std::size_t j=0; j<currentLeafNumericalConstraints.size(); j++){
          solver.rightHandSideFromConfig (currentLeafNumericalConstraints[j],
                                          currentLeaf.config ());
        }
        hppDout(info,"solver inter state) "<<solver);
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
      (const graph::EdgePtr_t& edge, core::PathPtr_t&  path,
       core::PathPtr_t& projpath,
       const ConfigurationPtr_t& q1,
       const ConfigurationPtr_t& configuration)
      {
        PathProjectorPtr_t pathProjector
          (manipulationProblem_.pathProjector ());
        //connect the interRoadmap to the nodeInter
        core::PathPtr_t validPath;
        NodePtr_t nodeConnect=
          roadmap_->addNode (ConfigurationPtr_t (new Configuration_t(*q1)));

        hppDout (info, " node1 " << displayConfig(*configuration));
        hppDout (info, " node2 " <<  displayConfig(*q1));

        core::SteeringMethodPtr_t sm= edge->steeringMethod();

        assert(edge->from()->contains(*q1));
        assert(edge->state()->contains(*q1));
        assert(edge->to()->contains(*configuration));
        assert(edge->state()->contains(*configuration));

        path =(*sm)(*q1,*configuration);

        if (path==NULL){
          hppDout (info,"path=NULL");
        }
        else {
          projpath = path;
          if ((!pathProjector) || (pathProjector->apply(path,projpath)))
            {

              PathValidationPtr_t pathValidation ( edge->pathValidation());
              PathValidationReportPtr_t report;
              bool valid =
                pathValidation->validate (projpath, false, validPath, report);

              if (valid){
                NodePtr_t node=
                  roadmap_->addNode(ConfigurationPtr_t
                                    (new Configuration_t(*configuration)));

                roadmap_->addEdges (nodeConnect,node,projpath);
                assert (projpath->initial () == *(nodeConnect->configuration()));
                assert (projpath->end () == *(node->configuration()));
                hppDout (info,"edge created ");
              }
              else {hppDout(info, "path not valid");}
            }
          else {
            hppDout(info, "path not OK");}
        }
      }
      ///////////////////////////////////////////////////////////////////////////

      void RMRStar::storeRhs ()
      {
        const constraints::solver::BySubstitution& solver
          (latestLeaf_.solver ());
        core::NumericalConstraints_t constraints =solver. numericalConstraints();

        for (std::size_t i=0 ; i<constraints.size() ; i++)
          {
            constraints::ImplicitPtr_t function = constraints[i];
            constraints::vectorOut_t rhs= function->nonConstRightHandSide();
            bool success = false;
            success=solver.getRightHandSide(function,rhs);

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
        p.initConfig(q_rand_);
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
      ( constraints::solver::BySubstitution solver1,
        constraints::solver::BySubstitution solver2)
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
        q_rand_=roadmap()->initNode ()->configuration();

        //Set init ContactState
        graph::StatePtr_t stateInit=
          graph_->getState(*(q_rand_));
        graph::EdgePtr_t loop_edge = transition_[stateInit];
        core::ConstraintSetPtr_t edgeConstraints =loop_edge->pathConstraint();
        ContactState contactStateInit
          (stateInit, *q_rand_, edgeConstraints);
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
          q_rand_=(*itn)->configuration();

          graph::StatePtr_t stateGoal=graph_->getState(*q_rand_);
          loop_edge = transition_[stateGoal];
          edgeConstraints =loop_edge->pathConstraint();

          ContactState contactStateGoal
            (stateGoal,*q_rand_, edgeConstraints);

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
      HPP_END_PARAMETER_DECLARATION(RMRStar)
    } // namespace pathPlanner
  } // namespace manipulation
} // namespace hpp
