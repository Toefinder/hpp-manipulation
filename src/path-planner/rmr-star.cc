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

#include <deque>
#include "hpp/manipulation/path-planner/rmr-star.hh"

#include <hpp/pinocchio/configuration.hh>

#include <hpp/constraints/solver/by-substitution.hh>

#include <hpp/core/configuration-shooter.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/config-projector.hh>
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

      // State shooter
      //
      // Randomly shoot a state among those provided as input
      class StateShooter
      {
      public:
        StateShooter (const graph::States_t& states) :
          states_ (states)
        {
        }
        graph::StatePtr_t shoot () const
        {
          // Shoot random number among minimal dimension states
          std::size_t i_rand;
          i_rand=rand() % states_.size ();
          return states_ [i_rand];
        }
      private:
        graph::States_t states_;
      }; // class StateShooter

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

      void RMRStar::initialize ()
      {
        // Build vector of all states and of states with loop
        size_type minDimension (std::numeric_limits <size_type>::max ());
        const std::size_t nbComp = graph_->nbComponents();
        //create an indexed table with a node as key and the loop edge as value
        for (std::size_t i=0; i<nbComp; ++i) {
          const graph::GraphComponentPtr_t graphComp (graph_->get(i));
          graph::EdgePtr_t edge (HPP_DYNAMIC_PTR_CAST(graph::Edge, graphComp));
          if(edge) {
            if (edge->from() == edge->to()) {
              transition_ [edge->from()] = edge;
              const BySubstitution& solver
                (edge->pathConstraint ()->configProjector ()->solver());
              statesWithLoops_.push_back (edge->from ());
              // Compute minimal reduced dimension among loop edges
              size_type rd (solver.reducedDimension ());
              if (rd < minDimension) minDimension = rd;
              // Initialize map of right hand sides with empty vectors
              const NumericalConstraints_t& constraints (solver.constraints ());
              for (NumericalConstraints_t::const_iterator it
                     (constraints.begin ()); it != constraints.end (); ++it) {
                const std::string& name ((*it)->function ().name ());
                if (rightHandSides_.find (name) == rightHandSides_.end ()) {
                  rightHandSides_[name] =
                    std::vector <constraints::vector_t> ();
                }
              }
            }
          } else {
            graph::StatePtr_t state (HPP_DYNAMIC_PTR_CAST
                                     (graph::State, graphComp));
            if (state) {
              index_ [state] = allStates_.size ();
              allStates_.push_back (state);
            }
          }
        }
        // Total number of states
        size_type nStates ((size_type) allStates_.size ());
        // Store states with loop transitions of minimal reduced dimension
        for (graph::States_t::const_iterator it (statesWithLoops_.begin ());
             it != statesWithLoops_.end (); ++it) {
          if (transition_ [*it]->pathConstraint ()->configProjector ()->
              solver ().reducedDimension () == minDimension) {
            statesToSample_.push_back (*it);
          }
        }
        // Set state shooter
        stateShooter_ = StateShooterPtr_t (new StateShooter (statesToSample_));

        // Create matrix of state inclusions. Only states with loop transitions
        // are considered
        stateInclusion_.resize (nStates, nStates);
        stateInclusion_.fill (false);
        for (std::size_t i1=0; i1 < (std::size_t) nStates; ++i1) {
          const graph::StatePtr_t& s1 (allStates_ [i1]);
          assert (s1);
          for (std::size_t i2=0; i2 < (std::size_t) nStates; ++i2) {
            const graph::StatePtr_t& s2 (allStates_ [i2]);
            assert (s2);
            if (s1->configConstraint ()->configProjector ()->solver ().
                definesSubmanifoldOf
                (s2->configConstraint ()->configProjector ()->solver ())) {
              stateInclusion_ (i1, i2) = true;
              hppDout (info, "\"" << s1->name () << "\"");
              hppDout (info, "subset of");
              hppDout (info, "\"" << s2->name () << "\"");
              hppDout (info, "");
            }
          }
        }
        // Create matrix of state intersections
        stateIntersection_.resize (allStates_.size (),
                                   allStates_.size ());
        for (std::size_t i1=0; i1<allStates_.size (); ++i1) {
          const graph::StatePtr_t& s1 (allStates_ [i1]);
          stateIntersection_ (i1, i1) = s1;
          for (std::size_t i2=i1+1; i2<allStates_.size (); ++i2) {
            const graph::StatePtr_t& s2 (allStates_ [i2]);
            for (std::size_t i3=0; i3<allStates_.size (); ++i3) {
              const graph::StatePtr_t& s3 (allStates_ [i3]);
              if (stateInclusion_ (i3, i1) && stateInclusion_ (i3, i2)) {
                stateIntersection_ (i1, i2) = s3;
                stateIntersection_ (i2, i1) = s3;
                hppDout (info, "\"" << s1->name () << "\"");
                hppDout (info, "inter");
                hppDout (info, "\"" << s2->name () << "\"");
                hppDout (info, "=\"" << s3->name () << "\"");
              }
            }
          }
        }
        // Initialize map of leaves to explore
        for (graph::States_t::const_iterator it = statesWithLoops_.begin ();
             it != statesWithLoops_.end (); ++it) {
          leaves_ [*it] = ContactStates_t ();
        }
      }

      //////////////////////////////////////////////////////////////////////////

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
        HierarchicalIterative::Status status;
        size_type nInfeasible = 0;
        while (!valid && j <= maxNbTry && 4*nInfeasible <= maxNbTry) {
          // shoot random configuration
          q = shooter->shoot();
          // project it on state
          status = solver.solve (*q);
          if (status == HierarchicalIterative::INFEASIBLE) {
            ++nInfeasible;
          }
          if (status == HierarchicalIterative::SUCCESS) {
            // until it is valid
            ValidationReportPtr_t validationReport;
            valid = configValidations->validate (*q, validationReport);
          }
          ++j;
        } // while (!valid && j <= maxNbTry)
        assert (!valid || status == HierarchicalIterative::SUCCESS);
        return valid;
      }


      void RMRStar::addConfigToLeafRoadmap (const ContactState& contactState,
                                            const ConfigurationPtr_t& q)
      {
        LeafRoadmaps_t::iterator it (leafRoadmaps_.find (contactState));
        if (it == leafRoadmaps_.end ()) {
          createLeafRoadmap (contactState, q);
        } else {
          ProblemAndRoadmap_t& pr (it->second);
          core::RoadmapPtr_t& roadmap (pr.second);
          roadmap->addNode (q);
        }
      }

      static bool belongsToLeaf (const Configuration_t& q,
                                 const ContactState& contactState)
      {
        vector_t error (contactState.solver ().errorSize ());
        bool res (contactState.solver ().isSatisfied (q, error));
        hppDout (info, contactState.solver ());
        hppDout (info, "state \"" << contactState.state ()->name () << "\"");
        hppDout (info, "contact state config: " << displayConfig (contactState.config ()));
        hppDout (info, "config: " << displayConfig (q));
        hppDout (info, "error: " << displayConfig (error));
        hppDout (info, "norme of error:" << error.norm ());
        for (NumericalConstraints_t::const_iterator it
               (contactState.solver ().numericalConstraints ().begin ());
             it != contactState.solver ().numericalConstraints ().end ();
             ++it) {
          vector_t error ((*it)->function ().outputSpace ()->nv ());
          bool found;
          contactState.solver ().isConstraintSatisfied ((*it), q, error, found);
          assert (found);
          if (error.norm () > contactState.solver ().errorThreshold ()) {
            hppDout (info, (*it)->function ().name () << ": " << error.norm ());
          }
        }
        return res;
      }

      static RMRStar::ContactStates_t::const_iterator findLeafContainingConfig
      (const Configuration_t& q, const RMRStar::ContactStates_t& contactStates)
      {
        for (RMRStar::ContactStates_t::const_iterator it =
               contactStates.begin (); it != contactStates.end (); ++it) {
          if (belongsToLeaf (q, *it)) return it;
        }
        return contactStates.end ();
      }

      void RMRStar::sampleIntersectionStates ()
      {
        size_type maxNbTry (problem ().getParameter
                            ("RMR*/nbTryRandomConfig").intValue ());
        ConfigurationShooterPtr_t shooter
          (manipulationProblem_.configurationShooter ());
        ConfigValidationsPtr_t configValidations
          (problem ().configValidations ());
        // Loop over intersection states
        for (graph::States_t::const_iterator it1 (statesToSample_.begin ());
             it1 != statesToSample_.end (); ++it1) {
          const graph::StatePtr_t& s1 (*it1); size_type i1 (index_ [s1]);
          for (graph::States_t::const_iterator it2 (it1+1);
               it2 != statesToSample_.end (); ++it2) {
            const graph::StatePtr_t& s2 (*it2); size_type i2 (index_ [s2]);
            graph::StatePtr_t interState (stateIntersection_ (i1, i2));
            if (!interState) {
              continue;
            }
            assert (interState);
            bool validConfig = false;
            ConfigurationPtr_t q;
            // Build solver corresponding to the intersection of loop edges
            // of states.
            BySubstitution solver (transition_ [s1]->pathConstraint ()->
                                   configProjector ()->solver ());
            solver.merge (transition_ [s2]->pathConstraint ()->
                          configProjector ()->solver ());
            NumericalConstraints_t intersectionConstraints
              (solver.numericalConstraints ());
            // Check whether right hand side of each constraint has already
            // been instantiated. If not, a random configuration needs to be
            // sampled in this state.
            bool needToSample (false);
            for (std::size_t i = 0; i < intersectionConstraints.size ();
                 ++i) {
              const ImplicitPtr_t& constraint (intersectionConstraints [i]);
              const std::string& name (constraint->function ().name ());
              if (rightHandSides_ [name].empty ()) {
                needToSample = true;
              }
            } // for (std::size_t i = 0; i < intersectionConstraints.size ();

            // If at least one of the constraints has never been instantiated,
            // sample a valid configuration in state to initialize right hand
            // of constraints that have none.
            if (needToSample) {
              if (!sampleInState (interState, shooter, configValidations,
                                  maxNbTry, q)) {
                hppDout (info, "Failed to sample a configuration in state "
                         << interState->name ());
                continue;
              }
              assert (interState->contains (*q));
              // Instantiate right hand side of each constraint
              for (std::size_t i = 0; i < intersectionConstraints.size ();
                   ++i) {
                const ImplicitPtr_t& constraint (intersectionConstraints [i]);
                const std::string& name (constraint->function ().name ());
                if (rightHandSides_ [name].empty ()) {
                  hppDout (info, "constraint \""
                           << constraint->function ().name ()
                           << "\" has no right hand side yet.");
                  // Initialize right hand side with random configuration
#ifndef NDEBUG
                  bool success =
#endif
                    solver.rightHandSideFromConfig (constraint, *q);
                  assert (success);
                  vector_t rhs (constraint->function ().
                                outputSpace ()->nv ());
#ifndef NDEBUG
                  success =
#endif
                    solver.getRightHandSide (constraint, rhs);
                  assert (success);
                  rightHandSides_ [name].push_back (rhs);
                }
              } // for (std::size_t i = 0; i < intersectionConstraints.size ();
            }
            // At this point all constraints should have at least one
            // right hand side.
            for (std::size_t i = 0; i < intersectionConstraints.size (); ++i) {
              const ImplicitPtr_t& constraint (intersectionConstraints [i]);
              const std::string& name (constraint->function ().name ());
              // Sample right hand side among already instantiated
              std::size_t size (rightHandSides_ [name].size());
              std::size_t indice_rand = rand() % size;
              vector_t rhs (rightHandSides_ [name][indice_rand]);
              hppDout (info, "setting " << displayConfig (rhs)
                       << " for constraint \"" << name << "\"");
              hppDout (info, "indice_rand = " << indice_rand);
              hppDout (info, "size = " << size);
#ifndef NDEBUG
              bool success =
#endif
                solver.rightHandSide (constraint, rhs);
              assert (success);
            }
            // At this point, the right hand side of the solver is set.
            // We now sample a valid configuration with solver
            hppDout (info, solver);
            validConfig = sampleValidConfiguration
              (solver, shooter, configValidations, maxNbTry, q);
            if (validConfig) {
              vector_t error (solver.errorSize ());
              assert (solver.isSatisfied (*q, error));
              assert (interState->contains (*q));
              // register leaf to be visited in each state
              assert (leaves_.find (s1) != leaves_.end ());
              ContactStates_t::const_iterator it1
                (findLeafContainingConfig (*q, leaves_ [s1]));
              if (it1 == leaves_ [s1].end ()) {
                hppDout (info, "Adding a new leaf in state \""
                         << s1->name () << "\" with configuration "
                         << displayConfig (*q));
                // Add a leaf in state s1
                leaves_ [s1].push_back (ContactState (s1, *q, transition_ [s1]->
                                                      pathConstraint ()));
              } else {
                hppDout (info, "Adding configuration " << displayConfig (*q)
                         << " in state \"" << s1->name () << "\"");
                // Add configuration to roadmap of the existing leaf
                const ContactState& contactState (*it1);
                addConfigToLeafRoadmap (contactState, q);
              }
              assert (leaves_.find (s2) != leaves_.end ());
              ContactStates_t::const_iterator cs2
                (findLeafContainingConfig (*q, leaves_ [s2]));
              if (cs2 == leaves_ [s2].end ()) {
                hppDout (info, "Adding a new leaf in state \""
                         << s2->name () << "\" with configuration "
                         << displayConfig (*q));
                // Add a leaf in state s2
                leaves_ [s2].push_back (ContactState (s2, *q, transition_ [s2]->
                                                      pathConstraint ()));
              } else {
                hppDout (info, "Adding configuration " << displayConfig (*q)
                         << " in state \"" << s2->name () << "\"");
                // Add configuration to roadmap of the existing leaf
                const ContactState& contactState (*cs2);
                addConfigToLeafRoadmap (contactState, q);
              }
            } else {
              hppDout (info, "Failed to sample a configuration in state \""
                       << interState->name ());
            }
          }
        }
      }

      //////////////////////////////////////////////////////////////////////////

      void RMRStar::createLeafRoadmap (const ContactState& contactState,
                                       const ConfigurationPtr_t& q)
      {

        //copy the problem and pass the edge contraints
        core::Problem p (problem ());
        p.initConfig (q);
        graph::EdgePtr_t edge (transition_ [contactState.state ()]);
        p.steeringMethod (edge->steeringMethod ()->copy ());
        p.constraints
          (core::ConstraintSet::createCopy(contactState.constraints ()));
        p.pathValidation (edge->pathValidation());
        core::RoadmapPtr_t r = core::Roadmap::create(p.distance(),p.robot());
        r->clear();
        r->addNode (q);
        RMRStar::ProblemAndRoadmap_t pbRoadmap (p, r);

        //complete the map with the association ContactState/ProblemAndRoadmap
        leafRoadmaps_.insert
          (std::pair<ContactState,RMRStar::ProblemAndRoadmap_t>
           (contactState,pbRoadmap));
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
      //////////////////////////////////////////////////////////////////////////

      void RMRStar::registerRightHandSides
      (const NumericalConstraints_t& constraints, const Configuration_t& q)
      {
        for (NumericalConstraints_t::const_iterator it (constraints.begin ());
             it != constraints.end (); ++it) {
          (*it)->rightHandSideFromConfig (q);
          vector_t rhs ((*it)->rightHandSide ());
          const std::string& name ((*it)->function ().name ());
          if (std::find (rightHandSides_ [name].begin (),
                         rightHandSides_ [name].end (), rhs) ==
              rightHandSides_ [name].end ()) {
            hppDout (info, "adding rhs " << displayConfig (rhs));
            hppDout (info, " to constraint \"" << name << "\"");
            rightHandSides_ [name].push_back (rhs);
          }
        }
      }

      //////////////////////////////////////////////////////////////////////////

      void RMRStar::startSolve ()
      {
        PathPlanner::startSolve ();
        initialize ();
        setRhsFreq_=problem().getParameter("RMR*/SetRhsFreq").intValue();
        counter_=0;

        ConfigurationPtr_t q (roadmap()->initNode ()->configuration());
        //Set init ContactState
        graph::StatePtr_t stateInit=
          graph_->getState(*q);
        graph::EdgePtr_t loop_edge = transition_[stateInit];
        core::ConstraintSetPtr_t edgeConstraints =loop_edge->pathConstraint();
        registerRightHandSides (edgeConstraints->configProjector ()->
                                solver ().numericalConstraints (), *q);
        ContactState contactStateInit (stateInit, *q, edgeConstraints);

        for (core::NodeVector_t::const_iterator itn
               (roadmap_->goalNodes ().begin ());
             itn != roadmap_->goalNodes ().end (); ++itn) {

          //Set final ContactState
          q=(*itn)->configuration();

          graph::StatePtr_t stateGoal=graph_->getState(*q);
          loop_edge = transition_[stateGoal];
          edgeConstraints =loop_edge->pathConstraint();
          registerRightHandSides (edgeConstraints->configProjector ()->
                                  solver ().numericalConstraints (), *q);

          ContactState contactStateGoal
            (stateGoal,*q, edgeConstraints);
          addConfigToLeafRoadmap (contactStateGoal, q);
        }
        step_ = SAMPLE_INTERSECTION_STATES;
      }

      ////////////////////////////////////////////////////////////////////////////
      void RMRStar::oneStep ()
      {
        switch (step_) {
        case BUILD_ROADMAP:
        case SAMPLE_INTERSECTION_STATES:
          sampleIntersectionStates ();
          throw std::runtime_error ("Interruption");
          step_= BUILD_ROADMAP;
          break;
        }
      }
      ////////////////////////////////////////////////////////////////////////////

      RMRStar::RMRStar (const core::Problem& problem,
                        const core::RoadmapPtr_t& roadmap) :
        core::PathPlanner (problem, roadmap),
        manipulationProblem_
        (static_cast <const manipulation::Problem& > (problem)),
        roadmap_ (HPP_STATIC_PTR_CAST (manipulation::Roadmap, roadmap)),
        graph_ (manipulationProblem_.constraintGraph ()),
        transition_ ()

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
