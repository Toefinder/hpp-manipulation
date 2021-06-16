// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr),
//          Florent Lamiraux (florent.lamiraux@laas.fr)
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

#include <hpp/manipulation/path-planner/states-path-finder.hh>

#include <map>
#include <queue>
#include <vector>
#include <iomanip>

#include <hpp/util/exception-factory.hh>

#include <pinocchio/multibody/model.hpp>

#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/joint-collection.hh>

#include <hpp/constraints/affine-function.hh>
#include <hpp/constraints/locked-joint.hh>
#include <hpp/constraints/solver/by-substitution.hh>
#include <hpp/constraints/explicit.hh>

#include <hpp/core/path-vector.hh>
#include <hpp/core/configuration-shooter.hh>

#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph/state.hh>

namespace hpp {
  namespace manipulation {
    namespace pathPlanner {
      namespace statesPathFinder {

      std::size_t idxDeb;
      using Eigen::RowBlockIndices;
      using Eigen::ColBlockIndices;

      using graph::StatePtr_t;
      using graph::States_t;
      using graph::EdgePtr_t;
      using graph::Edges_t;
      using graph::Neighbors_t;
      using graph::NumericalConstraints_t;
      using graph::LockedJoints_t;
      using graph::segments_t;

      StatesPathFinderPtr_t StatesPathFinder::create (
          const ProblemConstPtr_t& problem)
      {
        StatesPathFinderPtr_t shPtr(new StatesPathFinder (problem));
        shPtr->init(shPtr);
        return shPtr;
      }

      StatesPathFinderPtr_t StatesPathFinder::copy () const
      {
        StatesPathFinder* ptr = new StatesPathFinder (*this);
        StatesPathFinderPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      struct StatesPathFinder::GraphSearchData
      {
        StatePtr_t s1, s2;

        // Datas for findNextTransitions
        struct state_with_depth {
          StatePtr_t s;
          EdgePtr_t e;
          std::size_t l; // depth to root
          std::size_t i; // index in parent state_with_depths_t
          inline state_with_depth () : s(), e(), l(0), i (0) {}
          inline state_with_depth (EdgePtr_t _e, std::size_t _l, std::size_t _i)
            : s(_e->stateFrom()), e(_e), l(_l), i (_i) {}
        };
        typedef std::vector<state_with_depth> state_with_depths_t;
        typedef std::map<StatePtr_t,state_with_depths_t> StateMap_t;
        /// std::size_t is the index in state_with_depths_t at StateMap_t::iterator
        struct state_with_depth_ptr_t {
          StateMap_t::iterator state;
          std::size_t parentIdx;
          state_with_depth_ptr_t (const StateMap_t::iterator& it, std::size_t idx) : state (it), parentIdx (idx) {}
        };
        typedef std::queue<state_with_depth_ptr_t> Queue_t;
        std::size_t maxDepth;
        StateMap_t parent1; // TODO, parent2;
        Queue_t queue1;

        const state_with_depth& getParent(const state_with_depth_ptr_t& _p) const
        {
          const state_with_depths_t& parents = _p.state->second;
          return parents[_p.parentIdx];
        }

        state_with_depth_ptr_t addInitState()
        {
          StateMap_t::iterator next =
            parent1.insert(StateMap_t::value_type(s1, state_with_depths_t(1))).first;
          return state_with_depth_ptr_t (next, 0);
        }

        state_with_depth_ptr_t addParent(
            const state_with_depth_ptr_t& _p,
            const EdgePtr_t& transition)
        {
          const state_with_depths_t& parents = _p.state->second;
          const state_with_depth& from = parents[_p.parentIdx];

          // Insert state to if necessary
          StateMap_t::iterator next = parent1.insert (
              StateMap_t::value_type(
                transition->stateTo(), state_with_depths_t ()
                )).first;

          next->second.push_back (
              state_with_depth(transition, from.l + 1, _p.parentIdx));

          return state_with_depth_ptr_t (next, next->second.size()-1);
        }
      };

      static bool containsLevelSet(const graph::EdgePtr_t& e) {
        graph::WaypointEdgePtr_t we =
          HPP_DYNAMIC_PTR_CAST(graph::WaypointEdge, e);
        if (!we)
          return false;
        for (std::size_t i = 0; i <= we->nbWaypoints(); i++) {
          graph::LevelSetEdgePtr_t lse =
            HPP_DYNAMIC_PTR_CAST(graph::LevelSetEdge, we->waypoint(i));
          if (lse)
            return true;
        }
        return false;
      }

      void StatesPathFinder::gatherGraphConstraints ()
      {
        typedef graph::Edge Edge;
        typedef graph::EdgePtr_t EdgePtr_t;
        typedef graph::GraphPtr_t GraphPtr_t;
        typedef constraints::solver::BySubstitution Solver_t;

        GraphPtr_t cg (problem_->constraintGraph ());
        const ConstraintsAndComplements_t& cac
          (cg->constraintsAndComplements ());
        for (std::size_t i = 0; i < cg->nbComponents (); ++i) {
          EdgePtr_t edge (HPP_DYNAMIC_PTR_CAST (Edge, cg->get (i).lock ()));
          if (edge) {
            // Don't even consider level set edges
            if (containsLevelSet(edge)) continue;
            const Solver_t& solver (edge->pathConstraint ()->
                                    configProjector ()->solver ());
            const NumericalConstraints_t& constraints
              (solver.numericalConstraints ());
            for (NumericalConstraints_t::const_iterator it
                   (constraints.begin ()); it != constraints.end (); ++it) {
              if ((*it)->parameterSize () > 0) {
                const std::string& name ((*it)->function ().name  ());
                if (index_.find (name) == index_.end ()) {
                  // constraint is not in map, add it
                  index_ [name] = constraints_.size ();
                  // Check whether constraint is equivalent to a previous one
                  for (NumericalConstraints_t::const_iterator it1
                         (constraints_.begin ()); it1 != constraints_.end ();
                       ++it1) {
                    for (ConstraintsAndComplements_t::const_iterator it2
                           (cac.begin ()); it2 != cac.end (); ++it2) {
                      if (((**it1 == *(it2->complement)) &&
                           (**it == *(it2->both))) ||
                          ((**it1 == *(it2->both)) &&
                           (**it == *(it2->complement)))) {
                        assert (sameRightHandSide_.count (*it1) == 0);
                        assert (sameRightHandSide_.count (*it) == 0);
                        sameRightHandSide_ [*it1] = *it;
                        sameRightHandSide_ [*it] = *it1;
                      }
                    }
                  }
                  constraints_.push_back (*it);
                  hppDout (info, "Adding constraint \"" << name << "\"");
                  hppDout (info, "Edge \"" << edge->name () << "\"");
                  hppDout (info, "parameter size: " << (*it)->parameterSize ());

                }
              }
            }
          }
        }
      }

      bool StatesPathFinder::findTransitions (GraphSearchData& d) const
      {
        while (! d.queue1.empty())
        {
          GraphSearchData::state_with_depth_ptr_t _state = d.queue1.front();

          const GraphSearchData::state_with_depth& parent = d.getParent(_state);
          if (parent.l >= d.maxDepth) return true;
          d.queue1.pop();

          bool done = false;

          const Neighbors_t& neighbors = _state.state->first->neighbors();
          for (Neighbors_t::const_iterator _n = neighbors.begin();
              _n != neighbors.end(); ++_n) {
            EdgePtr_t transition = _n->second;

            // Don't even consider level set edges
            if (containsLevelSet(transition)) continue;

            // Avoid identical consecutive transition
            if (transition == parent.e) continue;

            // TODO
            // If (transition->to() == d.s2) check if this list is feasible.
            // - If a constraint with non-constant right hand side is present
            //   in all transitions, then the rhs from d.q1 and d.q2 should be
            //   equal

            // Insert parent
            d.queue1.push (
              d.addParent (_state, transition)
            );

            done = done || (transition->stateTo() == d.s2);
          }
          if (done) break;
        }
        return false;
      }

      Edges_t StatesPathFinder::getTransitionList (
          const GraphSearchData& d, const std::size_t& i) const
      {
        assert (d.parent1.find (d.s2) != d.parent1.end());
        const GraphSearchData::state_with_depths_t& roots = d.parent1.at(d.s2);
        Edges_t transitions;
        if (i >= roots.size()) return transitions;

        const GraphSearchData::state_with_depth* current = &roots[i];
        transitions.reserve (current->l);
        graph::WaypointEdgePtr_t we;
        while (current->e) {
          assert (current->l > 0);
          we = HPP_DYNAMIC_PTR_CAST(graph::WaypointEdge, current->e);
          if (we) {
            for (int i = (int)we->nbWaypoints(); i >= 0; --i)
              transitions.push_back(we->waypoint(i));
          } else {
            transitions.push_back(current->e);
          }
          current = &d.parent1.at(current->s)[current->i];
        }
        std::reverse (transitions.begin(), transitions.end());
        return transitions;
      }

      struct StatesPathFinder::OptimizationData
      {
        typedef constraints::solver::HierarchicalIterative::Saturation_t
        Saturation_t;
        enum RightHandSideStatus_t {
          // Constraint is not in solver for this waypoint
          ABSENT,
          // right hand side of constraint for this waypoint is equal to
          // right hand side for previous waypoint
          EQUAL_TO_PREVIOUS,
          // right hand side of constraint for this waypoint is equal to
          // right hand side for initial configuration
          EQUAL_TO_INIT,
          // right hand side of constraint for this waypoint is equal to
          // right hand side for goal configuration
          EQUAL_TO_GOAL
        }; // enum RightHandSideStatus_t
        const std::size_t N, nq, nv;
        std::vector <Solver_t> solvers;
        std::vector <bool> isTargetWaypoint;
        std::vector <bool> isEdgeShort;
        // Waypoints lying in each intermediate state
        matrix_t waypoint;
        Configuration_t q1, q2;
        core::DevicePtr_t robot;
        // Matrix specifying for each constraint and each waypoint how
        // the right hand side is initialized in the solver.
        Eigen::Matrix < LiegroupElement, Eigen::Dynamic, Eigen::Dynamic > M_rhs;
        Eigen::Matrix < RightHandSideStatus_t, Eigen::Dynamic, Eigen::Dynamic >
        M_status;
        // Number of trials to generate each waypoint configuration
        OptimizationData (const core::ProblemConstPtr_t _problem,
                          const Configuration_t& _q1,
                          const Configuration_t& _q2,
                          const Edges_t& transitions
                          ) :
          N (transitions.size () - 1), nq (_problem->robot()->configSize()),
          nv (_problem->robot()->numberDof()),
          solvers (N, _problem->robot()->configSpace ()),
          waypoint (nq, N), q1 (_q1), q2 (_q2),
          robot (_problem->robot()),
          M_rhs (), M_status ()
        {
          waypoint.setZero();
          for (auto solver: solvers){
            // Set maximal number of iterations for each solver
            solver.maxIterations(_problem->getParameter
                            ("StatesPathFinder/maxIteration").intValue());
            // Set error threshold for each solver
            solver.errorThreshold(_problem->getParameter
                        ("StatesPathFinder/errorThreshold").floatValue());
          }
          assert (transitions.size () > 0);
          isTargetWaypoint.assign(N+1, false);
          isEdgeShort.assign(N+1, true);
          for (std::size_t i = 0; i < transitions.size(); i++) {
            isTargetWaypoint[i] = transitions[i]->stateTo()->isWaypoint();
            std::string name = transitions[i]->name();
            if (name.find("_34") != std::string::npos)
              isEdgeShort[i] = false;
            isEdgeShort[i] = transitions[i]->isShort();
          } 
        }
      };

      bool StatesPathFinder::checkConstantRightHandSide (size_type index)
      {
        OptimizationData& d = *optData_;
        const ImplicitPtr_t c (constraints_ [index]);
        LiegroupElement rhsInit(c->function().outputSpace());
        c->rightHandSideFromConfig (d.q1, rhsInit);
        LiegroupElement rhsGoal(c->function().outputSpace());
        c->rightHandSideFromConfig (d.q2, rhsGoal);
        // Check that right hand sides are close to each other
        value_type eps (problem_->constraintGraph ()->errorThreshold ());
        value_type eps2 (eps * eps);
        if ((rhsGoal - rhsInit).squaredNorm () > eps2) {
          return false;
        }
        // Matrix of solver right hand sides
        for (size_type j=0; j<d.M_rhs.cols (); ++j) {
          d.M_rhs (index, j) = rhsInit;
        }
        return true;
      }

      bool StatesPathFinder::checkWaypointRightHandSide
      (std::size_t ictr, std::size_t jslv) const
      {
        const OptimizationData& d = *optData_;
        ImplicitPtr_t c = constraints_ [ictr]->copy();
        LiegroupElement rhsNow(c->function().outputSpace());
        assert(rhsNow.size() == c->rightHandSideSize());
        c->rightHandSideFromConfig (d.waypoint.col (jslv), rhsNow);
        c = constraints_ [ictr]->copy();
        LiegroupElement rhsOther(c->function().outputSpace());
        switch (d.M_status(ictr, jslv)) {
        case OptimizationData::EQUAL_TO_INIT:
          c->rightHandSideFromConfig (d.q1, rhsOther);
          break;
        case OptimizationData::EQUAL_TO_GOAL:
          c->rightHandSideFromConfig (d.q2, rhsOther);
          break;
        case OptimizationData::EQUAL_TO_PREVIOUS:
          c->rightHandSideFromConfig (d.waypoint.col (jslv-1), rhsOther);
          break;
        case OptimizationData::ABSENT:
        default:
          return true;
        }
        hpp::pinocchio::vector_t diff = rhsOther - rhsNow;
        hpp::pinocchio::vector_t diffmask(diff.size());
        for (auto k: c->activeRows()) // filter with constraint mask
          for (size_type kk = k.first; kk < k.first + k.second; kk++)
            diffmask[kk] = diff[kk];
        value_type eps (problem_->constraintGraph ()->errorThreshold ());
        value_type eps2 (eps * eps);
        return diffmask.squaredNorm () < eps2;
      }

      bool StatesPathFinder::checkWaypointRightHandSide
      (std::size_t jslv) const
      {
        for (std::size_t ictr = 0; ictr < constraints_.size(); ictr++)
          if (!checkWaypointRightHandSide(ictr, jslv))
            return false;
        return true;
      }

      void StatesPathFinder::displayRhsMatrix ()
      {
        OptimizationData& d = *optData_;
        Eigen::Matrix < LiegroupElement, Eigen::Dynamic, Eigen::Dynamic >& m
          = d.M_rhs;

        for (std::size_t i = 0; i < constraints_.size(); i++) {
          const ImplicitPtr_t& constraint = constraints_[i];
          for (std::size_t j = 0; j < d.solvers.size(); j++) {
            const vectorIn_t& config = d.waypoint.col(j);
            LiegroupElement le(constraint->function().outputSpace());
            constraint->rightHandSideFromConfig(d.waypoint.col(j), le);
            m(i,j) = le;
          }
        }

        std::ostringstream oss; oss.precision (2);
        oss << "\\documentclass[12pt,landscape]{article}" << std::endl;
        oss << "\\usepackage[a3paper]{geometry}" << std::endl;
        oss << "\\begin {document}" << std::endl;

        for (size_type ii = 0; ii < (m.cols()+7)/8; ii++) {
          size_type j0 = ii*8;
          size_type j1 = std::min(ii*8+8, m.cols());
          size_type dj = j1-j0;
          oss << "\\begin {tabular}{";
          for (size_type j=0; j<dj + 2; ++j)
            oss << "c";
          oss << "}" << std::endl;
          oss << "Constraint & mask";
          for (size_type j=j0; j<j1; ++j)
            oss << " & WP" << j;
          oss << "\\\\" << std::endl;
          for (size_type i=0; i<m.rows (); ++i) {
            std::vector<int> mask(constraints_[i]->parameterSize());
            for (auto k: constraints_[i]->activeRows())
              for (size_type kk = k.first; kk < k.first + k.second; kk++)
                mask[kk] = 1;
            std::ostringstream oss1; oss1.precision (2);
            std::ostringstream oss2; oss2.precision (2);
            oss1 << constraints_ [i]->function ().name () << " & ";
            
            oss1 << "$\\left(\\begin{array}{c} ";
            for (std::size_t k=0; k<mask.size (); ++k) {
              oss1 << mask[k] << "\\\\ ";
            }
            oss1 << "\\end{array}\\right)$" << std::endl;
            oss1 << " & " << std::endl;

            for (size_type j=j0; j<j1; ++j) {
              if (d.M_status(i,j) != OptimizationData::ABSENT || (j < m.cols () - 1 &&
                  d.M_status(i,j+1) ==  OptimizationData::EQUAL_TO_PREVIOUS)) {
                oss2 << "$\\left(\\begin{array}{c} ";
                for (size_type k=0; k<m (i,j).size (); ++k) {
                  oss2 << ((abs(m (i,j).vector()[k]) < 1e-6) ? 0 : m (i,j).vector()[k]) << "\\\\ ";
                }
                oss2 << "\\end{array}\\right)$" << std::endl;
              }
              if (j < j1 - 1) {
                oss2 << " & " << std::endl;
              }
            }
            std::string str2 = oss2.str();
            if (str2.size() > 50) { // don't display constraints used nowhere
              oss << oss1.str() << str2 << "\\\\" << std::endl;
            }
          }
          oss << "\\end{tabular}" << std::endl << std::endl;
        }
        oss << "\\end{document}" << std::endl;

        std::string s = oss.str ();
        std::string s2 = "";
        for (std::size_t i=0; i < s.size(); i++) {
          if (s[i] == '_') s2 += "\\_";
          else s2.push_back(s[i]);
        }
        hppDout (info, s2);
      }

      void StatesPathFinder::displayStatusMatrix (const graph::Edges_t& transitions)
      {
        const Eigen::Matrix < OptimizationData::RightHandSideStatus_t, Eigen::Dynamic, Eigen::Dynamic >&
          m = optData_->M_status;
        std::ostringstream oss; oss.precision (5);
        oss << "\\documentclass[12pt,landscape]{article}" << std::endl;
        oss << "\\usepackage[a3paper]{geometry}" << std::endl;
        oss << "\\begin {document}" << std::endl;
        oss << "\\paragraph{Edges}" << std::endl;
        oss << "\\begin{enumerate}" << std::endl;
        for (auto edge : transitions) {
          oss << "\\item \\texttt{" << edge->name() << "}" << std::endl;
        }
        oss << "\\end{enumerate}" << std::endl;
        oss << "\\begin {tabular}{l|";
        for (size_type j=0; j<m.cols (); ++j)
          if (transitions[j]->stateTo()->isWaypoint()) oss << "c";
          else oss << "|c|";
        oss << "|}" << std::endl;
        oss << "Constraint";
        for (size_type j=0; j<m.cols (); ++j)
          oss << " & " << j+1;
        oss << "\\\\" << std::endl;
        for (size_type i=0; i<m.rows (); ++i) {
          oss << "\\texttt{" << constraints_ [i]->function ().name () << "} & " << std::endl;
          for (size_type j=0; j<m.cols (); ++j) {
            oss << m (i,j);
            if (j < m.cols () - 1)
              oss << " & ";
          }
          oss << "\\\\" << std::endl;
        }
        oss << "\\end{tabular}" << std::endl;
        oss << "\\end{document}" << std::endl;

        std::string s = oss.str ();
        std::string s2 = "";
        for (std::size_t i=0; i < s.size(); i++) {
          if (s[i] == '_') s2 += "\\_";
          else s2.push_back(s[i]);
        }
        hppDout (info, s2);
      }

      bool StatesPathFinder::contains
      (const Solver_t& solver, const ImplicitPtr_t& c) const
      {
        if (solver.contains (c)) return true;
        /* // TODO remove this when bug fixed
        std::map <ImplicitPtr_t, ImplicitPtr_t>::const_iterator it
          (sameRightHandSide_.find (c));
        if ((it != sameRightHandSide_.end () &&
             solver.contains (it->second))) {
          return true;
        }//*/
        return false;
      }

      bool StatesPathFinder::buildOptimizationProblem
        (const graph::Edges_t& transitions)
      {
        OptimizationData& d = *optData_;
        if (d.N == 0) return true;
        d.M_status.resize (constraints_.size (), d.N);
        d.M_status.fill (OptimizationData::ABSENT);
        d.M_rhs.resize (constraints_.size (), d.N);
        d.M_rhs.fill (LiegroupElement ());
        size_type index = 0;
        // Loop over constraints
        for (NumericalConstraints_t::const_iterator it (constraints_.begin ());
             it != constraints_.end (); ++it) {
          const ImplicitPtr_t& c (*it);
          // Loop forward over waypoints to determine right hand sides equal
          // to initial configuration
          for (std::size_t j = 0; j < d.N; ++j) {
            // Get transition solver
            const Solver_t& trSolver
              (transitions [j]->pathConstraint ()->configProjector ()->
               solver ());
            if (contains (trSolver, c)) {
              if ((j==0) || d.M_status (index, j-1) ==
                  OptimizationData::EQUAL_TO_INIT) {
                d.M_status (index, j) = OptimizationData::EQUAL_TO_INIT;
              } else {
                d.M_status (index, j) = OptimizationData::EQUAL_TO_PREVIOUS;
              }
            }
          }
          // Loop backward over waypoints to determine right hand sides equal
          // to final configuration
          for (size_type j = d.N-1; j > 0; --j) {
            // Get transition solver
            const Solver_t& trSolver
              (transitions [(std::size_t)j+1]->pathConstraint ()->
               configProjector ()->solver ());
            if (contains (trSolver, c)) {
              if ((j==(size_type) d.N-1) || d.M_status (index, j+1) ==
                  OptimizationData::EQUAL_TO_GOAL) {
                // If constraint right hand side is already equal to
                // initial config, check that right hand side is equal
                // for init and goal configs.
                if (d.M_status (index, j) ==
                    OptimizationData::EQUAL_TO_INIT) {
                  if (checkConstantRightHandSide (index)) {
                    // stop for this constraint
                    break;
                  } else {
                    // Right hand side of constraint should be equal along the
                    // whole path but is different at init and at goal configs.
                    return false;
                  }
                } else {
                  d.M_status (index, j) = OptimizationData::EQUAL_TO_GOAL;
                }
              }
            } else {
              break;
            }
          }
          ++index;
        } // for (NumericalConstraints_t::const_iterator it
        displayStatusMatrix (transitions);
        // Fill solvers with target constraints of transition
        for (std::size_t j = 0; j < d.N; ++j) {
          d.solvers [j] = transitions [j]->
            targetConstraint ()->configProjector ()->solver ();
          for (std::size_t i = 0; i < constraints_.size (); i++) {
            if (j > 0 &&
                d.M_status(i, j-1) == OptimizationData::ABSENT && 
                d.M_status(i, j) == OptimizationData::EQUAL_TO_GOAL &&
                !contains(d.solvers[j], constraints_[i])) {
              d.solvers[j].add(constraints_[i]);
              hppDout(info, "Adding missing constraint " << constraints_[i]->function().name()
                                << " to solver for waypoint" << j+1);
            }
          }
        }

        return true;
      }

      bool StatesPathFinder::checkSolverRightHandSide
      (std::size_t ictr, std::size_t jslv) const
      {
        const OptimizationData& d = *optData_;
        ImplicitPtr_t c = constraints_ [ictr]->copy();
        const Solver_t& solver = d.solvers[jslv];
        vector_t rhs(c->rightHandSideSize());
        solver.getRightHandSide(c, rhs);
        LiegroupElement rhsNow(c->function().outputSpace());
        assert(rhsNow.size() == rhs.size());
        rhsNow.vector() = rhs;
        LiegroupElement rhsOther(c->function().outputSpace());
        switch (d.M_status(ictr, jslv)) {
        case OptimizationData::EQUAL_TO_INIT:
          c->rightHandSideFromConfig (d.q1, rhsOther);
          break;
        case OptimizationData::EQUAL_TO_GOAL:
          c->rightHandSideFromConfig (d.q2, rhsOther);
          break;
        case OptimizationData::EQUAL_TO_PREVIOUS:
          c->rightHandSideFromConfig (d.waypoint.col (jslv-1), rhsOther);
          break;
        case OptimizationData::ABSENT:
        default:
          return true;
        }
        hpp::pinocchio::vector_t diff = rhsOther - rhsNow;
        hpp::pinocchio::vector_t diffmask(diff.size());
        for (auto k: c->activeRows()) // filter with constraint mask
          for (size_type kk = k.first; kk < k.first + k.second; kk++)
            diffmask[kk] = diff[kk];
        value_type eps (problem_->constraintGraph ()->errorThreshold ());
        value_type eps2 (eps * eps);
        if (diffmask.squaredNorm () > eps2) hppDout(warning, diffmask.squaredNorm () << " vs " << eps2);
        return diffmask.squaredNorm () < eps2;
      }

      bool StatesPathFinder::checkSolverRightHandSide
      (std::size_t jslv) const
      {
        for (std::size_t ictr = 0; ictr < constraints_.size(); ictr++)
          if (!checkSolverRightHandSide(ictr, jslv))
            return false;
        return true;
      }

      bool StatesPathFinder::buildOptimizationProblemFromNames(std::vector<std::string> names)
      {
        graph::Edges_t transitions;
        graph::GraphPtr_t cg (problem_->constraintGraph ());
        for (const std::string& name: names) {
          for (std::size_t i = 0; i < cg->nbComponents (); ++i) {
            graph::EdgePtr_t edge (HPP_DYNAMIC_PTR_CAST (graph::Edge, cg->get (i).lock ()));
            if (edge && edge->name() == name)
              transitions.push_back(edge);
          }
        }
        return buildOptimizationProblem(transitions);
      }

      void StatesPathFinder::initializeRHS(std::size_t j) {
        OptimizationData& d = *optData_;
        Solver_t& solver (d.solvers [j]);
        for (std::size_t i=0; i<constraints_.size (); ++i) {
          const ImplicitPtr_t& c (constraints_ [i]);
          bool ok = true;
          switch (d.M_status ((size_type)i, (size_type)j)) {
          case OptimizationData::EQUAL_TO_PREVIOUS:
            assert (j != 0);
            ok = solver.rightHandSideFromConfig (c, d.waypoint.col (j-1));
            break;
          case OptimizationData::EQUAL_TO_INIT:
            ok = solver.rightHandSideFromConfig (c, d.q1);
            break;
          case OptimizationData::EQUAL_TO_GOAL:
            ok = solver.rightHandSideFromConfig (c, d.q2);
            break;
          case OptimizationData::ABSENT:
          default:
            ;
          }
          continue;
          if(!ok) {
            std::ostringstream err_msg;
            err_msg << "\nConstraint " << i << " missing in solver " << j+1
                    << " (" << c->function().name() << ")\n";
            hppDout(warning, err_msg.str());
            throw(std::logic_error(err_msg.str()));
          }
          assert (checkSolverRightHandSide(i, j));
        }
      }

      void StatesPathFinder::initWPRandom(std::size_t wp) {
        assert(wp >=1 && wp <= (std::size_t) optData_->waypoint.cols());
        initializeRHS(wp-1);
        optData_->waypoint.col (wp-1) = *(problem()->configurationShooter()->shoot());
      }
      void StatesPathFinder::initWPNear(std::size_t wp) {
        assert(wp >=1 && wp <= (std::size_t) optData_->waypoint.cols());
        initializeRHS(wp-1);
        if (wp == 1)
          optData_->waypoint.col (wp-1) = optData_->q1;
        else
          optData_->waypoint.col (wp-1) = optData_->waypoint.col (wp-2);
      }
      void StatesPathFinder::initWP(std::size_t wp, ConfigurationIn_t q) {
        assert(wp >=1 && wp <= (std::size_t) optData_->waypoint.cols());
        initializeRHS(wp-1);
        optData_->waypoint.col (wp-1) = q;
      }
      int StatesPathFinder::solveStep(std::size_t wp) {
        assert(wp >=1 && wp <= (std::size_t) optData_->waypoint.cols());
        std::size_t j = wp-1;
        Solver_t& solver (optData_->solvers [j]);
        Solver_t::Status status = solver.solve (optData_->waypoint.col (j),
              constraints::solver::lineSearch::Backtracking ());
        if (status == Solver_t::SUCCESS) {
          assert (checkWaypointRightHandSide(j));
          core::ConfigValidationsPtr_t configValidations = problem_->configValidations();
          core::ValidationReportPtr_t validationReport;
          if (!configValidations->validate
                (optData_->waypoint.col (j), validationReport)) {
            return 2;
          }
          return 0;
        } 
        return 1;
      }

      std::string StatesPathFinder::displayConfigsSolved() const {
        const OptimizationData& d = *optData_;
        std::ostringstream oss;
        oss << "configs = [" << std::endl; 
        oss << "  " << pinocchio::displayConfig(d.q1) << ",  # 0" << std::endl;
        for (size_type j = 0; j < d.waypoint.cols(); j++)
          oss << "  " << pinocchio::displayConfig(d.waypoint.col(j)) << ",  # " << j+1 << std::endl;
        oss << "  " << pinocchio::displayConfig(d.q2)
            << "  # " << d.waypoint.cols()+1 << std::endl << "]" << std::endl;
        std::string ans = oss.str();
        hppDout(info, ans);
        return ans;
      }

      Configuration_t StatesPathFinder::configSolved (std::size_t wp) const {
        const OptimizationData& d = *optData_;
        std::size_t nbs = optData_->solvers.size();
        if (wp == 0)
          return d.q1;
        if (wp >= nbs+1)
          return d.q2;
        return d.waypoint.col(wp-1);
      }

      bool StatesPathFinder::solveOptimizationProblem ()
      {
        OptimizationData& d = *optData_;
        // Try to solve with sets of random configs for each waypoint
        std::size_t nTriesMax = problem_->getParameter
	        ("StatesPathFinder/nTriesUntilBacktrack").intValue();
        std::size_t nTriesMax1 = nTriesMax*10; // more tries for the first waypoint
        std::size_t nFailsMax = nTriesMax*20; // fails before reseting the whole solution
        std::vector<std::size_t> nTriesDone(d.solvers.size()+1, 0);
        std::size_t nFails = 0;
        std::size_t wp = 1; // waypoint index starting at 1 (wp 0 = q1)

        while (wp <= d.solvers.size()) {
          // enough try for a waypoint: backtrack or stop
          while (nTriesDone[wp] >= nTriesMax) {
            if (wp == 1) {
              if (nTriesDone[wp] < nTriesMax1)
                break;
              displayConfigsSolved();
              return false; // too many tries that need to reset the entire solution
            }
            do {
              nTriesDone[wp] = 0;
              wp--; // backtrack: try a new solution for the latest waypoint
            } while (wp>1 && d.isTargetWaypoint[wp-1]);
          }
          
          // Completely reset a solution when too many tries have failed
          if (wp > 1 && nFails >= nFailsMax) {
            for (std::size_t k = 2; k <= d.solvers.size(); k++)
              nTriesDone[k] = 0;
            wp = 1;
            if (nTriesDone[1] >= nTriesMax1) {
              displayConfigsSolved();
              return false;
            }
          }
          // Reset the fail counter while the solution is empty
          if (wp == 1)
            nFails = 0;

          // Initialize right hand sides, and
          // Choose a starting configuration for the solver.solve method:
          // - from previous waypoint if it's the first time we see this solver
          //   given current solvers 0 to j-1 and the previous configuration is
          //   relevant (often)
          // - with a random configuration if the other initialization has been
          //   tried and failed, or when the previous configuration is not a relevant
          //   dtart (), now try random configs
          if (nTriesDone[wp] == 0)//d.isEdgeShort[wp])
            initWPNear(wp);
          else
            initWPRandom(wp);

          nTriesDone[wp]++;
          //if (wp > 1) nTriesDone[wp]++;
          // Backtrack to last state when nTriesDone[j] gets too big

          int out = solveStep(wp);
          hppDout(info, "solveStep exit code at WP" << wp << ": " << out);
          switch (out) {
          case 0: // Valid solution, go to next waypoint
            wp++; break;
          case 1: // Collision. If that happens too much, go back to first waypoint
            nFails++; break;
          case 2: // Bad solve status, considered usual so nothing more
            break;
          default:
            throw(std::logic_error("Unintended exit code for solveStep"));
          }
        }

        displayConfigsSolved();
        displayRhsMatrix ();

        return true;
      }

      core::Configurations_t StatesPathFinder::buildPath () const
      {
        OptimizationData& d = *optData_;
        core::Configurations_t pv;
        ConfigurationPtr_t q1 (new Configuration_t (d.q1));
        pv.push_back(q1);
        for (std::size_t i = 0; i < d.N; ++i) {
          ConfigurationPtr_t q (new Configuration_t (d.waypoint.col (i)));
          pv.push_back(q);
        }
        ConfigurationPtr_t q2 (new Configuration_t (d.q2));
        pv.push_back(q2);
        return pv;
      }

      core::Configurations_t StatesPathFinder::compute (
          ConfigurationIn_t q1, ConfigurationIn_t q2)
      {
        const graph::GraphPtr_t& graph(problem_->constraintGraph ());
        GraphSearchData d;
        d.s1 = graph->getState (q1);
        d.s2 = graph->getState (q2);
        // d.maxDepth = 2;
        d.maxDepth = problem_->getParameter
	  ("StatesPathFinder/maxDepth").intValue();

        // Find
        d.queue1.push (d.addInitState());
        std::size_t idxSol = (d.s1 == d.s2 ? 1 : 0);

        bool maxDepthReached;
        while (!(maxDepthReached = findTransitions (d))) { // mut
          Edges_t transitions = getTransitionList (d, idxSol); // const, const
          while (! transitions.empty()) {
#ifdef HPP_DEBUG
            std::ostringstream ss;
            ss << " Trying solution " << idxSol << ": \n\t";
            for (std::size_t j = 0; j < transitions.size(); ++j)
              ss << transitions[j]->name() << ", \n\t";
            hppDout (info, ss.str());
#endif // HPP_DEBUG
            if (optData_) delete optData_;
            optData_ = new OptimizationData (problem(), q1, q2, transitions);

            //if (idxDeb == 72)// || idxDeb == 86)
            if (buildOptimizationProblem (transitions)) { // mut, const
              lastBuiltTransitions_ = transitions;
              if (solveOptimizationProblem ()) { // mut, const
                core::Configurations_t path = buildPath (); // const 
                if (path.size() > 2) {
                  hppDout (warning, " Success for solution " << idxSol << ", return path");
                  return path;
                } else {
                  hppDout (warning, " Failed solution " << idxSol << " at step 5 (build path)");
                }
              } else {
                  hppDout (warning, " Failed solution " << idxSol << " at step 4 (solve opt pb)");
              }
            } else {
              hppDout (warning, " Failed solution " << idxSol << " at step 3 (build opt pb)");
            }
            ++idxSol;
            idxDeb = idxSol;
            transitions = getTransitionList(d, idxSol);
          }
        }
        hppDout (warning, " Max depth reached");

        core::Configurations_t empty_path;
        ConfigurationPtr_t q (new Configuration_t (q1));
        empty_path.push_back(q);
        return empty_path;
      }

      // access functions for Python interface
      std::vector<std::string> StatesPathFinder::constraintNamesFromSolverAtWaypoint
        (std::size_t wp)
      {
        std::size_t nbs = optData_->solvers.size();
        assert (wp > 0 && wp <= nbs);
        constraints::solver::BySubstitution& solver (optData_->solvers [wp-1]);
        std::vector<std::string> ans;
        for (std::size_t i=0; i < solver.constraints().size(); i++)
          ans.push_back(solver.constraints()[i]->function().name());
        return ans;
      }

      std::vector<std::string> StatesPathFinder::lastBuiltTransitions() const
      {
        std::vector<std::string> ans;
        for (const EdgePtr_t& edge: lastBuiltTransitions_)
          ans.push_back(edge->name());
        return ans;
      }

      using core::Parameter;
      using core::ParameterDescription;

      HPP_START_PARAMETER_DECLARATION(StatesPathFinder)
      core::Problem::declareParameter(ParameterDescription(Parameter::INT,
            "StatesPathFinder/maxDepth",
            "Maximum number of transitions to look for.",
            Parameter((size_type)6)));
      core::Problem::declareParameter(ParameterDescription(Parameter::INT,
            "StatesPathFinder/maxIteration",
            "Maximum number of iterations of the Newton Raphson algorithm.",
            Parameter((size_type)60)));
      core::Problem::declareParameter(ParameterDescription(Parameter::FLOAT,
            "StatesPathFinder/errorThreshold",
            "Error threshold of the Newton Raphson algorithm.",
            Parameter(1e-4)));
      core::Problem::declareParameter(ParameterDescription(Parameter::INT,
            "StatesPathFinder/nTriesUntilBacktrack",
            "Number of tries when sampling configurations before backtracking.",
            Parameter((size_type)10)));
      HPP_END_PARAMETER_DECLARATION(StatesPathFinder)
      } // namespace statesPathFinder
    } // namespace pathPlanner
  } // namespace manipulation
} // namespace hpp
