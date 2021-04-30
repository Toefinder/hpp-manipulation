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

#ifndef HPP_MANIPULATION_PATH_PLANNER_MULTIPLE_STATE_OPTIMIZATION_HH
# define HPP_MANIPULATION_PATH_PLANNER_MULTIPLE_STATE_OPTIMIZATION_HH

# include <hpp/core/fwd.hh>
# include <hpp/core/path.hh>
# include <hpp/core/projection-error.hh>

# include <hpp/core/config-projector.hh>

# include <hpp/manipulation/config.hh>
# include <hpp/manipulation/fwd.hh>
# include <hpp/manipulation/problem.hh>

namespace hpp {
  namespace manipulation {
    namespace pathPlanner {

      /// \addtogroup path_planner
      /// \{

      /// Optimization-based steering method.
      ///
      /// #### Sketch of the method
      ///
      /// Given two configuration \f$ (q_1,q_2) \f$, this class formulates and
      /// solves the problem as follows.
      /// - Compute the corresponding states \f$ (s_1, s_2) \f$.
      /// - For a each path \f$ (e_0, ... e_n) \f$ of length not more than
      ///   parameter "MultipleStateOptimization/maxDepth" between
      ///   \f$ (s_1, s_2)\f$ in the constraint graph, do:
      ///   - define \f$ n-1 \f$ intermediate configuration \f$ p_i \f$,
      ///   - initialize the optimization problem, as explained below,
      ///   - solve the optimization problem, which gives \f$ p^*_i \f$,
      ///   - in case of failure, continue the loop.
      ///   - call the Edge::build of each \f$ e_j \f$ for each consecutive
      ///     \f$ (p^*_i, p^*_{i+1}) \f$.
      ///
      /// #### Problem formulation
      /// Find \f$ (p_i) \f$ such that:
      /// - \f$ p_0 = q_1 \f$,
      /// - \f$ p_{n+1} = q_2 \f$,
      /// - \f$ p_i \f$ is in state between \f$ (e_{i-1}, e_i) \f$, (\ref StateFunction)
      /// - \f$ (p_i, p_{i+1}) \f$ are reachable with transition \f$ e_i \f$ (\ref EdgeFunction).
      ///
      /// #### Problem resolution
      ///
      /// One solver (hpp::constraints::solver::BySubstitution) is created
      /// for each waypoint \f$p_i\f$.
      /// - method buildOptimizationProblem builds a matrix the rows of which
      ///   are the parameterizable numerical constraints present in the
      ///   graph, and the columns of which are the waypoints. Each value in the
      ///   matrix defines the status of each constraint right hand side for
      ///   this waypoint, among {absent from the solver,
      ///                         equal to value for previous waypoint,
      ///                         equal to value for start configuration,
      ///                         equal to value for end configuration}.
      /// - method MultipleStateOptimization::solveOptimizationProblem loops over
      ///   the waypoint solvers, solves for each waypoint after
      ///   initializing the right hand sides with the proper values.
      /// - eventually method buildPath build paths between waypoints with
      ///   the constraints of the transition in which the path lies.
      ///
      /// #### Current status
      ///
      /// The method has been successfully tested with romeo holding a placard
      /// and the construction set benchmarks. The result is satisfactory
      /// except between pregrasp and grasp waypoints that may be far
      /// away from each other if the transition between those state does
      /// not contain the grasp complement constraint. The same holds
      /// between placement and pre-placement.
      class HPP_MANIPULATION_DLLAPI MultipleStateOptimization
      {

        public:
          struct OptimizationData;

        virtual ~MultipleStateOptimization () {};

          static MultipleStateOptimizationPtr_t create
	          (const ProblemConstPtr_t& problem);

          /// \warning core::Problem will be casted to Problem
          static MultipleStateOptimizationPtr_t create
            (const core::ProblemConstPtr_t& problem);      

          MultipleStateOptimizationPtr_t copy () const;

          ////// Previously inherited from SteeringMethod
          
          core::ProblemConstPtr_t problem() const
          {
            return problem_;
          }

          /// create a path between two configurations
          /// \return a Path from q1 to q2 if found. An empty
          /// Path if Path could not be built.
          /// \note if q1 == q2, the steering method should not return an empty
          ///       shared pointer.
          core::Configurations_t operator() (ConfigurationIn_t q1,
              ConfigurationIn_t q2) const
          {
            core::Configurations_t path;
            try {
              path = compute (q1, q2);
            } catch (const core::projection_error& e) {
              hppDout (info, "Could not build path: " << e.what());
            }
            assert (q1 != q2 || !path.empty());
            return path;
          }

        protected:
          MultipleStateOptimization (const ProblemConstPtr_t& problem) :
            problem_ (problem), sameRightHandSide_ (), weak_ ()
          {
            gatherGraphConstraints ();
          }

          MultipleStateOptimization (const MultipleStateOptimization& other) :
            problem_ (other.problem_), constraints_ (), index_ (other.index_),
            sameRightHandSide_ (other.sameRightHandSide_),  weak_ ()
          {}

          core::Configurations_t compute (ConfigurationIn_t q1, ConfigurationIn_t q2) const;

          void init (MultipleStateOptimizationWkPtr_t weak)
          {
            weak_ = weak;
          }

        private:
          typedef constraints::solver::BySubstitution Solver_t;
          struct GraphSearchData;

          /// Gather constraints of all edges
          void gatherGraphConstraints ();

          /// Step 1 of the algorithm
          /// \return whether the max depth was reached.
          bool findTransitions (GraphSearchData& data) const;

          /// Step 2 of the algorithm
          graph::Edges_t getTransitionList (GraphSearchData& data, const std::size_t& i) const;

          /// Step 3 of the algorithm
          bool buildOptimizationProblem
            (OptimizationData& d, const graph::Edges_t& transitions) const;

          /// Step 4 of the algorithm
          bool solveOptimizationProblem (OptimizationData& d) const;

          bool checkConstantRightHandSide (OptimizationData& d,
                                           size_type index) const;

          core::Configurations_t buildPath (OptimizationData& d, const graph::Edges_t& edges) const;

          bool contains (const Solver_t& solver, const ImplicitPtr_t& c) const;


          /// A pointer to the manipulation problem
          ProblemConstPtr_t problem_;

          /// Vector of parameterizable edge numerical constraints
          NumericalConstraints_t constraints_;
          /// Map of indexes in constraints_
          std::map < std::string, std::size_t > index_;

          /// associative map that stores pairs of constraints of the form
          /// (constraint, constraint/hold)
          std::map <ImplicitPtr_t, ImplicitPtr_t> sameRightHandSide_;

          /// Weak pointer to itself
          MultipleStateOptimizationWkPtr_t weak_;

      }; // class MultipleStateOptimization
      /// \}
      

    } // namespace pathPlanner
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_STEERING_METHOD_MULTIPLE_STATE_OPTIMIZATION_HH
