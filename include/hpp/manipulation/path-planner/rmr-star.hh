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

#ifndef HPP_MANIPULATION_PATH_PLANNER_RMR_STAR_HH
# define HPP_MANIPULATION_PATH_PLANNER_RMR_STAR_HH

# include <hpp/constraints/solver/hierarchical-iterative.hh>
# include <hpp/core/path-planner.hh>
# include <hpp/manipulation/graph/statistics.hh>
# include <hpp/manipulation/graph/fwd.hh>
# include <hpp/manipulation/fwd.hh>
# include <hpp/manipulation/path-planner/rmr-star/contact-state.hh>

namespace hpp {
  namespace manipulation {
    namespace pathPlanner {
      typedef constraints::solver::HierarchicalIterative HierarchicalIterative;
      typedef constraints::solver::BySubstitution BySubstitution;
      /// Implementation of RMR star
      ///
      /// This class implements algorithm RMR* as described in
      /// "Optimal Sampling-Based Manipulation Planner" by P. Schmitt,
      /// W. Neubauer, K. Wurm, G. Wichert, and W. Burgard.
      class HPP_MANIPULATION_DLLAPI RMRStar : public core::PathPlanner
      {
      public:

        typedef rmrStar::ContactState ContactState;
	typedef std::map <graph::StatePtr_t, graph::EdgePtr_t> TransitionMap_t;

        /// Create an instance and return a shared pointer to the instance
        /// \param problem reference to the problem to be solved,
        /// \param roadmap roadmap to be expanded.
        static RMRStarPtr_t create (const core::Problem& problem,
                                    const core::RoadmapPtr_t& roadmap);

        /// One step of extension.
        ///
        /// A set of constraints is chosen using the graph of constraints.
        /// A constraint extension is done using a chosen set.
        ///
        virtual void oneStep ();

	/// Map linking functions and its right hand side
	typedef std::multimap <constraints::ImplicitPtr_t,
                               constraints::vector_t> RightHandSides_t;
	RightHandSides_t rightHandSides_;

     protected:
        /// Protected constructor
        RMRStar (const core::Problem& problem,
		 const core::RoadmapPtr_t& roadmap);

      private:

	/// Pointer to the kPrmStar method
	core::pathPlanner::kPrmStarPtr_t kPrm_;

	///Pointer to the edge steering method
	core::SteeringMethodPtr_t edgeSteeringMethod_;

	/// Latest roadmap built in a leaf
	core::RoadmapPtr_t latestRoadmap_;

	/// latest contact state
	ContactState latestLeaf_;

	///Pointer to the graph problem
	graph::GraphPtr_t graph_;

	/// Computation step of the algorithm
	enum STEP {
	  BUILD_ROADMAP,
	  CONNECT_ROADMAPS
	};

	STEP step_;

        /// Pair (problem, roadmap)
	typedef std::pair<core::Problem, core::RoadmapPtr_t>
          ProblemAndRoadmap_t;
        /// Map of problems and roadmaps indexed by contact states.
	typedef std::multimap <ContactState , ProblemAndRoadmap_t>
          LeafRoadmaps_t;
	LeafRoadmaps_t leafRoadmaps_;
        /// Matrix of inclusion between states
        /// if stateInclusion_ (s1->id (), s2->id ()) is true, s1 is included
        /// in s2.
        Eigen::Matrix <bool, Eigen::Dynamic, Eigen::Dynamic> stateInclusion_;
        /// Shooter that uniformly samples states with minimal number of
        /// implicit constraints.
        StateShooterPtr_t stateShooter_;
	/// Pointer to the problem
        const Problem& manipulationProblem_;

	/// Pointer to the PathPlanner roadmap as a manipulation::roadmap
        const RoadmapPtr_t roadmap_;

	///Number of sample contact trials before shooting
	///a configuration with a random right hand side
	size_type setRhsFreq_;

        /// Mapping between states and loop transitions.
	TransitionMap_t transition_;

	///Counter
	int counter_;

	///////////////////////////////////////////////////////////////////
	//////////////////Members functions declaration
	//////////////////////////////////////////////////////////////////

	/// Shot a random config in a random state of the graph and create
	///the associatedContactState
	ContactState sampleContact ();

	///Complete the map transition_ with the states of the graph as key and
	///its loop edge associated as value
	void computeTransitionMap ();

	///Compute a roadmap in the initial configuration and final configuration leaf
	///and try to connect them
	void startSolve ();

	///Build a roadmap in the leaf of the ContactState_ configuration using kPRM*
	void buildRoadmap ();

      private:
	///Copy a roadmap in PathPlanner roadmap
	void copyRoadmapIntoGlobal (const core::RoadmapPtr_t& r);

        /// Find sequence of edges connecting two states
        ///
        /// An empty sequence is returned if none of the two following
        /// conditions are not met
        /// \li s2 is reachable from s1 by an edge,
        /// \li s2 is reachable from s1 by traversing a state that is
        ///     included in s1 and s2.
        ///
        /// The result is the concatenation of edges connecting s1 to s2.
        /// Waypoint edges are expanded
        std::deque <graph::EdgePtr_t> getConnectionBetweenStates
          (const graph::StatePtr_t& s1, const graph::StatePtr_t& s2);

        ///Connect the roadmaps build on different leaves
	void connectRoadmap ();

	///Store the contactStates, the roadmaps  and the problems associated
	/// of the visited leaves
	void storeLeafRoadmap ();

	///Store in RhsMap_ the functions's right hand side already visited
	void storeRhs ();

	/// Return true if the right hand side contains only zeros
	bool rhsNull (vector_t rhs);

	///Compare the threshold of two solvers and return the biggest
	pinocchio::value_type biggestThreshold
	  (const BySubstitution& solver1, const BySubstitution& solver2);


	/// Connect two roadmaps (latestRoadmap_ and an other one)
	///
        /// roadmaps should be in states that have a non empty intersection.
        /// They should be
        /// \li either in the same state and the same leaf,
        /// \li either in two states separated by a state that lies at the
        ///     intersection of the former.
	/// \li shoot a random config in the intersection of the two leaves
	/// \li identify wich waypoint correspond to the intersect one
	/// \li connect the waypoints from the intersection config to the first
        ///     one
	/// \li connect the first waypoint with the k nearest nodes of
        ///     interRoadmpap_
	/// \li connect the waypoints from the intersection config to the last
        ///     one
	/// \li connect the last waypoint with the k nearest nodes of roadmap_
        ///
        /// \param connectedEdges list of edges linking the states of the
        ///        roadmap. This can correspond to one of two edges. Waypoint
        ///        edges are expanded.
        /// \param currentLeaf, latestLeaf Leaves in which the roadmap lie.
	/// \param roadmap the roadmap we want to connect with latestRoadmap_
	/// \param k the nb of nearest nodes we connect to the first and last
        ///        waypoint
	/// \param state the state of the roadmap leaf
	void connectContactStateRoadmaps
          (const std::deque <graph::EdgePtr_t>& connectedEdges,
           const ContactState& currentLeaf, const ContactState& latestLeaf,
           size_type k,
           core::ValidationReportPtr_t& validationReport, bool valid,
           const core::RoadmapPtr_t& roadmap);

	///Add two nodes to the roadmap_ and create an edge between them
      /// \param edge edge of the graph between the respective states of the nodes
      /// \param q1 the configuration of the first node
      /// \param configuration the configuration of the second node
	 void connectConfigToNode
           (const graph::EdgePtr_t& edge, const ConfigurationPtr_t& q1,
            const ConfigurationPtr_t& configuration);

	 ///Shoot a random config in the intersection of two leaves
         ConfigurationPtr_t createInterStateConfig
           (const ContactState& currentLeaf, const ContactState& latestLeaf,
            const core::ConfigValidationsPtr_t& configValidations,
            core::ValidationReportPtr_t& validationReport, bool valid,
            const graph::StatePtr_t& currentState);
}; // class RMRStar
    } // namespace pathPlanning
  } // namespace manipulation
} // namespace hpp
#endif
