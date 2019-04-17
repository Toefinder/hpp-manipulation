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
                               constraints::vector_t> RhsMap_t;
	RhsMap_t RhsMap_;

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

	///Random config used as q_init
	mutable ConfigurationPtr_t q_rand_;

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

	typedef std::map <graph::StatePtr_t, graph::EdgePtr_t> TransitionMap_t;
	TransitionMap_t transition_;
        /// Pair (problem, roadmap)
	typedef std::pair<core::Problem, core::RoadmapPtr_t>
          ProblemAndRoadmap_t;
        /// Map of problems and roadmaps indexed by contact states.
	typedef std::multimap <ContactState , ProblemAndRoadmap_t>
          LeafRoadmaps_t;
	LeafRoadmaps_t leafRoadmaps_;


	/// Pointer to the problem
        const Problem& manipulationProblem_;

	/// Pointer to the PathPlanner roadmap as a manipulation::roadmap
        const RoadmapPtr_t roadmap_;

	///Number of sample contact trials before shooting
	///a configuration with a random right hand side
	size_type setRhsFreq_;

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

	/// Return a vector containing the states of the graph
	/// \param input_map map containnig the states of the graph and its loop edge associated
	std::vector<graph::StatePtr_t> extract_keys(TransitionMap_t input_map);

	///Compute a roadmap in the initial configuration and final configuration leaf
	///and try to connect them
	void startSolve ();

	///Build a roadmap in the leaf of the ContactState_ configuration using kPRM*
	void buildRoadmap ();

      private:
	///Copy a roadmap in PathPlanner roadmap
	void copyRoadmapIntoGlobal (const core::RoadmapPtr_t& r);

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
	  ( constraints::solver::BySubstitution solver1,
	    constraints::solver::BySubstitution solver2);


	/// Connect two roadmaps (latestRoadmap_ and an other one)
	/// in adjacent states connected directly
	/// \li shoot a random config in the intersection of the two roadmaps
	/// \li connect it with the k nearest nodes of each roadmap
	/// \param roadmap the roadmap we want to connect with latestRoadmap_
	/// \param k the nb of nearest nodes we connect to the intersection node
	/// \param state the state of the roadmap leaf
	void connectDirectStates
	  (const constraints::NumericalConstraints_t& constraints,
           const constraints::NumericalConstraints_t& transitConstraints,
           core:: ValidationReportPtr_t& validationReport,
           int max_iter , size_type  k,
           const ConfigurationShooterPtr_t& shooter,
           HierarchicalIterative::Status constraintApplied,
           core::PathPtr_t& path, core::PathPtr_t& projpath,
           const PathProjectorPtr_t& pathProjector,
           const Configuration_t& config, bool valid,
           const graph::StatePtr_t& state,
           const core::RoadmapPtr_t& roadmap);

	/// Connect two roadmaps (latestRoadmap_ and an other one)
	/// in adjacent states connected using waypoints
	/// \li shoot a random config in the intersection of the two roadmaps
	/// \li identify wich waypoint correspond to the intersect one
	/// \li connect the waypoints from the intersection config to the first one
	/// \li connect the first waypoint with the k nearest nodes of interRoadmpap_
	/// \li connect the waypoints from the intersection config to the last one
	/// \li connect the last waypoint with the k nearest nodes of roadmap_
	/// \param roadmap the roadmap we want to connect with latestRoadmap_
	/// \param k the nb of nearest nodes we connect to the first and last waypoint
	/// \param state the state of the roadmap leaf
	void connectStatesByWaypoints
	  (const graph::WaypointEdgePtr_t& waypointEdge,
           const constraints::NumericalConstraints_t& constraints,
           const constraints::NumericalConstraints_t& transitConstraints,
           const RhsMap_t& rhsMap, int max_iter, size_type k,
           core:: ValidationReportPtr_t& validationReport, bool valid,
           HierarchicalIterative::Status constraintApplied,
           core::PathPtr_t& projpath, const PathProjectorPtr_t& pathProjector,
           const ConfigurationShooterPtr_t& shooter,
           const core::RoadmapPtr_t& roadmap,
           const graph::StatePtr_t& state, const Configuration_t& config);

	///Add two nodes to the roadmap_ and create an edge between them
      /// \param edge edge of the graph between the respective states of the nodes
      /// \param q1 the configuration of the first node
      /// \param configuration the configuration of the second node
	 void connectConfigToNode
           (const graph::EdgePtr_t& edge, core::PathPtr_t&  path,
            const core::PathProjectorPtr_t& pathProjector,
            core::PathPtr_t& projpath, const ConfigurationPtr_t& q1,
            const ConfigurationPtr_t& configuration);

	 ///Shoot a random config in the intersection of two leaves
         ConfigurationPtr_t createInterStateNode
           (const constraints::NumericalConstraints_t& constraints,
            const constraints::NumericalConstraints_t& transitConstraints,
            const core::ConfigValidationsPtr_t& configValidations,
            core:: ValidationReportPtr_t& validationReport,
            int max_iter, const ConfigurationShooterPtr_t& shooter,
            HierarchicalIterative::Status constraintApplied,
            const core::Configuration_t& config, bool valid,
            const graph::StatePtr_t& state);

      }; // class RMRStar
    } // namespace pathPlanning
  } // namespace manipulation
} // namespace hpp
#endif
