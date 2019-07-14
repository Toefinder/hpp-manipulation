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
        /// vector of leaves
        typedef std::vector <ContactState> ContactStates_t;

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
	typedef std::map <std::string, std::vector <constraints::vector_t> >
          RightHandSides_t;
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

	/// Computation step of the algorithm
	enum STEP {
	  SAMPLE_INTERSECTION_STATES,
	  BUILD_ROADMAP
	};

	STEP step_;

        /// Pair (problem, roadmap)
	typedef std::pair<core::Problem, core::RoadmapPtr_t>
          ProblemAndRoadmap_t;
        /// Map of problems and roadmaps indexed by contact states.
	typedef std::map <ContactState , ProblemAndRoadmap_t>
          LeafRoadmaps_t;
	LeafRoadmaps_t leafRoadmaps_;
        /// Leaves to explore in each state
        std::map <graph::StatePtr_t, ContactStates_t> leaves_;
        /// Index in allStates_ from shared pointer to state
        std::map <graph::StatePtr_t, size_type> index_;
        /// Matrix of inclusion between states
        /// if stateInclusion_ (s1->id (), s2->id ()) is true, s1 is included
        /// in s2.
        Eigen::Matrix <bool, Eigen::Dynamic, Eigen::Dynamic> stateInclusion_;
        /// Matrix of intersection between states
        // if stateIntersection (i1, i2) = s,
        // statesToSample_ [i1] inter statesToSample_ [i2] = s
        Eigen::Matrix <graph::StatePtr_t, Eigen::Dynamic, Eigen::Dynamic>
          stateIntersection_;
        /// Shooter that uniformly samples states with minimal number of
        /// implicit constraints.
        StateShooterPtr_t stateShooter_;
	/// Pointer to the problem
        const Problem& manipulationProblem_;

	/// Pointer to the PathPlanner roadmap as a manipulation::roadmap
        const RoadmapPtr_t roadmap_;

	///Pointer to the graph problem
	graph::GraphPtr_t graph_;

	///Number of sample contact trials before shooting
	///a configuration with a random right hand side
	size_type setRhsFreq_;

        /// Mapping between states and loop transitions.
	TransitionMap_t transition_;

        /// Vector of all states including waypoint states
        graph::States_t allStates_;

        /// Vector of states with loop transitions
        graph::States_t statesWithLoops_;

        /// Vector of states with transitions of minimal reduced dimension
        graph::States_t statesToSample_;
	///Counter
	int counter_;

	///////////////////////////////////////////////////////////////////
	//////////////////Members functions declaration
	//////////////////////////////////////////////////////////////////

	///Complete the map transition_ with the states of the graph as key and
	///its loop edge associated as value
	void initialize ();

	///Compute a roadmap in the initial configuration and final configuration leaf
	///and try to connect them
	void startSolve ();

      private:

        /// store right hand sides of constraints for given configuration
        /// in map rightHandSides_
        void registerRightHandSides (const NumericalConstraints_t& constraints,
                                     const Configuration_t& q);


        /// Add a configuration to a leaf roadmap
        ///
        /// If leafRoadmaps_ already contains a pair (problem, roadmap)
        /// for this contact state, configuration q is added to the
        /// roadmap. Otherwise, a roadmap is created and registered in
        /// leafRoadmaps_.
        void addConfigToLeafRoadmap (const ContactState& contactState,
                                     const ConfigurationPtr_t& q);

        /// Sample intersection states
        void sampleIntersectionStates ();

	///Store the contactStates, the roadmaps  and the problems associated
	/// of the visited leaves
	void createLeafRoadmap (const ContactState& contactState,
                                const ConfigurationPtr_t& q);

	///Compare the threshold of two solvers and return the biggest
	pinocchio::value_type biggestThreshold
	  (const BySubstitution& solver1, const BySubstitution& solver2);

}; // class RMRStar
    } // namespace pathPlanning
  } // namespace manipulation
} // namespace hpp
#endif
