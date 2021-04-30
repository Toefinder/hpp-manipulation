











#ifndef HPP_MANIPULATION_PATH_PLANNER_RMR_STAR_ILP_HH
# define HPP_MANIPULATION_PATH_PLANNER_RMR_STAR_ILP_HH

#include <queue>
#include <set>
#include <map>
#include <algorithm>

#include <hpp/util/pointer.hh>
#include <hpp/util/timer.hh>
#include <hpp/util/assertion.hh>

#include <hpp/pinocchio/configuration.hh>

# include <hpp/constraints/solver/hierarchical-iterative.hh>
# include <hpp/core/path-planner.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/core/nearest-neighbor.hh>
#include <hpp/core/roadmap.hh>

#include "hpp/manipulation/graph/statistics.hh"
#include "hpp/manipulation/problem.hh"
#include "hpp/manipulation/roadmap-node.hh"
#include "hpp/manipulation/graph/edge.hh"
# include <hpp/manipulation/graph/fwd.hh>
# include <hpp/manipulation/fwd.hh>
# include <hpp/manipulation/path-planner/rmr-star/contact-state.hh>


namespace hpp {
  namespace manipulation {
    namespace pathPlanner {
      namespace rmrStar {

        typedef core::Configurations_t Configurations_t;

        typedef core::Nodes_t Nodes_t;
        typedef core::NodePtr_t NodePtr_t;

        typedef graph::GraphPtr_t GraphPtr_t;
        typedef graph::StatePtr_t StatePtr_t;
        typedef graph::States_t States_t;
        typedef graph::Edges_t Edges_t;
        typedef graph::EdgePtr_t EdgePtr_t;
        typedef graph::Neighbors_t Neighbors_t;

        typedef std::queue<States_t> PathQueue_t;

        typedef std::map<ImplicitPtr_t, vector_t> ConstrMap_t;

        class HPP_MANIPULATION_DLLAPI IncreasingLengthPaths
        {
        public:
          IncreasingLengthPaths(const GraphPtr_t& graph);

          void setInitState(const ConfigurationPtr_t& q_init);
          void setInitState(const ConfigurationPtr_t& q_init, const StatePtr_t& s_init);
          
          void addGoalState(const ConfigurationPtr_t& q_goal);
          void addGoalState(const ConfigurationPtr_t& q_goal, const StatePtr_t& s_goal);

          void resetGoalStates();

          void initialize();

          bool nextPath(States_t& path);

          States_t uniqueStatesIn(const States_t& path) const;

          bool seemsWorkable(const States_t& path);

        private:
          GraphPtr_t graph_;
          Eigen::Matrix<Edges_t, Eigen::Dynamic, Eigen::Dynamic> edges_;
          std::map<StatePtr_t, size_t> index_ ;

          ConfigurationPtr_t q_init_;
          StatePtr_t s_init_;
          Configurations_t q_goals_;
          States_t s_goals_;

          PathQueue_t queue_;

          bool inGoal(const StatePtr_t& s) const;

          bool nextPathAnyGoal(States_t& path);

          void updateQueue(const States_t& path);

          ConstrMap_t registerConstraints(const EdgePtr_t& edge);

        }; // class IncreasingLengthPaths
      } // namespace rmrStar
    } // namespace pathPlanner
  } // namespace manipulation
} // namespace hpp
#endif
