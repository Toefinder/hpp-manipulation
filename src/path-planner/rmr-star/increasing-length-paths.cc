











#define HPP_DEBUG
#include "hpp/manipulation/path-planner/rmr-star/increasing-length-paths.hh"

namespace hpp {
  namespace manipulation {
    namespace pathPlanner {
      namespace rmrStar {

        IncreasingLengthPaths::IncreasingLengthPaths(const GraphPtr_t& graph)
          : graph_ (graph)
        {
        }

        void IncreasingLengthPaths::setInitState(const ConfigurationPtr_t& q_init)
        {
          setInitState(q_init, graph_->getState(*q_init));
        }
        
        void IncreasingLengthPaths::setInitState(const ConfigurationPtr_t& q_init,
            const StatePtr_t& s_init)
        {
          q_init_ = q_init;
          s_init_ = s_init;
          queue_ = PathQueue_t();
          queue_.push({s_init_});
        }
        
        void IncreasingLengthPaths::addGoalState(const ConfigurationPtr_t& q_goal)
        {
          addGoalState(q_goal, graph_->getState(*q_goal));
        }

        void IncreasingLengthPaths::addGoalState(const ConfigurationPtr_t& q_goal,
            const StatePtr_t& s_goal)
        {
          if (!s_goal->isWaypoint() &&
              std::find(s_goals_.begin(), s_goals_.end(), s_goal) == s_goals_.end())
          {
            q_goals_.push_back(q_goal);
            s_goals_.push_back(s_goal);
          }
        }
        
        void IncreasingLengthPaths::resetGoalStates()
        {
          q_goals_.clear();
          s_goals_.clear();
        }

        void IncreasingLengthPaths::initialize()
        {
          hppDout(info, "ilp initialize entered");
          States_t statesWithLoops;
          States_t allStates;
          const std::size_t nbComp = graph_->nbComponents();
          for (std::size_t i = 0; i < nbComp; ++i) {
            const graph::GraphComponentPtr_t graphComp (graph_->get(i));
            graph::EdgePtr_t edge (HPP_DYNAMIC_PTR_CAST(graph::Edge, graphComp));
            if(edge && (edge->stateFrom() == edge->stateTo()))
                statesWithLoops.push_back(edge->stateTo());
            graph::StatePtr_t state (HPP_DYNAMIC_PTR_CAST(graph::State, graphComp));
            if (state)
              allStates.push_back(state);
          }

          edges_.resize(allStates.size(), allStates.size());
          for (std::size_t i = 0; i < allStates.size(); i++) {
            for (std::size_t j = 0; j < i; j++) {
              edges_(i,j) = graph_->getEdges(allStates[i], allStates[j]);
              edges_(j,i) = graph_->getEdges(allStates[j], allStates[i]);
              assert(edges_(i,j).size() <= 1);
              assert(edges_(j,i).size() <= 1);
            }
            edges_(i,i) = graph_->getEdges(allStates[i], allStates[i]);
            assert(edges_(i,i).size() <= 1);
            index_[allStates[i]] = i;
          }

        }

        bool IncreasingLengthPaths::inGoal(const StatePtr_t& s) const
        {
          return std::find(s_goals_.begin(), s_goals_.end(), s) != s_goals_.end();
        }

        bool IncreasingLengthPaths::nextPath(States_t& path)
        {
          bool ok = false;
          while ((ok = nextPathAnyGoal(path)) && !inGoal(path.back()));
          return ok;
        }

        bool IncreasingLengthPaths::nextPathAnyGoal(States_t& path)
        {
          if (queue_.empty()) return false;
          path = queue_.front();
          updateQueue(path);
          return true;
        }

        static void followWaypointPath(size_t& state1Id, StatePtr_t& state2)
        {
          Neighbors_t state2Neighbors = state2->neighbors();
          assert(state2Neighbors.size() == 2);
          // One of the two neighbors is state1, we need the second because
          // we don't want to turn back in the middle of a waypoint edges path
          for (Neighbors_t::iterator it = state2Neighbors.begin();
            it != state2Neighbors.end(); ++it) {
            if (it->second->stateTo()->id() != state1Id) {
              state1Id = state2->id();
              state2 = it->second->stateTo();
              return;
            }
          }
        }

        void IncreasingLengthPaths::updateQueue(const States_t& path)
        {
          queue_.pop();
          assert(path.size() >= 1);
          StatePtr_t pathEnd = path.back();
          Neighbors_t neighbors = pathEnd->neighbors();
          States_t nextPath = path;
          for (Neighbors_t::iterator it = neighbors.begin();
            it != neighbors.end(); ++it) {
            // When going through a waypoint state, we should follow the
            // path of waypoint edges until reaching an actual state
            size_t beforeNextStateId = pathEnd->id();
            StatePtr_t nextState = it->second->stateTo();
            while (nextState->isWaypoint()) {
              nextPath.push_back(nextState);
              followWaypointPath(beforeNextStateId, nextState);
            }
              
            nextPath.push_back(nextState);
            queue_.push(nextPath);
            nextPath.pop_back();

            while (nextPath[nextPath.size() - 1]->isWaypoint())
              nextPath.pop_back();
          }
        }

        States_t IncreasingLengthPaths::uniqueStatesIn(const States_t& path) const
        {
          States_t ans;
          for (const graph::StatePtr_t& s: path)
            if (!s->isWaypoint() && std::find(ans.begin(), ans.end(), s) == ans.end())
              ans.push_back(s);
          return ans;
        }

        static ConstrMap_t mergeConstrMaps(const ConstrMap_t& map1, const ConstrMap_t& map2)
        {
            ConstrMap_t map;
            for (ConstrMap_t::const_iterator it = map1.begin(); it != map1.end(); it++) {
              const ImplicitPtr_t& key = it->first;
              if (map2.count(key) == 1) {
                assert(map2.find(key)->second == it->second);
                map[it->first] = it->second;
              }
            }
            return map;
        }

        bool IncreasingLengthPaths::seemsWorkable(const States_t& path)
        {
          hppDout(info, "seemsWorkable entered");
          //return uniqueStatesIn(path).size() > 1;
          if (path.size() <= 1) return false;

          // Find a possible goal configuration corresponding to the path end
          StatePtr_t s_goal;
          size_t i = 0;
          for (States_t::iterator its = s_goals_.begin(); its != s_goals_.end(); its++, i++) {
            s_goal = (*its);
            if (path.back() == s_goal)
              break;
          }
          assert(i < q_goals_.size());
          ConfigurationPtr_t q_goal = q_goals_[i];

          hppDout(info, "q_goal recovered");

          // Recover edges
          Edges_t pathEdges;
          size_t lastState = index_[path[0]];
          size_t curState = lastState;
          for (size_t i=1; i<path.size(); i++) {
            if (true || !path[i]->isWaypoint()) {
              curState = index_[path[i]];
              pathEdges.push_back(edges_(lastState, curState).front());
              lastState = curState;
            }
          }

          hppDout(info, "edges recovered");

          // Recover edge constraints
          ConstrMap_t intersectionOfConstraints = registerConstraints(pathEdges[0]);
          for (size_t i=1; i<pathEdges.size(); i++) {
            ConstrMap_t nextConstraintMap = registerConstraints(pathEdges[i]);
            intersectionOfConstraints = mergeConstrMaps(
              intersectionOfConstraints, nextConstraintMap);
          }

          /*
          

          */

          hppDout(info, "edge constraints recovered");

          // If some constraint changes RHS between q_init_ and q_goal, it's a fail
          hppDout(info, "constraints in all edges: " << intersectionOfConstraints.size());
          for (ConstrMap_t::const_iterator it = intersectionOfConstraints.begin();
              it != intersectionOfConstraints.end(); it++) {
            const ImplicitPtr_t& constraint = it->first;
            const DifferentiableFunctionPtr_t& constrFun = (it->first)->functionPtr();
            //vector_t rhsInit (constraint->function ().outputSpace ()->nq ());
            LiegroupElement rhsInit = (*constrFun)(q_init_->array());
            LiegroupElement rhsGoal = (*constrFun)(q_goal ->array());
            if (rhsInit != rhsGoal) return false;
          }

          return true;
        }

        ConstrMap_t IncreasingLengthPaths::registerConstraints(const EdgePtr_t& edge)
        {
          hppDout(info, "registerConstraints entered for edge " << edge->name());
          ConstrMap_t ans;
          ConfigProjectorPtr_t configProjector (edge->
            pathConstraint()->configProjector());
          // path constraint throws if edge not "initialized"
          core::NumericalConstraints_t edgeConstraints =
            configProjector->numericalConstraints();
          core::BySubstitution solver = configProjector->solver();
          for (const ImplicitPtr_t& constraint: edgeConstraints) {
            vector_t rhs (constraint->function ().outputSpace ()->nq ());
            solver.getRightHandSide(constraint, rhs);
            ans[constraint] = rhs;
          }
          hppDout(info, "registeredConstraints done");
          return ans;
        }

      } // namespace rmrStar
    } // namespace pathPlanner
  } // namespace manipulation
} // namespace hpp
