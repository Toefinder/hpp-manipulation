// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#ifndef HPP_MANIPULATION_GRAPH_NODE_HH
# define HPP_MANIPULATION_GRAPH_NODE_HH

#include <hpp/core/constraints-set.hh>

#include "hpp/manipulation/fwd.hh"
#include "hpp/manipulation/graph/fwd.hh"
#include "hpp/manipulation/graph/edge.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      /// State of an end-effector.
      ///
      /// Nodes of the graph of constraints. There is one
      /// graph for each end-effector.
      class HPP_MANIPULATION_DLLAPI Node
      {
        public:
          /// Check whether the configuration is in this state.
          /// \return True if this state contains this configuration
          /// \param config The configuration to be tested.
          /// \note You should note use that to know in which states a
          /// configuration is. This only checks if the configuration satisfies
          /// the constraints. Instead, use the class NodeSelector.
          virtual bool contains(const Configuration_t config) const;

        private:
          /// List of possible motions from this state (i.e. the outgoing
          /// vertices).
          std::vector < EdgePtr_t > neighbors_;

          /// Set of constraints to be statisfied.
          ConstraintPtr_t constraints_;

          /// A selector that will implement the selection of the next state.
          NodeSelectorPtr_t selector_;
      }; // class Node
    } // namespace graph
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_NODE_HH