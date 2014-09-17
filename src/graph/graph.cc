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

#include <hpp/util/assertion.hh>

#include "hpp/manipulation/robot.hh"
#include "hpp/manipulation/graph/node-selector.hh"
#include "hpp/manipulation/graph/node.hh"
#include "hpp/manipulation/graph/edge.hh"
#include "hpp/manipulation/graph/graph.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      static std::string toString (const Nodes_t& n) {
        std::string nodesStr = "(";
        for (graph::Nodes_t::const_iterator itNode = n.begin();
            itNode != n.end (); itNode++)
          nodesStr += (*itNode)->name () + " / ";
        nodesStr += ")";
        return nodesStr;
      }

      static std::string toString (const Edges_t& e) {
        std::string edgesStr = "(";
        for (graph::Edges_t::const_iterator itEdge = e.begin();
            itEdge != e.end (); itEdge++)
          edgesStr += (*itEdge)->name () + " / ";
        edgesStr += ")";
        return edgesStr;
      }

      GraphPtr_t Graph::create(RobotPtr_t robot)
      {
        Graph* ptr = new Graph;
        GraphPtr_t shPtr (ptr);
        ptr->init (shPtr, robot);
        return shPtr;
      }

      void Graph::init (const GraphWkPtr_t& weak, RobotPtr_t robot)
      {
        GraphComponent::init (weak);
        robot_ = robot;
        wkPtr_ = weak;
      }

      NodeSelectorPtr_t Graph::createNodeSelector()
      {
        NodeSelectorPtr_t newNodeSelector = NodeSelector::create();
        nodeSelectors_.push_back(newNodeSelector);
        newNodeSelector->parentGraph (wkPtr_);
        return newNodeSelector;
      }

      void Graph::maxIterations (size_type iterations)
      {
        maxIterations_ = iterations;
      }

      size_type Graph::maxIterations () const
      {
        return maxIterations_;
      }

      void Graph::errorThreshold (const value_type& threshold)
      {
        errorThreshold_ = threshold;
      }

      value_type Graph::errorThreshold () const
      {
        return errorThreshold_;
      }

      const RobotPtr_t& Graph::robot () const
      {
        return robot_;
      }

      Nodes_t Graph::getNode(const Configuration_t config) const
      {
        Nodes_t nodes;
        for (NodeSelectors_t::const_iterator it = nodeSelectors_.begin();
            it != nodeSelectors_.end(); it++)
          nodes.push_back( (*it)->getNode(config) );
        return nodes;
      }

      static void buildPossibleEdges (Edges_t current,
          const std::vector <Edges_t>& edgesPerNodeSelector,
          std::vector <Edges_t>& possibleEdges)
      {
        size_t d = current.size();
        if (d == edgesPerNodeSelector.size()) {
          possibleEdges.push_back (current);
          return;
        }
        Edges_t::const_iterator it = edgesPerNodeSelector[d].begin();
        while (it != edgesPerNodeSelector[d].end()) {
          current.push_back (*it);
          buildPossibleEdges (current, edgesPerNodeSelector, possibleEdges);
          current.pop_back ();
          it++;
        }
      }

      std::vector <Edges_t> Graph::getEdge(const Nodes_t& from, const Nodes_t& to) const
      {
        assert (from.size() == to.size());
        assert (nodeSelectors_.size() == to.size());
        size_t numberPossibleEdges = 1;
        std::vector < Edges_t > edgesPerNodeSelector (nodeSelectors_.size());
        std::vector < Edges_t >::iterator itEdgePerNodeSelector = edgesPerNodeSelector.begin();
        Nodes_t::const_iterator itFrom = from.begin (),
                                itTo   =   to.begin ();
        // We first iterate through from. For each element of from,
        // we look for all edges between this element of from and its corresponding
        // element in to. The resulting set of Edges_t is stored in
        // edgesPerNodeSelector.

        // Temporary variable.
        Edges_t edgesInNodeSelector;
        const Neighbors_t* neighbors;
        Neighbors_t::const_iterator itEdge;
        while (itFrom != from.end()) {
          edgesInNodeSelector.clear ();
          neighbors = &((*itFrom)->neighbors ());
          itEdge = neighbors->begin();
          // Find the edges between *itFrom and *itTo
          while (itEdge != neighbors->end()) {
            if (itEdge->second->to() == (*itTo) )
              edgesInNodeSelector.push_back (itEdge->second);
            itEdge++;
          }
          /// If no Edge is found, the two Node are not connected.
          if (edgesInNodeSelector.empty ())
            return std::vector <Edges_t>(0);

          // Store the Edges.
          numberPossibleEdges *= edgesInNodeSelector.size();
          *itEdgePerNodeSelector = edgesInNodeSelector;

          itFrom++; itTo++; itEdgePerNodeSelector++;
        }
        assert (itTo == to.end());
        assert (itEdgePerNodeSelector == edgesPerNodeSelector.end());

        // Now, we can create the list of possible Edges_t
        // between from and to.
        std::vector <Edges_t> possibleEdges;
        buildPossibleEdges (Edges_t(), edgesPerNodeSelector, possibleEdges);
        assert (possibleEdges.size() == numberPossibleEdges);
        return possibleEdges;
      }

      Edges_t Graph::chooseEdge(const Nodes_t& nodes) const
      {
        Edges_t edges;
        for (Nodes_t::const_iterator it = nodes.begin();
            it != nodes.end(); it++)
          edges.push_back( (*it)->nodeSelector().lock()->chooseEdge(*it) );
        return edges;
      }

      NodeSelectorPtr_t Graph::getNodeSelectorByName (const std::string& name)
      {
        for (NodeSelectors_t::iterator it = nodeSelectors_.begin();
            it != nodeSelectors_.end(); it++) {
          if (name == (*it)->name())
            return *it;
        }
        return NodeSelectorPtr_t();
      }

      ConstraintSetPtr_t Graph::configConstraint (const Nodes_t& nodes)
      {
        ConstraintSetPtr_t constraint;
        MapFromNode::const_iterator it = constraintSetMapFromNode_.find (nodes);
        if (it == constraintSetMapFromNode_.end ()) {
          std::string name = toString (nodes);
          constraint = ConstraintSet::create (robot (), "Set " + name);

          ConfigProjectorPtr_t proj = ConfigProjector::create(robot(), "proj " + name, errorThreshold(), maxIterations());
          insertNumericalConstraints (proj);
          for (Nodes_t::const_iterator it = nodes.begin();
              it != nodes.end(); it++)
            (*it)->insertNumericalConstraints (proj);
          constraint->addConstraint (HPP_DYNAMIC_PTR_CAST(Constraint, proj));

          insertLockedDofs (constraint);
          for (Nodes_t::const_iterator it = nodes.begin();
              it != nodes.end(); it++)
            (*it)->insertLockedDofs (constraint);
          constraintSetMapFromNode_.insert (PairNodesConstraints(nodes, constraint));
        } else {
          constraint = it->second;
        }
        return constraint;
      }

      ConstraintSetPtr_t Graph::configConstraint (const Edges_t& edges, ConfigurationIn_t config)
      {
        ConstraintSetPtr_t constraint = configConstraint (edges);
        constraint->offsetFromConfig (config);
        return constraint;
      }

      ConstraintSetPtr_t Graph::configConstraint (const Edges_t& edges)
      {
        ConstraintSetPtr_t constraint;
        MapFromEdge::const_iterator it = cfgConstraintSetMapFromEdge_.find (edges);
        if (it == cfgConstraintSetMapFromEdge_.end ()) {
          std::string name = toString (edges);
          constraint = ConstraintSet::create (robot (), "Set " + name);

          ConfigProjectorPtr_t proj = ConfigProjector::create(robot(), "proj " + name, errorThreshold(), maxIterations());
          insertNumericalConstraints (proj);
          for (Edges_t::const_iterator it = edges.begin();
              it != edges.end(); it++) {
            (*it)->insertNumericalConstraints (proj);
            (*it)->to ()->insertNumericalConstraints (proj);
          }
          constraint->addConstraint (HPP_DYNAMIC_PTR_CAST(Constraint, proj));

          insertLockedDofs (constraint);
          for (Edges_t::const_iterator it = edges.begin();
              it != edges.end(); it++) {
            (*it)->insertLockedDofs (constraint);
            (*it)->to ()->insertLockedDofs (constraint);
          }
          cfgConstraintSetMapFromEdge_.insert (PairEdgesConstraints(edges, constraint));
        } else {
          constraint = it->second;
        }

        return constraint;
      }

      ConstraintSetPtr_t Graph::pathConstraint (const Edges_t& edges, ConfigurationIn_t config)
      {
        ConstraintSetPtr_t constraint = pathConstraint (edges);
        constraint->offsetFromConfig (config);
        return constraint;
      }

      ConstraintSetPtr_t Graph::pathConstraint (const Edges_t& edges)
      {
        ConstraintSetPtr_t constraint;
        MapFromEdge::const_iterator it = pathConstraintSetMapFromEdge_.find (edges);
        if (it == pathConstraintSetMapFromEdge_.end ()) {
          std::string name = toString (edges);
          constraint = ConstraintSet::create (robot (), "Set " + name);

          ConfigProjectorPtr_t proj = ConfigProjector::create(robot(), "proj " + name, errorThreshold(), maxIterations());
          insertNumericalConstraints (proj);
          for (Edges_t::const_iterator it = edges.begin();
              it != edges.end(); it++) {
            (*it)->insertNumericalConstraints (proj);
            (*it)->node ()->insertNumericalConstraintsForPath (proj);
          }
          constraint->addConstraint (HPP_DYNAMIC_PTR_CAST(Constraint, proj));

          insertLockedDofs (constraint);
          for (Edges_t::const_iterator it = edges.begin();
              it != edges.end(); it++) {
            (*it)->insertLockedDofs (constraint);
            (*it)->node ()->insertLockedDofs (constraint);
          }
          pathConstraintSetMapFromEdge_.insert (PairEdgesConstraints (edges, constraint));
        } else {
          constraint = it->second;
        }

        return constraint;
      }

      std::ostream& Graph::print (std::ostream& os) const
      {
        GraphComponent::print (os) << std::endl;
        for (NodeSelectors_t::const_iterator it = nodeSelectors_.begin();
            it != nodeSelectors_.end(); it++)
          os << *(*it);
        return os;
      }
    } // namespace graph
  } // namespace manipulation

} // namespace hpp
