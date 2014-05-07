//
// Copyright (c) 2014 CNRS
// Authors: Mylene Campana, Florent Lamiraux
//
// This file is part of hpp-core
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_CORE_POTENTIAL_METHOD_HH
# define HPP_CORE_POTENTIAL_METHOD_HH

# include <hpp/core/steering-method.hh>
# include <limits>
# include <hpp/core/fwd.hh>
# include <hpp/core/edge.hh>
# include <hpp/core/node.hh>
# include <hpp/core/path-vector.hh>
# include <hpp/core/distance.hh>
# include <hpp/model/distance-result.hh>

# include "astar.hh" // for isGoal(..)

namespace hpp {
  namespace core {
    ///  Method based on potential fields that replaces straight path method
    ///
    class HPP_CORE_DLLAPI PotentialMethod : public SteeringMethod
    {
    public:
      PotentialMethod (const DevicePtr_t& device) :
	SteeringMethod (), device_ (device))
      {
      }

      // Construct tree for BFP algorithm
      NodePtr_t findPotentialNode (ConfigurationIn_t q1,
				      ConfigurationIn_t q2) const
      {
	// Call for BFP algorithm here, inspired by Astar with some changes
	closed_.clear ();
	open_.clear ();
	parent_.clear ();
	costFromStart_.clear ();
	open_.push_back (roadmap_->initNode ());
	value_type res = std::numeric_limits <value_type>::infinity ();

	while (!open_.empty ()) {
	  //open_.sort (SortFunctor (estimatedCostToGoal_)); // trier open_
	  Nodes_t::iterator itv = open_.begin ();
	  NodePtr_t current (*itv);
	  if (isGoal (current)) return current;
	  open_.erase (itv);
	  closed_.push_back (current);
	  for (Edges_t::const_iterator itEdge = current->outEdges ().begin ();
	       itEdge != current->outEdges ().end (); itEdge++) {
	    NodePtr_t child ((*itEdge)->to ());
	    if (std::find (closed_.begin (), closed_.end (), child) ==
		closed_.end ()) {
	      // node is not in closed set	      
	      bool childNotInOpenSet = (std::find (open_.begin (),
						   open_.end (),
						   child) == open_.end ());
	      const ConfigurationPtr_t q = current->configuration ();
	      computePotential(q);
	      if ((childNotInOpenSet) && (potential_ < res)) {
		parent_ [child] = *itEdge; // cree l'arc parent pour retour
		res = potential_;
		open_.push_back (child);
	      }//if
	    }//if
	  }//for
	}//while
	throw std::runtime_error ("BFP is stuck in a local minimum.");
      }//findPotentialNode


    /// create a path between two configurations
    virtual PathPtr_t impl_compute (ConfigurationIn_t q1,
				    ConfigurationIn_t q2) const
    {
      NodePtr_t node = findPotentialNode ();
      Edges_t edges;

      while (node) {//remonte de la fin au debut avec arcs "parents" crees
	Parent_t::const_iterator itEdge = parent_.find (node);
	if (itEdge != parent_.end ()) {
	  EdgePtr_t edge = itEdge->second;
	  edges.push_front (edge);
	  node = edge->from ();
	}
	else node = NodePtr_t (0x0);
      }
      PathVectorPtr_t pathVector;
      for (Edges_t::const_iterator itEdge = edges.begin ();
	   itEdge != edges.end (); itEdge++) {
	const PathPtr_t& path ((*itEdge)->path ());
	if (!pathVector)
	  pathVector = PathVector::create (path->outputSize ());
	pathVector->appendPath (path);
      }
      return pathVector;
    }


  private:
    // Compute potential (attractive and repulsive) for a device configuration
    void computePotential(ConfigurationIn_t q)
    {
      value_type kAtt = 1;
      value_type kRep = 1;
      double kRange = 0.1;
      value_type distGoal = (*distance) (q,qGoal);
      value_type potentialAtt = 1/2*kAtt*distGoal*distGoal;
      value_type potentialRep = 0;
      double& distObst = DistanceResult.distance;
	if (distObst < kRange){
	  value_type potentialRep = 
	  1/2*kRep*(1/distObst - 1/kRange)*(1/distObst - 1/kRange)};
      potential_ = potentialAtt + potentialRep;
      return;
    }

    void computeMinDistance ()
    {
      const DistanceResults_t& dr = device_->distanceResults ();
      distancesType distList; //TODO : vecteur de double
      for (DistanceResults_t::const_iterator itDistance = dr.begin ();
	   itDistance != dr.end (); itDistance++) {
	//(*distances_ptr) [distPairId] = itDistance->fcl.min_distance;
	nomListe.push_back(itDistance->fcl.min_distance);
	minDistance_ = (value_type) min(distList) // cast double -> value_type
	  // std::fmin ?? include<algorithm ou cmath> ?? pour faire le min
	return;
    }

    /*struct SortFunctor {
	std::map <NodePtr_t, value_type>& cost_;
	SortFunctor (std::map <NodePtr_t, value_type>& cost) :
	  cost_ (cost) {}
	bool operator () (const NodePtr_t& n1, const NodePtr_t& n2)
	{
	  return cost_ [n1] < cost_ [n2];
	}
	}; // struc SortFunctor*/

      DeviceWkPtr_t device_;
      Configuration_t qGoal_;
      value_type potential_; // U(q)
      value_type minDistance_; // min distance between device and obstacles
      // stuff from Astar :
      typedef std::list < NodePtr_t > Nodes_t;
      typedef std::list <EdgePtr_t> Edges_t;
      typedef std::map <NodePtr_t, EdgePtr_t> Parent_t;
      Nodes_t closed_;
      Nodes_t open_;

    }; // PotentialMethod
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_POTENTIAL_METHOD_HH
