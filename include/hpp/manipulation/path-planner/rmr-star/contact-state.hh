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

#ifndef HPP_MANIPULATION_RMR_STAR_CONTACT_STATE_HH
# define HPP_MANIPULATION_RMR_STAR_CONTACT_STATE_HH

# include <hpp/manipulation/fwd.hh>
# include <hpp/constraints/solver/by-substitution.hh>
# include <hpp/manipulation/graph/fwd.hh>

namespace hpp {
  namespace manipulation {
    namespace pathPlanner {
      namespace rmrStar {
	/// Structure defining the leaf of a foliation
        ///
        /// It contains
        ///   \li a configuration,
        ///   \li the state of the configuration in the constraint graph,
        ///   \li the right hand side of the constraint of the loop edge of
        ///       the state,
	///   \li the loop edge of the state, and
        ///   \li a map linking each constraint function of
	///the configuration to its right hand side
	class HPP_MANIPULATION_DLLAPI ContactState
        {
          typedef constraints::solver::BySubstitution BySubstitution;
	public:
          /// Map linking constraints and their right hand side
          typedef std::map <constraints::ImplicitPtr_t,
                            constraints::vector_t> RightHandSides_t;

	  ///Empty constructor
	  ContactState ();

	  ///Constructor
	  /// \param state the configuration's state
	  /// \param config the configuration
	  /// \param constraints the loop constraint of the state
          ///
          /// a pointer to the solver of the input constraints is stored
          /// and can be accessed in read only mode.
	  ContactState (const graph::StatePtr_t& state,
			ConfigurationIn_t config,
			const core::ConstraintSetPtr_t& constraints);

          /// Copy constructor
          ContactState (const ContactState& other);

	  /// Return state_
	  const graph::StatePtr_t& state () const
	  {
	    assert (state_);
	    return state_;
	  }

	  /// Return rightHandSide_
	  const constraints::vector_t& rightHandSide () const
	  {
	    assert (state_);
	    return rightHandSide_;
	  }

          /// Return constraint set containing the solver
          core::ConstraintSetPtr_t constraints () const
          {
            return constraints_;
          }

	  /// Return loopEdgeConstraint_
          const BySubstitution& solver () const;

	  /// Return config_
	  const Configuration_t& config () const
	  {
	    assert (state_);
	    return config_;
	  }

	  ///Return rightHandSides_
	  const RightHandSides_t& rightHandSides () const
	  {
	    assert (state_);
	    return rightHandSides_;
	  }

          ContactState& operator= (const ContactState& constactState);

	private:
	  graph::StatePtr_t state_;
	  constraints::vector_t rightHandSide_;
          core::ConstraintSetPtr_t constraints_;
	  Configuration_t config_;
	  RightHandSides_t rightHandSides_;
	};

	/// Compare to ContactState and return true if a is smaller than b
        bool operator< (const ContactState& c1 , const ContactState& c2);
      } // namespace rmrStar
    } // namespace pathPlanner
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_RMR_STAR_CONTACT_STATE_HH
