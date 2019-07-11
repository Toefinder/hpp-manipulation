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
#define HPP_DEBUG
#include <hpp/manipulation/path-planner/rmr-star/contact-state.hh>

#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>

#include <hpp/constraints/solver/by-substitution.hh>

#include <hpp/core/config-projector.hh>
#include <hpp/core/constraint-set.hh>

#include <hpp/core/constraint-set.hh>
#include <hpp/core/config-projector.hh>

#include <hpp/manipulation/graph/state.hh>

namespace hpp {
  namespace manipulation {
    namespace pathPlanner {
      namespace rmrStar {
        static core::ConstraintSetPtr_t emptyConstraints ()
        {
          pinocchio::DevicePtr_t dev (pinocchio::Device::create (""));
          core::ConstraintSetPtr_t cs (core::ConstraintSet::create (dev, ""));
          core::ConfigProjectorPtr_t proj
            (core::ConfigProjector::create (dev, "", 0, 0));
          cs->addConstraint (proj);
          return cs;
        }
        ContactState::ContactState () :
          state_ (), rightHandSide_ (), constraints_ (emptyConstraints ()),
          rightHandSides_()
        {
        }

        ContactState::ContactState
        (const graph::StatePtr_t& state, ConfigurationIn_t config,
         const core::ConstraintSetPtr_t& constraints) :
          state_ (state), rightHandSide_ (),
          constraints_ (HPP_STATIC_PTR_CAST (core::ConstraintSet,
                                             constraints->copy ())),
          config_(config), rightHandSides_()
        {
          BySubstitution& solver
            (const_cast <BySubstitution&> (constraints_->configProjector ()->
                                           solver ()));
          hppDout (info, pinocchio::displayConfig (config));
          assert (state->contains (config));
          rightHandSide_ = solver.rightHandSideFromConfig (config);
          core::NumericalConstraints_t num = solver.numericalConstraints();

          for (std::size_t i=0 ; i<num.size() ; i++) {
            constraints::ImplicitPtr_t c = num[i];
            constraints::vector_t rhs (c->rightHandSideSize ());
            bool success (solver.getRightHandSide(num[i],rhs));
            assert (success);
            assert (rightHandSides_.count (c) == 0);
            rightHandSides_.insert
              (std::pair<constraints::ImplicitPtr_t,constraints::vectorIn_t>
               (c,rhs));
          }
        }

        ContactState::ContactState (const ContactState& other) :
          state_ (other.state_), rightHandSide_ (other.rightHandSide_),
          constraints_ (other.constraints_), config_ (other.config_),
          rightHandSides_ (other.rightHandSides_)
        {
        }

        ContactState& ContactState::operator=
        (const ContactState& other)
        {
          state_ = other.state_;
          rightHandSide_ = other.rightHandSide_;
          constraints_ = other.constraints_;
          config_ = other.config_;
          rightHandSides_ = other.rightHandSides_;
          return *this;
        }

        const ContactState::BySubstitution& ContactState::solver () const
        {
          assert (state_);
          return constraints_->configProjector ()->solver ();
        }

        ////////////////////////////////////////////////////////////////////////////
        bool smaller (ContactState a, ContactState b){
          if (a.state () < b.state ()){
            return true;
          }
          else{return false; }

          if (a.rightHandSide ().size () < b.rightHandSide ().size ()){
            return true;
          }
          else {
            return false;
          }
          for (int i=0; i < a.rightHandSide ().size(); ++i)	{
            if (a.rightHandSide ()[i]<b.rightHandSide ()[i]){
              return true;
            }
            else {
              return false;
            }
          }
        }
        bool operator< (const ContactState& c1 ,
                        const ContactState& c2)
        {
          return smaller (c1,c2);
        }
      } // namespace rmrStar
    } // namespace pathPlanner
  }//end namespace manipulation
}//end namespace hpp
