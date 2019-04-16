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

#include <hpp/manipulation/path-planner/rmr-star/contact-state.hh>

#include <hpp/constraints/solver/by-substitution.hh>

#include <hpp/core/constraint-set.hh>
#include <hpp/core/config-projector.hh>

namespace hpp {
  namespace manipulation {
    namespace pathPlanner {
      namespace rmrStar {
        ContactState::ContactState () : state_ (), rightHandSide_ (),
                                        loopEdgeConstraint_ (), config_ (),
                                        rhsMap_()
        {
        }

        ContactState::ContactState
        (const graph::StatePtr_t& state, ConfigurationIn_t config,
         const core::ConstraintSetPtr_t& constraints) :
          state_ (state), rightHandSide_ (),
          loopEdgeConstraint_ (constraints), config_(config),rhsMap_()
        {
          assert (loopEdgeConstraint_);
          assert (loopEdgeConstraint_->configProjector ());
          rightHandSide_ = loopEdgeConstraint_->configProjector ()->
            rightHandSideFromConfig (config);
          core::NumericalConstraints_t num =
            constraints->configProjector ()->solver().numericalConstraints();

          constraints::solver::BySubstitution solver
            ( constraints->configProjector ()-> solver ());

          for (std::size_t i=0 ; i<num.size() ; i++) {
            constraints::ImplicitPtr_t function = num[i];
            constraints::vectorOut_t rhs= function->nonConstRightHandSide();
            solver.getRightHandSide(num[i],rhs);

            rhsMap_.insert
              (std::pair<constraints::ImplicitPtr_t,constraints::vectorIn_t>
               (function,rhs));
          }
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
