//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux, Joseph Mirabel
//
//
// This file is part of hpp-manipulation.
// hpp-manipulation is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation. If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_MANIPULATION_FWD_HH
# define HPP_MANIPULATION_FWD_HH

# include <map>
# include <hpp/manipulation/config.hh>
# include <hpp/core/fwd.hh>

namespace hpp {
  namespace manipulation {
    HPP_PREDEF_CLASS (Device);
    typedef shared_ptr <Device> DevicePtr_t;
    typedef shared_ptr <const Device> DeviceConstPtr_t;
    typedef pinocchio::Joint Joint;
    typedef pinocchio::JointPtr_t JointPtr_t;
    typedef pinocchio::JointIndex JointIndex;
    typedef std::vector<JointIndex> JointIndices_t;
    typedef pinocchio::FrameIndex FrameIndex;
    typedef std::vector<pinocchio::FrameIndex> FrameIndices_t;
    typedef pinocchio::Configuration_t Configuration_t;
    typedef pinocchio::ConfigurationIn_t ConfigurationIn_t;
    typedef pinocchio::ConfigurationOut_t ConfigurationOut_t;
    typedef core::ConfigurationPtr_t ConfigurationPtr_t;
    typedef pinocchio::GripperPtr_t GripperPtr_t;
    typedef pinocchio::LiegroupElement LiegroupElement;
    typedef pinocchio::LiegroupSpace LiegroupSpace;
    typedef pinocchio::LiegroupSpacePtr_t LiegroupSpacePtr_t;
    HPP_PREDEF_CLASS (AxialHandle);
    typedef shared_ptr <AxialHandle> AxialHandlePtr_t;
    HPP_PREDEF_CLASS (Handle);
    typedef shared_ptr <Handle> HandlePtr_t;
    HPP_PREDEF_CLASS (Object);
    typedef shared_ptr <Object> ObjectPtr_t;
    typedef shared_ptr <const Object> ObjectConstPtr_t;
    HPP_PREDEF_CLASS (ProblemSolver);
    typedef ProblemSolver* ProblemSolverPtr_t;
    HPP_PREDEF_CLASS (Problem);
    typedef shared_ptr <Problem> ProblemPtr_t;
    typedef shared_ptr <const Problem> ProblemConstPtr_t;
    HPP_PREDEF_CLASS (Roadmap);
    typedef shared_ptr <Roadmap> RoadmapPtr_t;
    HPP_PREDEF_CLASS (RoadmapNode);
    typedef RoadmapNode* RoadmapNodePtr_t;
    typedef std::vector<RoadmapNodePtr_t> RoadmapNodes_t;
    HPP_PREDEF_CLASS (ConnectedComponent);
    typedef shared_ptr<ConnectedComponent> ConnectedComponentPtr_t; 
    HPP_PREDEF_CLASS (LeafConnectedComp);
    typedef shared_ptr<LeafConnectedComp> LeafConnectedCompPtr_t;
    typedef shared_ptr<const LeafConnectedComp>
    LeafConnectedCompConstPtr_t;
    typedef std::set<LeafConnectedCompPtr_t> LeafConnectedComps_t;
    HPP_PREDEF_CLASS (WeighedLeafConnectedComp);
    typedef shared_ptr<WeighedLeafConnectedComp> WeighedLeafConnectedCompPtr_t;
    HPP_PREDEF_CLASS (WeighedDistance);
    typedef shared_ptr<WeighedDistance> WeighedDistancePtr_t;
    typedef constraints::RelativeOrientation RelativeOrientation;
    typedef constraints::RelativePosition RelativePosition;
    typedef constraints::RelativeOrientationPtr_t RelativeOrientationPtr_t;
    typedef constraints::RelativePositionPtr_t RelativePositionPtr_t;
    typedef constraints::RelativeTransformation RelativeTransformation;
    typedef constraints::RelativeTransformationR3xSO3
    RelativeTransformationR3xSO3;
    typedef constraints::RelativeTransformationPtr_t
    RelativeTransformationPtr_t;
    typedef core::value_type value_type;
    typedef core::size_type size_type;
    typedef core::Transform3f Transform3f;
    typedef core::vector_t vector_t;
    typedef core::vectorIn_t vectorIn_t;
    typedef core::vectorOut_t vectorOut_t;
    HPP_PREDEF_CLASS (ManipulationPlanner);
    typedef shared_ptr < ManipulationPlanner > ManipulationPlannerPtr_t;
    namespace pathPlanner {
      HPP_PREDEF_CLASS (RMRStar);
      typedef shared_ptr < RMRStar > RMRStarPtr_t;
      HPP_PREDEF_CLASS (StatesPathFinder);
      typedef shared_ptr < StatesPathFinder > StatesPathFinderPtr_t;
      HPP_PREDEF_CLASS (InStatePath);
      typedef shared_ptr < InStatePath > InStatePathPtr_t;
      HPP_PREDEF_CLASS (StateShooter);
      typedef shared_ptr < StateShooter > StateShooterPtr_t;
    } // namespace pathPlanner
    HPP_PREDEF_CLASS (GraphPathValidation);
    typedef shared_ptr < GraphPathValidation > GraphPathValidationPtr_t;
    HPP_PREDEF_CLASS (SteeringMethod);
    typedef shared_ptr < SteeringMethod > SteeringMethodPtr_t;
    typedef core::PathOptimizer PathOptimizer;
    typedef core::PathOptimizerPtr_t PathOptimizerPtr_t;
    HPP_PREDEF_CLASS (GraphOptimizer);
    typedef shared_ptr < GraphOptimizer > GraphOptimizerPtr_t;
    HPP_PREDEF_CLASS (GraphNodeOptimizer);
    typedef shared_ptr < GraphNodeOptimizer > GraphNodeOptimizerPtr_t;
    typedef core::PathProjectorPtr_t PathProjectorPtr_t;

    typedef std::vector <pinocchio::DevicePtr_t> Devices_t;
    typedef std::vector <ObjectPtr_t> Objects_t;
    typedef core::Constraint Constraint;
    typedef core::ConstraintPtr_t ConstraintPtr_t;
    typedef constraints::Explicit Explicit;
    typedef constraints::ExplicitPtr_t ExplicitPtr_t;
    typedef constraints::ImplicitPtr_t ImplicitPtr_t;
    typedef constraints::LockedJoint LockedJoint;
    typedef constraints::LockedJointPtr_t LockedJointPtr_t;
    typedef hpp::core::ComparisonTypes_t ComparisonTypes_t;
    typedef core::ConfigProjector ConfigProjector;
    typedef core::ConfigProjectorPtr_t ConfigProjectorPtr_t;
    HPP_PREDEF_CLASS (ConstraintSet);
    typedef shared_ptr <ConstraintSet> ConstraintSetPtr_t;
    typedef core::DifferentiableFunctionPtr_t DifferentiableFunctionPtr_t;
    typedef core::ConfigurationShooter ConfigurationShooter;
    typedef core::ConfigurationShooterPtr_t ConfigurationShooterPtr_t;
    typedef core::ValidationReport ValidationReport;
    typedef core::NumericalConstraints_t NumericalConstraints_t;
    typedef core::PathValidationPtr_t PathValidationPtr_t;
    typedef core::PathValidationReportPtr_t PathValidationReportPtr_t;
    typedef core::matrix_t matrix_t;
    typedef core::matrixIn_t matrixIn_t;
    typedef core::matrixOut_t matrixOut_t;
    typedef core::size_type size_type;
    typedef core::value_type value_type;
    typedef core::vector3_t vector3_t;
    typedef core::matrix3_t matrix3_t;

    typedef core::Shape_t Shape_t;
    typedef core::JointAndShape_t JointAndShape_t;
    typedef core::JointAndShapes_t JointAndShapes_t;

    typedef std::list <std::string> StringList_t;

    namespace pathOptimization {
      HPP_PREDEF_CLASS (SmallSteps);
      typedef shared_ptr < SmallSteps > SmallStepsPtr_t;
      HPP_PREDEF_CLASS (Keypoints);
      typedef shared_ptr < Keypoints > KeypointsPtr_t;
    } // namespace pathOptimization

    namespace problemTarget {
      HPP_PREDEF_CLASS (State);
      typedef shared_ptr < State > StatePtr_t;
    } // namespace problemTarget
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_FWD_HH
