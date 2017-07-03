//
// Copyright (c) 2015 CNRS
// Author: Mylene Campana, Joseph Mirabel
//
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

#ifndef HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_COLLISION_CONSTRAINTS_HH
# define HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_COLLISION_CONSTRAINTS_HH

#include <hpp/util/exception-factory.hh>

# include <hpp/fcl/collision.h>
# include <hpp/fcl/distance.h>

# include <hpp/pinocchio/configuration.hh>
# include <pinocchio/multibody/liegroup/liegroup.hpp>

# include <hpp/constraints/generic-transformation.hh>

namespace hpp {
  namespace core {
    namespace pathOptimization {
      HPP_PREDEF_CLASS (CollisionFunction);
      typedef boost::shared_ptr <CollisionFunction> CollisionFunctionPtr_t;
      typedef pinocchio::Transform3f Transform3f;
      namespace eigen {
	typedef Eigen::Matrix <value_type, 3, 1> vector3_t;
      } // namespace eigen


      class CollisionFunction : public DifferentiableFunction
      {
      public:
       virtual ~CollisionFunction () {}
       template <typename SplinePtr_t>
       static CollisionFunctionPtr_t create
       (const DevicePtr_t& robot, const SplinePtr_t& freeSpline,
        const SplinePtr_t& collSpline,
        const CollisionPathValidationReportPtr_t& report)
       {
         const value_type& tColl = report->parameter;
         bool success;
         Configuration_t qColl = (*collSpline) (tColl, success);
         assert(success);

         HPP_STATIC_CAST_REF_CHECK (CollisionValidationReport,
             *(report->configurationReport));
         CollisionObjectConstPtr_t object1 =
           HPP_STATIC_PTR_CAST (CollisionValidationReport,
               report->configurationReport)->object1;
         CollisionObjectConstPtr_t object2 =
           HPP_STATIC_PTR_CAST (CollisionValidationReport,
               report->configurationReport)->object2;

         hppDout (info, "Collision at ratio (in [0,1]) = " << tColl / collSpline->length());
         hppDout (info, "obj1 = " << object1->name()
             << " and obj2 = " << object2->name());
         hppDout (info, "qColl = " << qColl.transpose());

         // Backtrack collision in previous path (x0) to create constraint
         value_type tFree = tColl * freeSpline->length () / collSpline->length();

         Configuration_t qFree = (*freeSpline) (tFree, success);
         assert(success);
         hppDout (info, "qFree = " << qFree.transpose());

         return create (robot, qFree, qColl, object1, object2);
       }
       static CollisionFunctionPtr_t create
       (const DevicePtr_t& robot, const Configuration_t& qFree,
	const Configuration_t& qColl, const CollisionObjectConstPtr_t& object1,
	const CollisionObjectConstPtr_t& object2)
       {
         CollisionFunction* ptr = new CollisionFunction
           (robot, qFree, qColl, object1, object2);
         CollisionFunctionPtr_t shPtr (ptr);
         return shPtr;
       }
      protected:
       CollisionFunction (const DevicePtr_t& robot,
                            const Configuration_t& qFree,
                            const Configuration_t& qColl,
			    const CollisionObjectConstPtr_t& object1,
			    const CollisionObjectConstPtr_t& object2)
         : DifferentiableFunction (robot->configSize (), robot->numberDof (),
                                   1, ""), robot_ (robot), qFree_ (qFree),
           J_ (), difference_ ()
	   
       {
	 difference_.resize (robot->numberDof ());
	 // Compute contact point in configuration qColl
	 robot_->currentConfiguration (qColl);
	 robot_->computeForwardKinematics ();
         robot_->updateGeometryPlacements ();
	 fcl::CollisionResult result;
         fcl::CollisionRequest collisionRequest (1, true, false, 1, false, true, fcl::GST_INDEP);
         fcl::collide (object1->fcl (), object2->fcl (), collisionRequest, result);
         if (result.numContacts() != 1) {
           result.clear();
           collisionRequest.enable_contact = false;
           fcl::collide (object1->fcl (), object2->fcl (), collisionRequest, result);
           if (result.isCollision()) {
             hppDout (error, "FCL does not returns the same result when asking or not for the contact points.");
           }

           HPP_THROW(std::invalid_argument,
               "Object " << object1->name() << " and " << object2->name()
               << " are not in collision in configuration\n"
               << qColl.transpose().format(IPythonFormat)
               << "\nqFree is\n" << qFree.transpose().format(IPythonFormat));
         }
	 assert (result.numContacts () == 1);
	 const vector3_t& contactPoint (result.getContact (0).pos);
	 hppDout (info, "contact point = " << contactPoint.transpose());

	 JointConstPtr_t joint1 = object1->joint ();
         // FIXME this copy can probably be avoided.
         Transform3f M1 (joint1->currentTransformation ());
         if (object2->joint ()) { // object2 = body part
           JointConstPtr_t joint2 = object2->joint ();
           Transform3f M2 (joint2->currentTransformation ());
           // Position of contact point in each object local frame
           vector3_t x1_J1 = M1.actInv (contactPoint);
           vector3_t x2_J2 = M2.actInv (contactPoint);
           // Compute contact points in configuration qFree
           robot_->currentConfiguration (qFree);
           robot_->computeForwardKinematics ();
           M2 = joint2->currentTransformation ();
           M1 = joint1->currentTransformation ();
           // Position of x2 in local frame of joint1
           vector3_t x2_J1 (M1.actInv (M2.act (x2_J2)));
           hppDout (info, "x1 in J1 = " << x1_J1.transpose());
           hppDout (info, "x2 in J1 = " << x2_J1.transpose());
           eigen::vector3_t u=x2_J1 - x1_J1;
           matrix3_t rot;
           rot.setIdentity();
           DifferentiableFunctionPtr_t f = constraints::RelativePosition::create
             ("", robot_, joint1, joint2, Transform3f(rot,x1_J1), Transform3f(rot,x2_J2));
           matrix_t Jpos (f->outputSize (), f->inputDerivativeSize ());
           f->jacobian (Jpos, qFree);
           J_ = u.transpose () * Jpos;
           assert (J_.rows () == 1);
         } else{ // object2 = fixed obstacle and has no joint
           vector3_t x1_J1 (M1.actInv(contactPoint));
           vector3_t x2_J2 (contactPoint);
           // Compute contact points in configuration qFree
           robot_->currentConfiguration (qFree);
           robot_->computeForwardKinematics ();
           Transform3f M1 (joint1->currentTransformation ());
           // position of x1 in global frame
           vector3_t x1_J2 (M1.act (x1_J1));
           hppDout (info, "x1 in J2 = " << x1_J2.transpose());
           eigen::vector3_t u=x1_J2 - x2_J2;
           matrix3_t rot;
           rot.setIdentity();
           DifferentiableFunctionPtr_t f = constraints::Position::create
             ("", robot_, joint1, Transform3f(rot,x1_J1), Transform3f(rot,x2_J2));
           matrix_t Jpos (f->outputSize (), f->inputDerivativeSize ());
           f->jacobian (Jpos, qFree);
           J_ = u.transpose () * Jpos;
           assert (J_.rows () == 1);
         }
       }

       virtual void impl_compute (vectorOut_t result, vectorIn_t argument)
         const
       {
         pinocchio::difference<se3::LieGroupTpl> (robot_, argument, qFree_, difference_);
         result = J_ * difference_;
       }
       virtual void impl_jacobian (matrixOut_t jacobian, vectorIn_t) const
       {
         jacobian = J_;
       }
      private:
	DevicePtr_t robot_;
	Configuration_t qFree_;
	matrix_t J_;
	mutable vector_t difference_;
      }; // class CollisionFunction
    } // namespace pathOptimization
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_COLLISION_CONSTRAINTS_HH
