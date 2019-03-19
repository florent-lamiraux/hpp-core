//
// Copyright (c) 2019 CNRS
// Authors: Florent Lamiraux
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

#define BOOST_TEST_MODULE weighed_distance

#include <boost/test/included/unit_test.hpp>

#include <hpp/pinocchio/simple-device.hh>
#include <hpp/core/weighed-distance.hh>

using namespace hpp::core;
using hpp::pinocchio::unittest::makeDevice;
using hpp::pinocchio::unittest::CarLike;

BOOST_AUTO_TEST_CASE (carlike)
{
  DevicePtr_t robot = makeDevice (CarLike);
  Configuration_t q1 (robot->configSize ());
  Configuration_t q2 (robot->configSize ());
  q1 [0] = 3; q1 [1] = 10; q1 [2] = 0; q1 [3] = 1;
  q2 [0] = 26.1; q2 [1] = 9.90; q2 [2] = 0.94974146132194259;
  q2 [3] = -0.31303539200234359;
  WeighedDistancePtr_t d (WeighedDistance::create (robot));
  BOOST_CHECK ((*d)(q1,q2) < std::numeric_limits <value_type>::infinity ());
}
