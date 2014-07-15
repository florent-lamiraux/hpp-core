//
// Copyright (c) 2014 CNRS
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

#include <hpp/core/continuous-collision-checking.hh>

namespace hpp {
  namespace core {

    ContinuousCollisionCheckingPtr_t
    ContinuousCollisionChecking::create (const DevicePtr_t& robot)
    {
      ContinuousCollisionChecking* ptr =
	new ContinuousCollisionChecking (robot);
      ContinuousCollisionCheckingPtr_t shPtr (ptr);
      return shPtr;
    }

    bool ContinuousCollisionChecking::validate
    (const PathPtr_t& path, bool reverse, PathPtr_t& validPart)
    {
      return true;
    }

    ContinuousCollisionChecking::ContinuousCollisionChecking
    (const DevicePtr_t& robot) : robot_ (robot)
    {
    }

  } // namespace core
} // namespace hpp
