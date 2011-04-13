/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef OMPL_BASE_MANIFOLDS_SE2_STATE_MANIFOLD_
#define OMPL_BASE_MANIFOLDS_SE2_STATE_MANIFOLD_

#include "ompl/base/StateManifold.h"
#include "ompl/base/manifolds/RealVectorStateManifold.h"
#include "ompl/base/manifolds/SO2StateManifold.h"

#ifndef PI
#define PI 3.1415926
#endif

#define my_min(x,y) (x<y?x:y)
#define my_max(x,y) (y>x?y:x)

namespace ompl
{
    namespace base
    {

        /** \brief A manifold representing SE(2) + 1 additional Z component */
        class SE2plusStateManifold : public CompoundStateManifold
        {
        public:

            /** \brief A state in SE(2): (x, y, z, yaw) */
            class StateType : public CompoundStateManifold::StateType
            {
            public:
                StateType(void) : CompoundStateManifold::StateType()
                {
                }

                /** \brief Get the X component of the state */
                double getX(void) const
                {
                    return as<RealVectorStateManifold::StateType>(0)->values[0];
                }

                /** \brief Get the Y component of the state */
                double getY(void) const
                {
                    return as<RealVectorStateManifold::StateType>(0)->values[1];
                }

                /** \brief Get the Z component of the state */
                double getZ(void) const
                {
                    return as<RealVectorStateManifold::StateType>(0)->values[2];
                }

                /** \brief Get the yaw component of the state. This is
                    the rotation in plane, with respect to the Z
                    axis. */
                double getYaw(void) const
                {
                    return as<SO2StateManifold::StateType>(1)->value;
                }

                /** \brief Set the X component of the state */
                void setX(double x)
                {
                    as<RealVectorStateManifold::StateType>(0)->values[0] = x;
                }

                /** \brief Set the Y component of the state */
                void setY(double y)
                {
                    as<RealVectorStateManifold::StateType>(0)->values[1] = y;
                }

                /** \brief Set the Y component of the state */
                void setZ(double z)
                {
                    as<RealVectorStateManifold::StateType>(0)->values[2] = z;
                }

                /** \brief Set the X and Y components of the state */
                void setXYZ(double x, double y, double z)
                {
                    setX(x);
                    setY(y);
		    setZ(z);
                }

                /** \brief Set the yaw component of the state. This is
                    the rotation in plane, with respect to the Z
                    axis. */
                void setYaw(double yaw)
                {
                    as<SO2StateManifold::StateType>(1)->value = yaw;
                }

            };


            SE2plusStateManifold(void) : CompoundStateManifold()
            {
                name_ = "SE2plus" + name_;
                addSubManifold(StateManifoldPtr(new RealVectorStateManifold(3)), 1.0);
                addSubManifold(StateManifoldPtr(new SO2StateManifold()), 0.5);
                lock();
            }

            virtual ~SE2plusStateManifold(void)
            {
            }

            /** \copydoc RealVectorStateManifold::setBounds() */
            void setBounds(const RealVectorBounds &bounds)
            {
                as<RealVectorStateManifold>(0)->setBounds(bounds);
            }

            /** \copydoc RealVectorStateManifold::getBounds() */
            const RealVectorBounds& getBounds(void) const
            {
                return as<RealVectorStateManifold>(0)->getBounds();
            }
            
            virtual double distance (const State *state1, const State *state2) const
	    {	 
		double x1 = state1->as<SE2plusStateManifold::StateType>()->getX();
		double y1 = state1->as<SE2plusStateManifold::StateType>()->getY();
// 		double z1 = state1->as<SE2plusStateManifold::StateType>()->getZ();
		double yaw1 = state1->as<SE2plusStateManifold::StateType>()->getYaw();
		
		double x2 = state2->as<SE2plusStateManifold::StateType>()->getX();
		double y2 = state2->as<SE2plusStateManifold::StateType>()->getY();
// 		double z2 = state2->as<SE2plusStateManifold::StateType>()->getZ();
		double yaw2 = state2->as<SE2plusStateManifold::StateType>()->getYaw();
		
		double dist = sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
// 		double zPenalty = abs(z2 - z1);
// 		double crabPenalty = (abs((x2-x1)*sin(yaw1) - (y2-y1)*cos(yaw1)) + abs((x2-x1)*sin(yaw2) - (y2-y1)*cos(yaw2)))/2.0;
// 		double crabPenalty = ( (x2-x1)*cos(yaw1) + (y2-y1)*sin(yaw1) < 0.0 
// 				    || (x2-x1)*cos(yaw2) + (y2-y1)*sin(yaw2) < 0.0) ? 1000.0 : 0.0;
		double backPenalty = (my_max(0,-((x2-x1)*cos(yaw1) + (y2-y1)*sin(yaw1))) + my_max(0,-((x2-x1)*cos(yaw2) + (y2-y1)*sin(yaw2))))/2.0;
// 		double backPenalty = ( (x2-x1)*cos(yaw1) + (y2-y1)*sin(yaw1) > 0.0 
// 				    || (x2-x1)*cos(yaw2) + (y2-y1)*sin(yaw2) > 0.0) ? 1000.0 : 0.0;
		double turnPenalty = my_min(my_min(abs(yaw2 - yaw1), abs(yaw2 - yaw1 + 2*PI)), abs(yaw2 - yaw1 - 2*PI));
		
		return ( dist + backPenalty + turnPenalty);
	    }

            virtual State* allocState(void) const;
            virtual void freeState(State *state) const;

            virtual void registerProjections(void);

        };
    }
}

#endif
