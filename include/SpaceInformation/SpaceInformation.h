/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Texas A&M University
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

/* Authors: Saurav Agarwal, Ali-akbar Agha-mohammadi */

#ifndef FIRM_SPACE_INFORMATION_
#define FIRM_SPACE_INFORMATION_

#include "ompl/control/SpaceInformation.h"
#include "../MotionModels/MotionModelMethod.h"
#include "../ObservationModels/ObservationModelMethod.h"

/**
The FIRMSpace information class is a derivative of the control::spaceinformation
that enables us to add FIRM specific information to the space. Specifically,
we need to use the motion/observation models time and again. We need not construct
them multiple times. Instead, we can make them members of this new class
*/

namespace firm
{
    class SpaceInformation : public ompl::control::SpaceInformation
    {

        public:
            typedef MotionModelMethod::MotionModelPointer MotionModelPointer;
            typedef ObservationModelMethod::ObservationModelPointer ObservationModelPointer;
            typedef boost::shared_ptr<SpaceInformation> SpaceInformationPtr;

            SpaceInformation(const ompl::base::StateSpacePtr &stateSpace,
                                const ompl::control::ControlSpacePtr &controlSpace) :
            ompl::control::SpaceInformation(stateSpace, controlSpace)
            {
            }


            virtual ~SpaceInformation(void)
            {
            }

            void setObservationModel(ObservationModelPointer om)
            {
                observationModel_ = om;
            }

            void setMotionModel(MotionModelPointer mm)
            {
                motionModel_ = mm;
            }

            ObservationModelPointer getObservationModel(void)
            {
                return observationModel_;
            }

            MotionModelPointer getMotionModel(void)
            {
                return motionModel_;
            }

        protected:

            ObservationModelPointer observationModel_;
            MotionModelPointer motionModel_;



    };
}
#endif
