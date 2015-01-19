/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Texas A&M University
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
*   * Neither the name of the Texas A&M University nor the names of its
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
/* Author: Ali-akbar Agha-mohammadi, Saurav Agarwal */

#include "../../include/MotionModels/CarrotStatePropagator.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/util/Exception.h"
using namespace ompl;

CarrotStatePropagator::CarrotStatePropagator(const firm::SpaceInformation::SpaceInformationPtr &si) : StatePropagator(si), siF_(si)
{
    // The path to this setup file must not be hardcopied, need a better way to do this
    motionModel_ = siF_->getMotionModel();
}

void CarrotStatePropagator::propagate(const base::State *state, const control::Control* control, const double duration, base::State *result) const
{

    // set the time step
    if(duration != 0.1)
    {
        std::cout<<"In state propagator, duration is: (Press Enter to Continue)"<<duration<<std::endl;
        std::cin.get();
    }

    //motionModel_->setTimeStep(duration);

    typedef CarrotBeliefSpace::StateType StateType;

    ompl::base::State *to = si_->allocState();

    // use the motionmodel to apply the controls
    motionModel_->Evolve(state, control, motionModel_->getZeroNoise(), to);

    si_->copyState(result,to);

}

bool CarrotStatePropagator::canPropagateBackward(void) const
{
    return false;
}

