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

/* Authors: Saurav Agarwal */

#include "../../include/SpaceInformation/ROSSpaceInformation.h"
#include "../../include/Visualization/CarrotVisualizer.h"



void firm::ROSSpaceInformation::setBelief(const ompl::base::State *state)
{
    this->copyState(belief_, state);
    CarrotVisualizer::updateCurrentBelief(belief_);
}

void firm::ROSSpaceInformation::setTrueState(const ompl::base::State *state)
{
    std::cout << "preparing to spin..." << std::endl;


    std::cout << "spinning..." << std::endl;
    this->copyState(trueState_, state);
    CarrotVisualizer::updateTrueState(trueState_);
}

void firm::ROSSpaceInformation::applyControl(const ompl::control::Control *control, bool withNoise)
{
    typename CarrotMotionModelMethod::NoiseType noise;

    if(withNoise)
    {
        noise = motionModel_->generateNoise(trueState_, control);
    }
    else
    {
        noise = motionModel_->getZeroNoise();
    }

    motionModel_->Evolve(trueState_, control, noise, trueState_);

    CarrotVisualizer::updateTrueState(trueState_);
}

ObservationModelMethod::ObservationType firm::ROSSpaceInformation::getObservation()
{
    return observationModel_->getObservation(trueState_, true);
}


