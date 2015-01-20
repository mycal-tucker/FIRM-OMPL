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

/* Authors: Saurav Agarwal, Ali-akbar Agha-mohammadi */

#ifndef CARROT_CONTROLLER_METHOD_
#define CARROT_CONTROLLER_METHOD_


#include "../MotionModels/CarrotMotionModelMethod.h"
#include "../LinearSystem/CarrotLinearSystem.h"
#include "ompl/control/Control.h"

class CarrotControllerMethod
{

  public:
    typedef CarrotMotionModelMethod::SpaceType SpaceType;
    typedef CarrotMotionModelMethod::StateType StateType;
    typedef typename CarrotMotionModelMethod::ControlType   ControlType;
    typedef typename ObservationModelMethod::ObservationType ObservationType;
    typedef typename CarrotMotionModelMethod::MotionModelPointer MotionModelPointer;
    typedef typename arma::mat CostType;
    typedef typename arma::mat GainType;

    CarrotControllerMethod() {} //: MPBaseObject<MPTraits>() {}

    CarrotControllerMethod(ompl::base::State *goal,
        const std::vector<ompl::base::State*>& nominalXs,
        const std::vector<ompl::control::Control*>& nominalUs,
        const std::vector<CarrotLinearSystem>& linearSystems,
        const MotionModelPointer mm) :
        goal_(goal),
        nominalXs_(nominalXs),
        nominalUs_(nominalUs),
        linearSystems_(linearSystems),
        motionModel_(mm) {}

    ~CarrotControllerMethod() {}

    virtual ompl::control::Control* generateFeedbackControl(const ompl::base::State *state, const size_t& _t = 0) = 0;

    //void SetReachedFlag(bool _flag){m_reachedFlag = _flag;}

  protected:

    ompl::base::State *goal_;
    std::vector<ompl::base::State*> nominalXs_;
    std::vector<ompl::control::Control*> nominalUs_;
    std::vector< CarrotLinearSystem > linearSystems_;
    MotionModelPointer motionModel_;
    //bool m_reachedFlag;
};

#endif
