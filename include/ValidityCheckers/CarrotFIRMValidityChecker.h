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


#include "../ObservationModels/ObservationModelMethod.h"
/*
Used to check if a state is in collission or not moreover,
in FIRM we need to know if a state is observable or not
before adding it to the graph.
*/

class CarrotFIRMValidityChecker : public ompl::base::StateValidityChecker
{
  public:
    typedef ObservationModelMethod::ObservationModelPointer ObservationModelPointer;
    typedef CarrotBeliefSpace::StateType StateType;

    CarrotFIRMValidityChecker(const firm::SpaceInformation::SpaceInformationPtr &si) : ompl::base::StateValidityChecker(si), siF_(si)
    {
    }

    virtual bool isValid(const ompl::base::State *state) const
    {
      // states within a box are invalid
      double x_l =  2.0;
      double x_r =  14.5;
      double y_b =  2.0;
      double y_t =  5;

      arma::colvec pos = state->as<StateType>()->getArmaData();

      if(pos[0]>= 0 && pos[0] <=17)
      {
        if(pos[1]>=0 && pos[1]<=7)
        {
            return !isInsideBox(state,x_l,x_r, y_b, y_t); // if inside box then not valid
        }
      }

      //return true;//siF_->getObservationModel()->isStateObservable(state);
      return false;
    }

    /** \brief Checks if the state is within a bounding box */
    bool isInsideBox(const ompl::base::State *state, double xl, double xr, double yb, double yt) const
    {
        arma::colvec pos = state->as<StateType>()->getArmaData();
        double eps = 0.20;
        if(pos[0] >= xl-eps && pos[0] <= xr+eps )
            {
            if(pos[1] >= yb-eps && pos[1] <= yt+eps)
            {
                return true; // inside box
            }
        }
        return false; // outside box
    }
    protected:
        firm::SpaceInformation::SpaceInformationPtr siF_;
};
