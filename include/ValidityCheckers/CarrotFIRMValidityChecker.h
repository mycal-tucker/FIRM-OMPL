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
//#include <omplapp/geometry/detail/FCLStateValidityChecker.h>
/*
Used to check if a state is in collission or not moreover,
in FIRM we need to know if a state is observable or not
before adding it to the graph.
*/

#ifndef CARROT_FIRM_VALIDITY
#define CARROT_FIRM_VALIDITY

class CarrotFIRMValidityChecker : public ompl::base::StateValidityChecker
{
  public:
    typedef ObservationModelMethod::ObservationModelPointer ObservationModelPointer;
    typedef CarrotBeliefSpace::StateType StateType;

    CarrotFIRMValidityChecker(const firm::CarrotSpaceInformation::SpaceInformationPtr &si) : ompl::base::StateValidityChecker(si), siF_(si)
    {
    }

    virtual bool isValid(const ompl::base::State *state) const
    {
        double x = state->as<StateType>()->getX();
        double y = state->as<StateType>()->getY();
        double z = state->as<StateType>()->getZ();

        //must be within test area:
        //x >=-1.9, x <=1.0
        //y >=-6.8, y <=0.9
        //z >= 0, z <=2

        bool withinTestArea = isInsideBox(state, -1.9, 1.0, -6.8, 0.9) && (z >=0 && z <= 2);

          /*if (x <= -1.9 || x >= 1.0) {
              //std::cout << "invalid x: " << x << std::endl;
              return false;
          }
          if (y <= -6.8 || y >= 0.9) {
              //std::cout << "invalid y: " << y << std::endl;
              return false;
          }
          if (z <= 1 || z >= 2) {
              //std::cout << "invalid z: " << z << std::endl;
              return false;
          }*/

          //obs1:
          //x between -1.37 and -0.405
          //y between -1.46 and -2.40

          bool outsideObstacle1 = !isInsideBox(state, -1.13, -0.439, -1.81, -0.95);

          //obs2:
          //x between -0.095 and 0.90
          //y between -0.85 and -2.98

          bool outsideObstacle2 = !isInsideBox(state, 0.215, 0.973, -3.05, -0.85);

          return withinTestArea && outsideObstacle1 && outsideObstacle2;

      //ompl::base::SE3StateSpace::StateType *pos = state->as<ompl::base::SE3StateSpace::StateType>();
/*      ompl::base::StateSpacePtr si(new ompl::base::SE3StateSpace());
      ompl::base::State* se3_state = si->allocState();
      ompl::base::SO3StateSpace::StateType &so3 = se3_state->as<ompl::base::SE3StateSpace::StateType>()->rotation();
      so3.x = 1; so3.y = 0; so3.z = 0; so3.w = 0;*/
 //     if ( ompl::app::FCLStateValidityChecker::isValid(se3_state) )
  //    {
        //si->freeState(se3_state);
        //return siF_->getObservationModel()->isStateObservable(state);
   //   }
    // si->freeState(se3_state);
     // return false;
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
        firm::CarrotSpaceInformation::SpaceInformationPtr siF_;
};

#endif
