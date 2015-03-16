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

#include "../../include/SpaceInformation/CarrotSpaceInformation.h"
#include "../../include/Visualization/CarrotVisualizer.h"
#include "../../include/MotionModels/CarrotMotionModelMethod.h"


void firm::CarrotSpaceInformation::setBelief(const ompl::base::State *state)
{
    this->copyState(belief_, state);
    CarrotVisualizer::updateCurrentBelief(belief_);
}

void firm::CarrotSpaceInformation::setTrueState(const ompl::base::State *state)
{
    this->copyState(trueState_, state);
    CarrotVisualizer::updateTrueState(trueState_);
}

void firm::CarrotSpaceInformation::applyControl(const ompl::control::Control *control, bool withNoise)
{
    if (isSimulation_) {

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

    }
    else
    {
        arma::colvec u = motionModel_->OMPL2ARMA(control);
        arma ::colvec x = trueState_->as<CarrotBeliefSpace::StateType>()->getArmaData();
        geometry_msgs::PoseStamped msg;
        double carrot_x = u[0];
        double carrot_y = u[1];
        double carrot_z = u[2];
        msg.pose.position.x = carrot_x;
        msg.pose.position.y = carrot_y;
        msg.pose.position.z = carrot_z;
        // no rotation
        msg.pose.orientation.w = 1;
        msg.pose.orientation.x = 0;
        msg.pose.orientation.y = 0;
        msg.pose.orientation.z = 0;
        /*msg.takeoff =false;
        msg.land = false;
        msg.velocity = 1;*/
        control_pub_.publish(msg);
        std::cout << "[CSpaceInfo] Published: " << carrot_x << " " << carrot_y << " " << carrot_z << std::endl;
        //boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    }
    CarrotVisualizer::updateTrueState(trueState_);

}

void firm::CarrotSpaceInformation::flyToWaypoint(double wayX, double wayY, double wayZ, bool withNoise)
{
    //arma::colvec u = motionModel_->OMPL2ARMA(control);
    arma ::colvec x = trueState_->as<CarrotBeliefSpace::StateType>()->getArmaData();
    geometry_msgs::PoseStamped msg;

    msg.pose.position.x = wayX;
    msg.pose.position.y = wayY;
    msg.pose.position.z = wayZ;

    // no rotation
    msg.pose.orientation.w = 1;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;
    /*msg.takeoff =false;
    msg.land = false;
    msg.velocity = 1;*/
    control_pub_.publish(msg);
    std::cout << "[CSpaceInfo] Published: " << wayX << " " << wayY << " " << wayZ << std::endl;
    //boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    CarrotVisualizer::updateTrueState(trueState_);
}

ObservationModelMethod::ObservationType firm::CarrotSpaceInformation::getObservation()
{
    return observationModel_->getObservation(trueState_, true);
}


