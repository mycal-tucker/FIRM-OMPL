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
    // we are running on hardware now
    {
        arma::colvec u = motionModel_->OMPL2ARMA(control);
        arma ::colvec x = belief_->as<CarrotBeliefSpace::StateType>()->getArmaData();
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
        //std::cout << "[CSpaceInfo] Published: " << carrot_x << " " << carrot_y << " " << carrot_z << std::endl;

        raven_rviz::Waypoint wayMsg;
        wayMsg.header.frame_id = quadName_.substr(1, 5); //TODO set to quadName but remove the bracketing slashes (5 for sim, 4 for real)
        wayMsg.goal_pose.position.x = carrot_x + belief_->as<CarrotBeliefSpace::StateType>()->getX();;
        wayMsg.goal_pose.position.y = carrot_y + belief_->as<CarrotBeliefSpace::StateType>()->getY();;
        wayMsg.goal_pose.position.z = carrot_z + belief_->as<CarrotBeliefSpace::StateType>()->getZ();;
        wayMsg.goal_pose.orientation.w = 1;
        wayMsg.goal_pose.orientation.x = 0;
        wayMsg.goal_pose.orientation.y = 0;
        wayMsg.goal_pose.orientation.z = 0;
        wayMsg.takeoff = true; //TODO investigate whether can leave always as true
        wayMsg.land = false;
        wayMsg.velocity = quadSpeed_;
        control_pub_waypoint_.publish(wayMsg);

        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    }
    CarrotVisualizer::updateTrueState(trueState_);
}

std::vector<double> firm::CarrotSpaceInformation::flyToWaypoint(double wayX, double wayY, double wayZ, bool withNoise, int commandNumber)
{
    arma ::colvec x = belief_->as<CarrotBeliefSpace::StateType>()->getArmaData();
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
    //std::cout << "[CSpaceInfo] Published: " << wayX << " " << wayY << " " << wayZ << std::endl;
    CarrotVisualizer::updateTrueState(trueState_);
    boost::this_thread::sleep(boost::posix_time::milliseconds(200));

    raven_rviz::Waypoint wayMsg;
    wayMsg.header.frame_id = quadName_.substr(1, 5); //TODO test if properly parsed quadName_ to remove brackets
    wayMsg.goal_pose.position.x = wayX;
    wayMsg.goal_pose.position.y = wayY;
    wayMsg.goal_pose.position.z = wayZ;
    wayMsg.goal_pose.orientation.w = 1;
    wayMsg.goal_pose.orientation.x = 0;
    wayMsg.goal_pose.orientation.y = 0;
    wayMsg.goal_pose.orientation.z = 0;
    wayMsg.takeoff = true; //TODO test if can reset takeoff values
    if (commandNumber == 1){ wayMsg.takeoff = true;}
    wayMsg.land = false;
    wayMsg.velocity = quadSpeed_;
    control_pub_waypoint_.publish(wayMsg);


    double xTrue = belief_->as<CarrotBeliefSpace::StateType>()->getX();
    double yTrue = belief_->as<CarrotBeliefSpace::StateType>()->getY();
    double zTrue = belief_->as<CarrotBeliefSpace::StateType>()->getZ();
    std::vector<double> location = {xTrue, yTrue, zTrue};
    return location;
}

std::vector<double> firm::CarrotSpaceInformation::flyAlongVector(double vecX, double vecY, double vecZ)
{
    std::cout<<"Do not call this. This is legacy code."<<std::endl;
    double maxSpeed = 0.5; //just guessing for now
    double controllerGain = 0.0002;

    double vecMagnitude = controllerGain*sqrt(vecX*vecX + vecY*vecY + vecZ*vecZ);

    //will be normalized if needed (just used for scope here)
    double wayX = controllerGain*vecX;
    double wayY = controllerGain*vecY;
    double wayZ = controllerGain*vecZ;

    if (vecMagnitude > maxSpeed)
    {
        //must normalize to go at maxSpeed in right direction
        double normalizationFactor = maxSpeed/vecMagnitude;
        wayX = normalizationFactor*vecX;
        wayY = normalizationFactor*vecY;
        wayZ = normalizationFactor*vecZ;
        //std::cout<<"speed capped"<<std::endl;
    }

    arma ::colvec x = trueState_->as<CarrotBeliefSpace::StateType>()->getArmaData();

    CarrotVisualizer::updateTrueState(trueState_);

    raven_rviz::Waypoint wayMsg;
    wayMsg.goal_pose.position.x = wayX;
    wayMsg.goal_pose.position.y = wayY;
    wayMsg.goal_pose.position.z = wayZ;
    wayMsg.goal_pose.orientation.w = 1;
    wayMsg.goal_pose.orientation.x = 0;
    wayMsg.goal_pose.orientation.y = 0;
    wayMsg.goal_pose.orientation.z = 0;
    wayMsg.takeoff = false;
    wayMsg.land = false;
    wayMsg.velocity = 0.1; //TODO refine this.

    control_pub_waypoint_.publish(wayMsg);


    double xTrue = trueState_->as<CarrotBeliefSpace::StateType>()->getX();
    double yTrue = trueState_->as<CarrotBeliefSpace::StateType>()->getY();
    double zTrue = trueState_->as<CarrotBeliefSpace::StateType>()->getZ();
    //std::cout<<"true state: x: "<<xTrue<<", y: "<<yTrue<<", z: "<<zTrue<<std::endl;
    std::vector<double> location = {xTrue, yTrue, zTrue};
    return location;
}

ObservationModelMethod::ObservationType firm::CarrotSpaceInformation::getObservation()
{
    return observationModel_->getObservation(trueState_, true);
}
