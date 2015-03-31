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

#ifndef FIRM_ROS_SPACE_INFORMATION_
#define FIRM_ROS_SPACE_INFORMATION_

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "ompl/control/SpaceInformation.h"
#include "../MotionModels/CarrotMotionModelMethod.h"
#include "../ObservationModels/ObservationModelMethod.h"


/**
The FIRMSpace information class is a derivative of the control::spaceinformation
that enables us to add FIRM specific information to the space. Specifically,
we need to use the motion/observation models time and again. We need not construct
them multiple times. Instead, we can make them members of this new class
*/

namespace firm
{
    class ROSSpaceInformation : public ompl::control::SpaceInformation
    {

        public:
            typedef typename ObservationModelMethod::ObservationType ObservationType;
            typedef CarrotMotionModelMethod::MotionModelPointer MotionModelPointer;
            typedef ObservationModelMethod::ObservationModelPointer ObservationModelPointer;
            typedef boost::shared_ptr<ROSSpaceInformation> SpaceInformationPtr;

            void stateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
            {
                ROS_INFO("I heard something");
                //ROS_INFO("I heard: [%s]", msg->pose.c_str());
                //std::cout << "I heard something" << std::endl;
            }

            ROSSpaceInformation(const ompl::base::StateSpacePtr &stateSpace,
                                const ompl::control::ControlSpacePtr &controlSpace) :
            ompl::control::SpaceInformation(stateSpace, controlSpace)
            {
                trueState_ = this->allocState();
                belief_    = this->allocState();
                int argc = 0;
                ros::init(argc,NULL,"state_listener"); // not command line, argc, argv not needed
                ros::NodeHandle n;
                ros::Subscriber state_sub = n.subscribe("/BQ02s/pose",10,&ROSSpaceInformation::stateCallback,this);
                //ros::Subscriber state_sub = n.subscribe("/BQ00/pose",10,stateCallback);

                state_sub_ = state_sub;

            }


            virtual ~ROSSpaceInformation(void)
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

            void setBelief(const ompl::base::State *state);

            void setTrueState(const ompl::base::State *state);

            ObservationModelPointer getObservationModel(void)
            {
                return observationModel_;
            }

            MotionModelPointer getMotionModel(void)
            {
                return motionModel_;
            }

            void getTrueState(ompl::base::State *state)
            {
                this->copyState(state, trueState_);
            }

            /** \brief Checks whether the true system state is in valid or not*/
            bool checkTrueStateValidity(void)
            {
                /*double x = trueState_->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                double y = trueState_->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
                double z = trueState_->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];

                if (x <= 0 || x >= 8) {
                    std::cout << "invalid x: " << x << std::endl;
                    return false;
                }
                if (y <=0 || y >= 12) {
                    std::cout << "invalid y: " << y << std::endl;
                    return false;
                }
                if (z <= 0 || z >= 3) {
                    std::cout << "invalid z: " << z << std::endl;
                    return false;
                }*/
                return true;
                //return this->isValid(trueState_);
            }

            void spin() {
                ros::spinOnce();
            }



            void applyControl(const ompl::control::Control *control, bool withNoise = true);

            ObservationType getObservation() ;


        protected:

            ObservationModelPointer observationModel_; // a model of the robot's sensor
            MotionModelPointer motionModel_; // a model of the robot's motion
            ompl::base::State *trueState_; // The real state of the robot
            ompl::base::State *belief_; // the estimated state of the robot
            ros::Subscriber state_sub_; //subscribes to quad pose



    };
}
#endif
