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

#ifndef FIRM_CARROT_SPACE_INFORMATION_
#define FIRM_CARROT_SPACE_INFORMATION_


#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "ompl/control/SpaceInformation.h"
#include "../MotionModels/CarrotMotionModelMethod.h"
#include "../ObservationModels/ObservationModelMethod.h"
#include "../../include/Spaces/CarrotBeliefSpace.h"



/**
The FIRMSpace information class is a derivative of the control::spaceinformation
that enables us to add FIRM specific information to the space. Specifically,
we need to use the motion/observation models time and again. We need not construct
them multiple times. Instead, we can make them members of this new class
*/

namespace firm
{
    class CarrotSpaceInformation : public ompl::control::SpaceInformation
    {

        public:
            typedef typename ObservationModelMethod::ObservationType ObservationType;
            typedef CarrotMotionModelMethod::MotionModelPointer MotionModelPointer;
            typedef ObservationModelMethod::ObservationModelPointer ObservationModelPointer;
            typedef boost::shared_ptr<CarrotSpaceInformation> SpaceInformationPtr;

            CarrotSpaceInformation(const ompl::base::StateSpacePtr &stateSpace,
                                const ompl::control::ControlSpacePtr &controlSpace) :
            ompl::control::SpaceInformation(stateSpace, controlSpace)
            {
                trueState_ = this->allocState();
                belief_    = this->allocState();
                isSimulation_ = true; // set to false when executing feedback path
                quadName_ = "/BQ02/";
                // set up ROS subscriber (state) and publisher (control)

            }

            void initializeSubscriber(void)
            {
                std::cout<<"Initialized subscriber"<<std::endl;
                int argc = 0;
                ros::init(argc,NULL,"state_listener"); // not command line, argc, argv not needed
                ros::NodeHandle n;
                ros::Subscriber state_sub = n.subscribe(quadName_+"pose",1000,&CarrotSpaceInformation::stateCallback,this);
                state_sub_ = state_sub;
            }

            void initializePublisher(void)
            {
                int argc = 0;
                ros::init(argc,NULL,"control_publisher");
                ros::NodeHandle n;
                //ros::Publisher control_pub = n.advertise<geometry_msgs::PoseStamped>(quadName_+"quad_waypoint",1000);
                ros::Publisher control_pub = n.advertise<geometry_msgs::PoseStamped>(quadName_+"quad_waypoint",10);
                control_pub_ = control_pub;
            }

            // state callback for subscriber
            void stateCallback(const geometry_msgs::PoseStamped& msg)
            {
                //std::cout << "[CSpaceInfo] Received state from ROS node" << std::endl;
                ompl::base::State* trueState = this->allocState();
                double x = msg.pose.position.x;
                double y = msg.pose.position.y;
                double z = msg.pose.position.z;
                trueState->as<CarrotBeliefSpace::StateType>()->setXYZ(x,y,z);
                this->copyState(trueState_,trueState);
                this->freeState(trueState);
                //std::cout << "State set to: " << trueState_->as<CarrotBeliefSpace::StateType>()->getArmaData() << std::endl;
                //ROS_INFO("State set to [%s]", msg.pose.c_str());
                //ROS_INFO("I heard something");
                //ROS_INFO("I heard: [%s]", msg->pose.c_str());
                //std::cout << "I heard something" << std::endl;
            }


            virtual ~CarrotSpaceInformation(void)
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

                return this->isValid(trueState_);
            }

            void applyControl(const ompl::control::Control *control, bool withNoise = true);

            std::vector<double> flyToWaypoint(double wayX, double wayY, double wayZ, bool withNoise = true);

            std::vector<double> flyAlongVector(double vecX, double vecY, double vecZ);

            std::vector<double> getQuadLocation(void)
            {
                double x = trueState_->as<CarrotBeliefSpace::StateType>()->getX();
                double y = trueState_->as<CarrotBeliefSpace::StateType>()->getY();
                double z = trueState_->as<CarrotBeliefSpace::StateType>()->getZ();
                std::vector<double> location = {x, y, z};
                return location;
            }

            ObservationType getObservation() ;

            void setSimulation(bool isSimulation)
            {
                isSimulation_ = isSimulation;
            }

            bool isSimulation( void )
            {
                return isSimulation_;
            }


        protected:

            ObservationModelPointer observationModel_; // a model of the robot's sensor
            MotionModelPointer motionModel_; // a model of the robot's motion
            ompl::base::State *trueState_; // The real state of the robot
            ompl::base::State *belief_; // the estimated state of the robot
            bool isSimulation_; // to switch between sim and ROS hardware
            ros::Subscriber state_sub_; // subscriber for quad pose
            ros::Publisher control_pub_; // publisher for quad control
            std::string quadName_; // name of quad we're pubbing topics to e.g. 'BQ04'




    };
}
#endif
