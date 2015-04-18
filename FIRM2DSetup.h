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

/* Author: Saurav Agarwal */

#ifndef FIRM_2D_SETUP_H
#define FIRM_2D_SETUP_H

#include <omplapp/geometry/RigidBodyGeometry.h>
#include "include/Planner/FIRM.h"
#include "FIRMOMPL.h"
#include <tinyxml.h>

/** \brief Wrapper for ompl::app::RigidBodyPlanning that plans for rigid bodies in SE2BeliefSpace using FIRM */
class FIRM2DSetup : public ompl::app::RigidBodyGeometry
{

    typedef SE2BeliefSpace::StateType StateType;

public:

    FIRM2DSetup() : ompl::app::RigidBodyGeometry(ompl::app::Motion_2D),
    ss_(ompl::base::StateSpacePtr(new SE2BeliefSpace()))
    {
        // set static variables
        RHCICreate::setControlQueueSize(10);
        RHCICreate::setTurnOnlyDistance(0.05);
        Controller<RHCICreate, ExtendedKF>::setNodeReachedAngle(30); // degrees
        Controller<RHCICreate, ExtendedKF>::setNodeReachedDistance(0.20);// meters
        Controller<RHCICreate, ExtendedKF>::setMaxTries(200);
        Controller<RHCICreate, ExtendedKF>::setMaxTrajectoryDeviation(4.0); // meters

        // setting the mean and norm weights (used in reachability check)
        StateType::covNormWeight_  =  1.0;
        StateType::meanNormWeight_ =  2.0;
        StateType::reachDist_ =  0.01;

        // set the state component norm weights
        arma::colvec normWeights(3);
        normWeights(0) = 2.0/3.0;
        normWeights(1) = 2.0/3.0;
        normWeights(2) = 1.0/3.0;
        StateType::normWeights_ = normWeights;

        // The bounds should be inferred from the geometry files,
        // there is a function in Apputils to do this, so use that.
        // set the bounds for the R^3 part of SE(3)
        ompl::base::RealVectorBounds bounds(2);
        // set X bound
        bounds.setLow(0,0.2);
        bounds.setHigh(0,16.8);
        //set Y bound
        bounds.setLow(1,0.2);
        bounds.setHigh(1,6.8);
        ss_->as<SE2BeliefSpace>()->setBounds(bounds);

        //Construct the control space
        ompl::control::ControlSpacePtr controlspace( new ompl::control::RealVectorControlSpace(ss_,2) ) ;
        cs_ = controlspace;

        // construct an instance of space information from this state space
        firm::SpaceInformation::SpaceInformationPtr si(new firm::SpaceInformation(ss_, cs_));
        siF_ = si;

        ompl::base::ProblemDefinitionPtr prblm(new ompl::base::ProblemDefinition(siF_));

        pdef_ = prblm;

        start_ = siF_->allocState();
        goal_  = siF_->allocState();

        setup_ = false;
    }

    virtual ~FIRM2DSetup(void)
    {
    }

    const firm::SpaceInformation::SpaceInformationPtr& getSpaceInformation() const
    {
        return siF_;
    }

    void setPathToSetupFile(const std::string &path)
    {
        pathToSetupFile_  = path;

        this->loadParameters();
    }

    void setStartState(const double X, const double Y, const double Yaw)
    {
        ompl::base::State *temp = siF_->allocState();

        temp->as<StateType>()->setXYYaw(X,Y,Yaw);

        siF_->copyState(start_, temp);

        siF_->freeState(temp);

    }

    void setGoalState(const double X, const double Y, const double Yaw)
    {
        ompl::base::State *temp = siF_->allocState();

        temp->as<StateType>()->setXYYaw(X,Y,Yaw);

        siF_->copyState(goal_,temp);

        siF_->freeState(temp);

    }


    void setStartAndGoalStates()
    {
        pdef_->setStartAndGoalStates(start_, goal_, 1.0);
    }

    void setup()
    {
        if(!setup_)
        {
            if(pathToSetupFile_.length() == 0)
            {
                throw ompl::Exception("Path to setup file not set!");
            }

            if(!hasEnvironment() || !hasRobot())
            {
                throw ompl::Exception("Robot/Environment mesh files not setup!");
            }

            ss_->as<SE2BeliefSpace>()->setBounds(inferEnvironmentBounds());

            // Create an FCL state validity checker and assign to space information
            const ompl::base::StateValidityCheckerPtr &fclSVC = this->allocStateValidityChecker(siF_, getGeometricStateExtractor(), false);
            siF_->setStateValidityChecker(fclSVC);

            // provide the observation model to the space
            ObservationModelMethod::ObservationModelPointer om(new CamAruco2DObservationModel(siF_, pathToSetupFile_.c_str()));
            siF_->setObservationModel(om);

            // Provide the motion model to the space
            MotionModelMethod::MotionModelPointer mm(new UnicycleMotionModel(siF_, pathToSetupFile_.c_str()));
            siF_->setMotionModel(mm);

            ompl::control::StatePropagatorPtr prop(ompl::control::StatePropagatorPtr(new UnicycleStatePropagator(siF_)));
            statePropagator_ = prop;
            siF_->setStatePropagator(statePropagator_);
            siF_->setPropagationStepSize(0.01); // this is the duration that a control is applied
            siF_->setStateValidityCheckingResolution(0.005);
            siF_->setMinMaxControlDuration(1,100);

            if(!start_ || !goal_)
            {
                throw ompl::Exception("Start/Goal not set");
            }

            this->setStartAndGoalStates();

            ompl::base::PlannerPtr plnr(new FIRM(siF_, false));

            planner_ = plnr;

            planner_->setProblemDefinition(pdef_);

            planner_->as<FIRM>()->setMinFIRMNodes(minNodes_);

            planner_->as<FIRM>()->setKidnappedState(kidnappedState_);

            planner_->setup();

            setup_ = true;
        }

    }

    ompl::base::PlannerStatus solve()
    {
        if(!setup_)
        {
            this->setup();
        }

        std::string pathXML = "FIRMRoadMap.xml";

        planner_->as<FIRM>()->loadRoadMapFromFile(pathXML);

        return planner_->solve(planningTime_);
    }

    void executeSolution()
    {
        planner_->as<FIRM>()->executeFeedback();
    }


    ompl::app::GeometricStateExtractor getGeometricStateExtractor(void) const
    {
        return boost::bind(&FIRM2DSetup::getGeometricComponentStateInternal, this, _1, _2);
    }

protected:

    const ompl::base::State* getGeometricComponentStateInternal(const ompl::base::State *state, unsigned int /*index*/) const
    {
        return state;
    }

    void loadParameters()
    {
        using namespace arma;

        TiXmlDocument doc(pathToSetupFile_);

        bool loadOkay = doc.LoadFile();

        if ( !loadOkay )
        {
            printf( "Could not load setup file in planning problem. Error='%s'. Exiting.\n", doc.ErrorDesc() );

            exit( 1 );
        }

        TiXmlNode* node = 0;

        TiXmlElement* itemElement = 0;

        node = doc.FirstChild( "PlanningProblem" );
        assert( node );

        TiXmlNode* child = 0;

        // Read the env mesh file
        child = node->FirstChild("Environment");
        assert( child );

        itemElement = child->ToElement();
        assert( itemElement );

        std::string environmentFilePath;
        itemElement->QueryStringAttribute("environmentFile", &environmentFilePath);

        this->setEnvironmentMesh(environmentFilePath);

        // Read the robot mesh file
        child  = node->FirstChild("Robot");
        assert( child );

        itemElement = child->ToElement();
        assert( itemElement );

        std::string robotFilePath;
        itemElement->QueryStringAttribute("robotFile", &robotFilePath);

        this->setRobotMesh(robotFilePath);

        /*
        // Read the roadmap filename
        child  = node->FirstChild("RoadMap");
        assert( child );
        itemElement = child->ToElement();
        assert( itemElement );
        itemElement->QueryStringAttribute("roadmapFile", &pathToSetupFile_);
        */

        // Read the start Pose
        child  = node->FirstChild("StartPose");
        assert( child );

        itemElement = child->ToElement();
        assert( itemElement );

        double startX = 0,startY = 0, startTheta = 0;

        itemElement->QueryDoubleAttribute("x", &startX);
        itemElement->QueryDoubleAttribute("y", &startY);
        itemElement->QueryDoubleAttribute("theta", &startTheta);

        setStartState(startX, startY, startTheta);

        // Read the Goal Pose
        child  = node->FirstChild("GoalPose");
        assert( child );

        itemElement = child->ToElement();
        assert( itemElement );

        double goalX = 0 , goalY = 0, goalTheta = 0;

        itemElement->QueryDoubleAttribute("x", &goalX);
        itemElement->QueryDoubleAttribute("y", &goalY);
        itemElement->QueryDoubleAttribute("theta", &goalTheta);

        setGoalState(goalX, goalY, goalTheta);

        // read planning time
        child  = node->FirstChild("PlanningTime");
        assert( child );

        itemElement = child->ToElement();
        assert( itemElement );

        double time = 0;

        itemElement->QueryDoubleAttribute("maxTime", &time) ;

        planningTime_ = time;

        // read planning time
        child  = node->FirstChild("FIRMNodes");
        assert( child );

        itemElement = child->ToElement();
        assert( itemElement );

        int nodeNum = 0;

        itemElement->QueryIntAttribute("minNodes", &nodeNum) ;

        minNodes_ = nodeNum;

        // Read Kidnapped State
        // Read the Goal Pose
        child  = node->FirstChild("KidnappedState");
        assert( child );

        itemElement = child->ToElement();
        assert( itemElement );

        double kX = 0 , kY = 0, kTheta = 0;

        itemElement->QueryDoubleAttribute("x", &kX);
        itemElement->QueryDoubleAttribute("y", &kY);
        itemElement->QueryDoubleAttribute("theta", &kTheta);

        kidnappedState_ = siF_->allocState();

        kidnappedState_->as<SE2BeliefSpace::StateType>()->setXYYaw(kX, kY, kTheta);

        OMPL_INFORM("Problem configuration is");

        std::cout<<"Path to environment mesh: "<<environmentFilePath<<std::endl;

        std::cout<<"Path to robot mesh: "<<robotFilePath<<std::endl;

        //std::cout<<"Path to roadmap file: "<<pathToRoadMapFile_<<std::endl;

        std::cout<<"Start Pose X: "<<startX<<" Y: "<<startY<<" Theta: "<<startTheta<<std::endl;

        std::cout<<"Goal Pose X: "<<goalX<<" Y: "<<goalY<<" Theta: "<<goalTheta<<std::endl;

        std::cout<<"Planning Time: "<<planningTime_<<" seconds"<<std::endl;

        std::cout<<"Min Nodes: "<<minNodes_<<" seconds"<<std::endl;

        std::cout<<"Kidnapped Pose x:"<<kX<<" y:"<<kY<<" theta:"<<kTheta<<std::endl;

    }

private:

    ompl::base::State *start_;

    ompl::base::State *goal_;

    ompl::base::State *kidnappedState_;

    firm::SpaceInformation::SpaceInformationPtr siF_;

    ompl::control::StatePropagatorPtr statePropagator_;

    ompl::control::ControlSpacePtr cs_;

    ompl::base::StateSpacePtr ss_;

    ompl::base::PlannerPtr planner_;

    ompl::base::ProblemDefinitionPtr pdef_;

    ompl::base::StateValidityCheckerPtr vc_;

    std::string pathToSetupFile_;

    double planningTime_;

    unsigned int minNodes_;

    bool setup_;

    std::string pathToRoadMapFile_;
};
#endif
