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

/* Authors:  Ali-akbar Agha-mohammadi, Saurav Agarwal */


#include <tinyxml.h>
#include "../../include/Spaces/CarrotBeliefSpace.h"
#include "../../include/MotionModels/CarrotMotionModel.h"
#include "../../include/Utils/FIRMUtils.h"
#include <algorithm>

//Produce the next state, given the current state, a control and a noise
void CarrotMotionModel::Evolve(const ompl::base::State *state, const ompl::control::Control *control, const NoiseType& w, ompl::base::State *result)
{

    using namespace arma;

    typedef typename CarrotMotionModelMethod::StateType StateType;

    arma::colvec u = OMPL2ARMA(control);
    //std::cout << "control: " << u <<std::endl;

    const colvec& Un = w.subvec(0, this->controlDim_-1);

    const colvec& Wg = w.subvec(this->controlDim_, this->noiseDim_-1);
    //std::cout << "process noise: " << Wg << std::endl;

    colvec x = state->as<StateType>()->getArmaData();
        //std::cout << "start state: " << x << std::endl;

    //colvec xPrev = x;

    x += u + Un;
    //std::cout << "[MotionModel] End state: " << x <<std::endl;

    result->as<StateType>()->setXYZ(x[0],x[1],x[2]);
    //std::cout << "difference" << result->as<StateType>()->getArmaData()-x-u << std::endl;
}


void CarrotMotionModel::generateOpenLoopControls(const ompl::base::State *startState,
                                                  const ompl::base::State *endState,
                                                  std::vector<ompl::control::Control*> &openLoopControls)
{

    using namespace arma;
    typedef typename CarrotMotionModelMethod::StateType StateType;

    colvec start = startState->as<StateType>()->getArmaData(); // turn into colvec
    colvec end = endState->as<StateType>()->getArmaData(); // turn into colvec
    //std::cout << "start: " << start << " end: " << end << std::endl;

    /*colvec x_c, y_c, z_c;
    x_c << start[0] << endr
      << end[0] << endr;
    y_c << start[1] << endr
      << end[1] << endr;
    z_c << start[2] << endr
      << end[2] << endr;*/

    double delta_x = end[0]-start[0];
    double delta_y = end[1]-start[1];
    double delta_z = end[2]-start[2];
    colvec diff;
    diff << delta_x << delta_y << delta_z << endr;

    double dist = norm(diff,2);
    double steps = fabs(dist/(maxVelocity_*this->dt_));
    double fsteps = 0;
    if (steps > 1) fsteps = floor(steps);
    else fsteps = steps;

    /*double delta_x = 0;
    double delta_y = 0;
    double delta_z = 0;
    double x_steps = 0;
    double y_steps = 0;
    double z_steps = 0;
    int kf = 0;

    // count x steps

    x_steps = fabs(delta_x/(maxXVelocity_*this->dt_));

    // count y steps
    delta_y = y_c[1]-y_c[0];
    y_steps = fabs(delta_y/(maxYVelocity_*this->dt_));

    //count z steps
    delta_z= z_c[1]-z_c[0];
    z_steps = fabs(delta_z/(maxZVelocity_*this->dt_));

    double si = 0;
    double csi = 0;
    if (x_steps > std::max(y_steps, z_steps)) {
        si = x_steps;
        csi = ceil(si);
    } else if (y_steps > std::max(x_steps, z_steps)) {
        si = y_steps;
        csi = ceil(si);
    } else {
        si = z_steps;
        csi = ceil(si);
    }*/

    const double x_carrot = delta_x/fsteps;
    const double y_carrot = delta_y/fsteps;
    const double z_carrot = delta_z/fsteps;

    colvec u_const_trans;

    u_const_trans << x_carrot<< endr
                << y_carrot<< endr
                << z_carrot<< endr;

    for(int j=0; j<fsteps; ++j)
    {
        ompl::control::Control *tempControl = si_->allocControl();
        ARMA2OMPL(u_const_trans, tempControl);
        openLoopControls.push_back(tempControl);
    }
    if (steps > fsteps)
    {
        ompl::control::Control *tempControl = si_->allocControl();
        ARMA2OMPL(u_const_trans*(steps-fsteps), tempControl);
        openLoopControls.push_back(tempControl);
    }


}

void CarrotMotionModel::generateOpenLoopControlsForPath(const ompl::geometric::PathGeometric path, std::vector<ompl::control::Control*> &openLoopControls)
{
    for(int i=0;i<path.getStateCount()-1;i++)
    {
        std::vector<ompl::control::Control*> olc;

        this->generateOpenLoopControls(path.getState(i),path.getState(i+1),olc) ;

        openLoopControls.insert(openLoopControls.end(),olc.begin(),olc.end());
    }
}

typename CarrotMotionModel::NoiseType
CarrotMotionModel::generateNoise(const ompl::base::State *state, const ompl::control::Control* control)
{

    using namespace arma;

    NoiseType noise(this->noiseDim_);
    colvec indepUn = randn(this->controlDim_,1);
    mat P_Un = controlNoiseCovariance(control);
    colvec Un = indepUn % sqrt((P_Un.diag()));

    colvec Wg = sqrt(P_Wg_) * randn(this->stateDim_,1);
    noise = join_cols(Un, Wg);

    return noise;
}

typename CarrotMotionModel::JacobianType
CarrotMotionModel::getStateJacobian(const ompl::base::State *state, const ompl::control::Control* control, const NoiseType& w)
{

    using namespace arma;

    typedef typename CarrotMotionModelMethod::StateType StateType;

    JacobianType A = eye(this->stateDim_,this->stateDim_);
    return A;
}

typename CarrotMotionModel::JacobianType
CarrotMotionModel::getControlJacobian(const ompl::base::State *state, const ompl::control::Control* control, const NoiseType& w)
{

    using namespace arma;
    typedef typename CarrotMotionModelMethod::StateType StateType;

    colvec xData = state->as<StateType>()->getArmaData();
    assert (xData.n_rows == (size_t)this->stateDim_);

    JacobianType B = eye(this->stateDim_,this->controlDim_);
    return B;

}

typename CarrotMotionModel::JacobianType
CarrotMotionModel::getNoiseJacobian(const ompl::base::State *state, const ompl::control::Control* control, const NoiseType& w)
{

    using namespace arma;
    typedef typename CarrotMotionModelMethod::StateType StateType;

    //colvec xData = state->as<StateType>()->getArmaData();
    //assert (xData.n_rows == (size_t)this->stateDim_);
    //const colvec& Un = w.subvec(0,this->controlDim_-1);
    //const colvec& Wg = w.subvec(this->controlDim_,this->noiseDim_-1);

    mat G(3,6);
    G << 1 << 0 << 0 << 1 << 0 << 0 << endr
      << 0 << 1 << 0 << 0 << 1 << 0 << endr
      << 0 << 0 << 1 << 0 << 0 << 1 << endr;

    return G;

}

arma::mat CarrotMotionModel::processNoiseCovariance(const ompl::base::State *state, const ompl::control::Control* control)
{

    using namespace arma;

    mat P_Un = controlNoiseCovariance(control);
    mat Q_processNoise = zeros<mat>(P_Un.n_rows + P_Wg_.n_rows, P_Un.n_cols +
    P_Wg_.n_cols);

    Q_processNoise.submat(0, 0, P_Un.n_rows-1, P_Un.n_cols-1) = P_Un;
    Q_processNoise.submat(P_Un.n_rows, P_Un.n_cols,
                  P_Un.n_rows + P_Wg_.n_rows -1,
                  P_Un.n_cols + P_Wg_.n_cols -1) = P_Wg_;

    return Q_processNoise;

}


arma::mat CarrotMotionModel::controlNoiseCovariance(const ompl::control::Control* control)
{

    using namespace arma;

    arma::colvec u = OMPL2ARMA(control);

    colvec uStd = eta_ % u + sigma_;

    mat P_Un = diagmat(square(uStd));

    return P_Un;
}


void CarrotMotionModel::loadParameters(const char *pathToSetupFile)
{
    using namespace arma;

    TiXmlDocument doc(pathToSetupFile);
    bool loadOkay = doc.LoadFile();

    if ( !loadOkay )
    {
        printf( "Could not load setup file in motion model. Error='%s'. Exiting.\n", doc.ErrorDesc() );

        exit( 1 );
    }

    TiXmlNode* node = 0;
    TiXmlElement* itemElement = 0;

    node = doc.FirstChild( "MotionModels" );
    assert( node );

    TiXmlNode* child = 0;

    child = node->FirstChild("CarrotMotionModel");

    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );

    double sigmaV=0;
    double etaV = 0;
    double sigmaOmega=0;
    double etaOmega=0;
    double windNoisePos=0;
    double windNoiseAng = 0;
/*
    double maxXVelocity=0;
    double maxYVelocity=0;
    double maxZVelocity =0;
*/
    double maxVelocity = 0;
    double dt = 0;

    itemElement->QueryDoubleAttribute("sigmaV", &sigmaV) ;
    itemElement->QueryDoubleAttribute("etaV", &etaV) ;
    //itemElement->QueryDoubleAttribute("sigmaOmega", &sigmaOmega) ;
    //itemElement->QueryDoubleAttribute("etaOmega", &etaOmega) ;
    itemElement->QueryDoubleAttribute("wind_noise_pos", &windNoisePos) ;
    //itemElement->QueryDoubleAttribute("wind_noise_ang", &windNoiseAng) ;=
    //itemElement->QueryDoubleAttribute("max_x_velocity", &maxXVelocity) ;
    //itemElement->QueryDoubleAttribute("max_y_velocity", &maxYVelocity) ;
    //itemElement->QueryDoubleAttribute("max_z_velocity", &maxZVelocity) ;
    itemElement->QueryDoubleAttribute("max_velocity", &maxVelocity) ;
    itemElement->QueryDoubleAttribute("dt", &dt) ;

    this->sigma_ << sigmaV << sigmaV << sigmaV << endr;
    this->eta_  << etaV << etaV << etaV << endr;

    rowvec Wg_root_vec(3);
    Wg_root_vec << windNoisePos << windNoisePos << windNoisePos << endr;
    P_Wg_ = diagmat(square(Wg_root_vec));

    /*maxXVelocity_  = maxXVelocity;
    maxYVelocity_  = maxYVelocity;
    maxZVelocity_ = maxZVelocity;*/
    maxVelocity_ = maxVelocity;
    dt_                 = dt;

    OMPL_INFORM("CarrotMotionModel: sigma_ = ");
    std::cout<<sigma_<<std::endl;

    OMPL_INFORM("CarrotMotionModel: eta_ = ");
    std::cout<<eta_<<std::endl;

    OMPL_INFORM("CarrotMotionModel: P_Wg_ = ");
    std::cout<<P_Wg_<<std::endl;

    /*
    OMPL_INFORM("CarrotMotionModel: max X Velocity (m/s)    = %f",
    maxXVelocity_ );

    OMPL_INFORM("CarrotMotionModel: max Y Velocity (m/s)    = %f",
    maxYVelocity_);

    OMPL_INFORM("CarrotMotionModel: max Z Velocity (rad/s) =%f",
    maxZVelocity_);
    */
    OMPL_INFORM("CarrotMotionModel: max Velocity (m/s)    = %f",
    maxVelocity_);
    OMPL_INFORM("CarrotMotionModel: Timestep (seconds) = %f", dt_);

}
