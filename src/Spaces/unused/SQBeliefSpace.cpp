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

#include "../../include/Spaces/SQBeliefSpace.h"

double SQBeliefSpace::StateType::meanNormWeight_  = -1;
double SQBeliefSpace::StateType::covNormWeight_   = -1;
double SQBeliefSpace::StateType::reachDist_   = -1;
arma::colvec SQBeliefSpace::StateType::normWeights_ = arma::zeros<arma::colvec>(3);

bool SQBeliefSpace::StateType::isReached(ompl::base::State *state) const
{
    // subtract the two beliefs and get the norm
    arma::colvec stateDiff = this->getArmaData() - state->as<SQBeliefSpace::StateType>()->getArmaData();

    if(stateDiff[3] > boost::math::constants::pi<double>() )
    {

        stateDiff[3] =  (stateDiff[3] - 2*boost::math::constants::pi<double>()) ;
    }
    if( stateDiff[3] < -boost::math::constants::pi<double>() ){

        stateDiff[3] =  stateDiff[3] + 2*boost::math::constants::pi<double>() ;
    }

    arma::mat covDiff = this->getCovariance() -  state->as<SQBeliefSpace::StateType>()->getCovariance();

    arma::colvec covDiffDiag = covDiff.diag();

    // Need weighted supNorm of difference in means
    double meanNorm = arma::norm(stateDiff % normWeights_, "inf");

    double covDiagNorm = arma::norm(sqrt(abs(covDiffDiag)) % normWeights_, "inf");

    double norm2 =  std::max(meanNorm*meanNormWeight_, covDiagNorm*covNormWeight_) ;

    if(norm2 <= reachDist_)
    {
        return true;
    }

    return false;

}


ompl::base::State* SQBeliefSpace::allocState(void) const
{
    StateType *state = new StateType();
    allocStateComponents(state);
    return state;
}

void SQBeliefSpace::copyState(State *destination, const State *source) const
{
    destination->as<StateType>()->setX(source->as<StateType>()->getX());
    destination->as<StateType>()->setY(source->as<StateType>()->getY());
    destination->as<StateType>()->setZ(source->as<StateType>()->getZ());
    destination->as<StateType>()->setYaw(source->as<StateType>()->getYaw());
    destination->as<StateType>()->setCovariance(source->as<StateType>()->getCovariance());
}

void SQBeliefSpace::freeState(State *state) const
{
    CompoundStateSpace::freeState(state);
}

double SQBeliefSpace::distance(const State* state1, const State *state2)
{
    double dx = state1->as<StateType>()->getX() - state2->as<StateType>()->getX();
    double dy = state1->as<StateType>()->getY() - state2->as<StateType>()->getY();
    double dz = state1->as<StateType>()->getZ() - state2->as<StateType>()->getZ();

    std::cout<<"Getting distance :"<<std::endl;
    std::cin.get();

    return pow(dx*dx+dy*dy+dz*dz, 0.5);
}
void SQBeliefSpace::getRelativeState(const State *from, const State *to, State *state)
{
	state->as<StateType>()->setX(to->as<StateType>()->getX() - from->as<StateType>()->getX());
	state->as<StateType>()->setY(to->as<StateType>()->getY() - from->as<StateType>()->getY());
	state->as<StateType>()->setZ(to->as<StateType>()->getZ() - from->as<StateType>()->getZ());

	/*
    	Calculating relative angle is a bit tricky.
    	Refer to "interpolate" function of SO2StateSpace at line 122 of SO2StateSpace.h  in OMPL lib
        to see the original implementation in OMPL
	*/

	double diff = to->as<StateType>()->getYaw() - from->as<StateType>()->getYaw();
	if (fabs(diff) <= boost::math::constants::pi<double>())
        state->as<StateType>()->setYaw(diff);
    else
    {
        double v;
        if (diff > 0.0)
            diff = 2.0 * boost::math::constants::pi<double>() - diff;
        else
            diff = -2.0 * boost::math::constants::pi<double>() - diff;

        v = - diff ;
        // input states are within bounds, so the following check is sufficient
        if (v > boost::math::constants::pi<double>())
            v -= 2.0 * boost::math::constants::pi<double>();
        else
            if (v < -boost::math::constants::pi<double>())
                v += 2.0 * boost::math::constants::pi<double>();
    	state->as<StateType>()->setYaw(v);
    }

    arma::mat fcov = from->as<StateType>()->getCovariance();
    arma::mat tocov = to->as<StateType>()->getCovariance();

    if (fcov.n_rows != 0 && fcov.n_cols != 0 && tocov.n_rows != 0 && tocov.n_cols != 0 )
    {
   		state->as<StateType>()->setCovariance(tocov - fcov);
    }
}

void SQBeliefSpace::printBeliefState(const State *state)
{
    std::cout<<"----Printing BeliefState----"<<std::endl;
    std::cout<<"State [X, Y, Z, Yaw]: ";
    std::cout<<"["<<state->as<SQBeliefSpace::StateType>()->getX()<<", "<<
        state->as<SQBeliefSpace::StateType>()->getY() << ", " <<
        state->as<SQBeliefSpace::StateType>()->getZ()
        <<", "<<state->as<SQBeliefSpace::StateType>()->getYaw()<<"]"<<std::endl;
    std::cout<<"Covariance  is" <<std::endl;
    std::cout<<state->as<SQBeliefSpace::StateType>()->getCovariance()<<std::endl;
    std::cout<<"------End BeliefState-------"<<std::endl;
}

