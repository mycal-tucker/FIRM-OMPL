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

#include "../../include/Spaces/CarrotBeliefSpace.h"

double CarrotBeliefSpace::StateType::meanNormWeight_  = -1;
double CarrotBeliefSpace::StateType::covNormWeight_   = -1;
double CarrotBeliefSpace::StateType::reachDist_   = -1;
arma::colvec CarrotBeliefSpace::StateType::normWeights_ = arma::zeros<arma::colvec>(3);

bool CarrotBeliefSpace::StateType::isReached(ompl::base::State *state) const
{
    // subtract the two beliefs and get the norm
    arma::colvec stateDiff = this->getArmaData() - state->as<CarrotBeliefSpace::StateType>()->getArmaData();

    arma::mat covDiff = this->getCovariance() -  state->as<CarrotBeliefSpace::StateType>()->getCovariance();

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


ompl::base::State* CarrotBeliefSpace::allocState(void) const
{
    //State *state = new State();
    State* state = allocState();
    return state;
}

void CarrotBeliefSpace::copyState(State *destination, const State *source) const
{
    destination->as<StateType>()->setX(source->as<StateType>()->getX());
    destination->as<StateType>()->setY(source->as<StateType>()->getY());
    destination->as<StateType>()->setZ(source->as<StateType>()->getZ());
    destination->as<StateType>()->setCovariance(source->as<StateType>()->getCovariance());
}

void CarrotBeliefSpace::freeState(State *state) const
{
    RealVectorStateSpace::freeState(state);
}

double CarrotBeliefSpace::distance(const State* state1, const State *state2)
{
    double dx = state1->as<StateType>()->getX() - state2->as<StateType>()->getX();
    double dy = state1->as<StateType>()->getY() - state2->as<StateType>()->getY();
    double dz = state1->as<StateType>()->getZ() - state2->as<StateType>()->getZ();

    std::cout<<"Getting distance :"<<std::endl;
    std::cin.get();

    return pow(dx*dx+dy*dy+dz*dz, 0.5);
}
void CarrotBeliefSpace::getRelativeState(const State *from, const State *to, State *state)
{
	state->as<StateType>()->setX(to->as<StateType>()->getX() - from->as<StateType>()->getX());
	state->as<StateType>()->setY(to->as<StateType>()->getY() - from->as<StateType>()->getY());
	state->as<StateType>()->setZ(to->as<StateType>()->getZ() - from->as<StateType>()->getZ());

    arma::mat fcov = from->as<StateType>()->getCovariance();
    arma::mat tocov = to->as<StateType>()->getCovariance();

    if (fcov.n_rows != 0 && fcov.n_cols != 0 && tocov.n_rows != 0 && tocov.n_cols != 0 )
    {
   		state->as<StateType>()->setCovariance(tocov - fcov);
    }
}

void CarrotBeliefSpace::printBeliefState(const State *state)
{
    std::cout<<"----Printing BeliefState----"<<std::endl;
    std::cout<<"State [X, Y, Z]: ";
    std::cout<<"["<<state->as<CarrotBeliefSpace::StateType>()->getX()<<", "<<
        state->as<CarrotBeliefSpace::StateType>()->getY() << ", " <<
        state->as<CarrotBeliefSpace::StateType>()->getZ() <<
        std::endl;
    std::cout<<"Covariance  is" <<std::endl;
    std::cout<<state->as<CarrotBeliefSpace::StateType>()->getCovariance()<<std::endl;
    std::cout<<"------End BeliefState-------"<<std::endl;
}

