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

#ifndef CARROT_BELIEF_SPACE_H_
#define CARROT_BELIEF_SPACE_H_

// OMPL includes
#include "ompl/base/spaces/RealVectorStateSpace.h"
//other includes
#include <boost/math/constants/constants.hpp>
#include <armadillo>

using namespace ompl::base;
class CarrotBeliefSpace : public ompl::base::RealVectorStateSpace
{

    public:

        /** \brief A belief in R3: (x, y, z, covariance) */
        class StateType : public RealVectorStateSpace::StateType
        {
        public:
            StateType(void) : RealVectorStateSpace::StateType()
            {
              //values = double[_dimension];
              //covariance_ = arma::zeros<arma::mat>(3,3);
              //controllerID_ = -1;

            }

            /** \brief Get the X component of the state */
            double getX(void) const
            {
                return as<RealVectorStateSpace::StateType>()->values[0];
            }

            /** \brief Get the Y component of the state */
            double getY(void) const
            {
                return this->values[1];
            }

            /** \brief Get the Z component of the state */
            double getZ(void) const
            {
                return this->values[2];
            }

            arma::mat getCovariance(void) const
            {
                return covariance_;
            }

            /** \brief Set the X component of the state */
            void setX(double x)
            {
                this->values[0] = x;
                                //std::cout << "value of x is: " << x << std::endl;

            }

            /** \brief Set the Y component of the state */
            void setY(double y)
            {
                this->values[1] = y;
            }

            /** \brief Set the Y component of the state */
            void setZ(double z)
            {
                this->values[2] = z;
            }

            /** \brief Set the X,Y,Z components of the state */
            void setXYZ(double x, double y, double z)
            {
                setX(x);
                setY(y);
                setZ(z);
            }


            void setCovariance(arma::mat cov){
                covariance_ = cov;
            }

            void setID(size_t t) {
                controllerID_ = t;
            }

            arma::colvec getArmaData(void) const
            {
                arma::colvec stateVec(3);

                std::cout << "X: " << getX() << std::endl;

                stateVec[0] = getX();
                stateVec[1] = getY();
                stateVec[2] = getZ();
                return stateVec;
            }

            /** \brief Checks if the input state has stabilized to this state (node reachability check) */
            bool isReached(ompl::base::State *state) const;

            static double meanNormWeight_, covNormWeight_, reachDist_;

            static arma::colvec normWeights_;

        private:
              arma::mat covariance_;
              size_t controllerID_;

        };


        CarrotBeliefSpace() : RealVectorStateSpace(3)
        {
            setName("CARROT_BELIEF" + getName());
            type_ = STATE_SPACE_REAL_VECTOR;
            //bounds_ = 3;
            //dimension_ = 3;
            //stateBytes_ = 12*sizeof(double); //state(3)+cov(9)
//            type_ = STATE_SPACE_CARROT;
        }

        virtual ~CarrotBeliefSpace(void)
        {
        }

        /** \copydoc RealVectorStateSpace::setBounds() */
        void setBounds(const RealVectorBounds &bounds)
        {
            //std::cout << "setting bounds in space" << std::endl;
            //this->setBounds(bounds);
            as<RealVectorStateSpace>()->setBounds(bounds);

        }

	/*bool satisfiesBounds(const State *state) const
	{
	    RealVectorStateSpace::satisfiesBounds(state->as<RealVectorStateSpace::StateType>());
	}*/

        /** \copydoc RealVectorStateSpace::getBounds() */
        const RealVectorBounds& getBounds(void) const
        {
            return as<RealVectorStateSpace>()->getBounds();
        }

        virtual State* allocState(void) const;
        virtual void copyState(State *destination,const State *source) const;
        virtual void freeState(State *state) const;

        //virtual void registerProjections(void);
        virtual double distance(const State* state1, const State *state2);

        // gets the relative vector between "from" and "to"
        // equivalent to result = vectorA-vectorB
        void getRelativeState(const State *from, const State *to, State *state);

        void printBeliefState(const State *state);

};
#endif

/*
  BeliefSpace(const SE2StateSpace& state = SE2StateSpace(),
                const arma::mat& covariance = arma::zeros<arma::mat>(0,0),
                double reachDist = 0.0,
                size_t controllerID = -1);

  GaussianBelief& operator-=(const GaussianBelief& b);
  GaussianBelief operator-(const GaussianBelief& b) const;
  GaussianBelief operator-() const;

  bool operator==(const GaussianBelief& b) const;

  virtual bool equalStates

  //virtual const string GetName() const {return "GaussianBelief";}

  //template<class DistanceMetricPointer>
  //void GetRandomRay(double _incr, Environment* _env,  DistanceMetricPointer _dm, bool _norm=true);

  size_t GetControllerID() const {return controllerID_;}
  void SetControllerID(size_t id){controllerID_ = id;}

  void SetCovariance(const arma::mat& covariance) {covariance_ = covariance;}

  const arma::mat& GetCovariance() const {return covariance_;}

  double Norm();

  bool IsReached(const GaussianBelief& b) const;

  //void Draw();

  static double meanNormWeight_, covNormWeight_;

  //virtual ostream& Write(ostream& _os) {return (_os << *this);}

  //I/O
  //friend ostream& operator<< (ostream&, const GaussianBelief& _gb);
  //friend istream& operator>> (istream&, GaussianBelief& _gb);

private:
  arma::mat covariance_;
  double reachDist_;
  size_t controllerID_;
*/
