/*********************************************************************
* Based on SE2BeliefSpace.cpp
*
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

#ifndef BELIEF_SPACE_H_
#define BELIEF_SPACE_H_

// OMPL includes
// #include "ompl/base/spaces/SE2StateSpace.h" //removed
#include "ompl/base/spaces/SE3StateSpace.h"
//other includes
#include <boost/math/constants/constants.hpp>
#include <armadillo>

using namespace ompl::base;
class SE3BeliefSpace : public ompl::base::CompoundStateSpace
{

    public:

        /** \brief A belief in SE(3): (x, y, z, roll, pitch, yaw, covariance) */
        class StateType : public CompoundStateSpace::StateType
        {
        public:
            StateType(void) : CompoundStateSpace::StateType()
            {
              covariance_ = arma::zeros<arma::mat>(6,6);
              controllerID_ = -1;

            }

            /** \brief Get the X component of the state */
            double getX(void) const
            {
                return as<RealVectorStateSpace::StateType>(0)->values[0];
            }

            /** \brief Get the Y component of the state */
            double getY(void) const
            {
                return as<RealVectorStateSpace::StateType>(0)->values[1];
            }
            
            /** \brief Get the Z component of the state */
            double getZ(void) const
            {
                return as<RealVectorStateSpace::StateType>(0)->values[2];
            }
            
            /** \brief Get the roll component of the state. */
            double getRoll(void) const
            {
                return as<SO3StateSpace::StateType>(1)->value; //TODO what does the (1) mean? It was there for yaw
            }
            
            /** \brief Get the pitch component of the state. */
            double getPitch(void) const
            {
                return as<SO3StateSpace::StateType>(1)->value;
            }
            /** \brief Get the yaw component of the state.  */
            double getYaw(void) const
            {
                return as<SO3StateSpace::StateType>(1)->value;
            }

            arma::mat getCovariance(void) const //for now use same covariance for all dimensions
            {
                return covariance_;
            }

            /** \brief Set the X component of the state */
            void setX(double x)
            {
                as<RealVectorStateSpace::StateType>(0)->values[0] = x;
            }

            /** \brief Set the Y component of the state */
            void setY(double y)
            {
                as<RealVectorStateSpace::StateType>(0)->values[1] = y;
            }
            
            /** \brief Set the Z component of the state */
            void setZ(double z)
            {
                as<RealVectorStateSpace::StateType>(0)->values[2] = z;
            }

            /** \brief Set the X, Y, and Z components of the state */
            void setXYZ(double x, double y, double z)
            {
                setX(x);
                setY(y);
                setZ(z);
            }

            /** \brief Set the roll component of the state. */
            void setRoll(double roll)
            {
                as<SO2StateSpace::StateType>(1)->value = roll; //TODO what's with the (1)?
            }
            
            /** \brief Set the pitch component of the state. */
            void setPitch(double pitch)
            {
                as<SO2StateSpace::StateType>(1)->value = pitch;
            }
            
            /** \brief Set the yaw component of the state. */
            void setYaw(double yaw)
            {
                as<SO2StateSpace::StateType>(1)->value = yaw;
            }

            void setXYZRollPitchYaw(double x, double y, double z, double roll, double pitch, double yaw)
            {
                setX(x);
                setY(y);
                setZ(z);
                setRoll(roll);
                setPitch(pitch);
                setYaw(yaw);
            }

            void setCovariance(arma::mat cov){ //using just one covariance for all dimensions
                covariance_ = cov;
            }

            arma::colvec getArmaData(void) const
            {
                arma::colvec stateVec(3);

                stateVec[0] = getX();
                stateVec[1] = getY();
                stateVec[2] = getYaw();
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


        SE3BeliefSpace(void) : CompoundStateSpace()
        {
            setName("SE3_BELIEF" + getName());
            type_ = STATE_SPACE_SE3;
            addSubspace(StateSpacePtr(new RealVectorStateSpace(3)), 1.0); //for x, y, z. I don't understand what 1.0 does
            addSubspace(StateSpacePtr(new SO3StateSpace()), 0.5); //I don't understan what 0.5 does
            lock();
        }

        virtual ~SE3BeliefSpace(void)
        {
        }

        /** \copydoc RealVectorStateSpace::setBounds() */
        void setBounds(const RealVectorBounds &bounds)
        {
            as<RealVectorStateSpace>(0)->setBounds(bounds);
        }

        /** \copydoc RealVectorStateSpace::getBounds() */
        const RealVectorBounds& getBounds(void) const
        {
            return as<RealVectorStateSpace>(0)->getBounds();
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
