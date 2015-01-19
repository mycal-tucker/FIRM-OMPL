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
#include "ompl/base/spaces/RealVectorStateSpace.h"
//#include "ompl/base/spaces/SE3StateSpace.h"
//other includes
#include <boost/math/constants/constants.hpp>
#include <armadillo>

using namespace ompl::base;
class SE3BeliefSpace : public RealVectorStateSpace
{

    public:

        /** \brief A belief in SE(3): (x, y, z, roll, pitch, yaw, time
         * derivatives, covariance) */
        class StateType : public RealVectorStateSpace::StateType
        {
        public:
            StateType(void) : RealVectorStateSpace::StateType()
            {
              covariance_ = arma::zeros<arma::mat>(12,12);
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

            double getXDot(void) const
            {
                return as<RealVectorStateSpace::StateType>(0)->values[3];
            }

            /** \brief Get the Ydot component of the state */
            double getYDot(void) const
            {
                return as<RealVectorStateSpace::StateType>(0)->values[4];
            }
            
            /** \brief Get the Zdot component of the state */
            double getZDot(void) const
            {
                return as<RealVectorStateSpace::StateType>(0)->values[5];
            }
            
            /** \brief Get the roll component of the state. */
            double getRoll(void) const
            {
                return as<RealVectorStateSpace::StateType>(0)->values[6]; 
            }
            
            /** \brief Get the pitch component of the state. */
            double getPitch(void) const
            {
                return as<RealVectorStateSpace::StateType>(0)->values[7];
            }
            /** \brief Get the yaw component of the state.  */
            double getYaw(void) const
            {
                return as<RealVectorStateSpace::StateType>(0)->values[8];
            }

            /** \brief Get the roll dot component of the state. */
            double getRollDot(void) const
            {
                return as<RealVectorStateSpace::StateType>(0)->values[9]; 
            }
            
            /** \brief Get the pitch component of the state. */
            double getPitchDot(void) const
            {
                return
                as<RealVectorStateSpace::StateType>(0)->values[10];
            }
            /** \brief Get the yaw dot component of the state.  */
            double getYawDot(void) const
            {
                return
                as<RealVectorStateSpace::StateType>(0)->values[11];
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

            /** \brief Set the X dot component of the state */
            void setXDot(double xdot)
            {
                as<RealVectorStateSpace::StateType>(0)->values[3] =
                xdot;
            }

            /** \brief Set the Y dot component of the state */
            void setYDot(double ydot)
            {
                as<RealVectorStateSpace::StateType>(0)->values[4] =
                ydot;
            }
            
            /** \brief Set the Z component of the state */
            void setZDot(double zdot)
            {
                as<RealVectorStateSpace::StateType>(0)->values[5] =
                zdot;
            }


            /** \brief Set the roll component of the state. */
            void setRoll(double roll)
            {
                as<RealVectorStateSpace::StateType>(0)->values[6] = roll;
            }
            
            /** \brief Set the pitch component of the state. */
            void setPitch(double pitch)
            {
                as<RealVectorStateSpace::StateType>(0)->values[7] = pitch;
            }
            
            /** \brief Set the yaw component of the state. */
            void setYaw(double yaw)
            {
                as<RealVectorStateSpace::StateType>(0)->values[8] = yaw;
            }

            /** \brief Set the roll dot component of the state. */
            void setRollDot(double rolldot)
            {
                as<RealVectorStateSpace::StateType>(0)->values[9] =
                rolldot;
            }
            
            /** \brief Set the pitch dot component of the state. */
            void setPitchDot(double pitchdot)
            {
                as<RealVectorStateSpace::StateType>(0)->values[10] =
                pitchdot;
            }
            
            /** \brief Set the yaw dot component of the state. */
            void setYawDot(double yawdot)
            {
                as<RealVectorStateSpace::StateType>(0)->values[11] =
                yawdot;
            }

            void setFullState(arma::colvec state)
            {
                setX(state[0]);
                setY(state[1]);
                setZ(state[2]);
                setXDot(state[3]);
                setYDot(state[4]);
                setZDot(state[5]);
                setRoll(state[6]);
                setPitch(state[7]);
                setYaw(state[8]);
                setRoll(state[9]);
                setPitch(state[10]);
                setYaw(state[11]);
            }

            void setCovariance(arma::mat cov){ //using just one covariance for all dimensions
                covariance_ = cov;
            }

            arma::colvec getArmaData(void) const
            {
                arma::colvec stateVec(12);

                stateVec[0] = getX();
                stateVec[1] = getY();
                stateVec[2] = getZ();
                stateVec[3] = getXDot();
                stateVec[4] = getYDot();
                stateVec[5] = getZDot();
                stateVec[6] = getRoll();
                stateVec[7] = getPitch();
                stateVec[8] = getYaw();
                stateVec[9] = getRollDot();
                stateVec[10] = getPitchDot();
                stateVec[11] = getYawDot();

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


        SE3BeliefSpace(unsigned int dim) : RealVectorStateSpacd()
        {
            dimension_ = dim;
            setName("SE3_BELIEF" + getName());
            type_ = STATE_SPACE_SE3;
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

//all this was commented out in SE2BeliefSpace
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
