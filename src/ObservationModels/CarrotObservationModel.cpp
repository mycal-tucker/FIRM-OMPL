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

/* Authors: Saurav Agarwal, Ali-akbar Agha-mohammadi */
#include "../../include/Spaces/CarrotBeliefSpace.h"
#include "../../include/ObservationModels/CarrotObservationModel.h"
#include <tinyxml.h>
#include "../../include/Visualization/CarrotVisualizer.h"
#include "../../include/Utils/FIRMUtils.h"


/*
  For each landmark, produces an observation that is the range
  of the given State from that landmark. Result is the concatenation of all
  such observations Obs dim is the number of  landmarks
*/
CarrotObservationModel::ObservationType CarrotObservationModel::getObservation(const ompl::base::State *state, bool isSimulation)
{
    using namespace arma;

    //colvec xVec = state->as<SE2BeliefSpace::StateType>()->getArmaData();

    ObservationType z;

    int counter = 0;

    //generate observation from state, and corrupt with the given noise
    for(unsigned int i = 0; i < landmarks_.size(); i++)
    {


        double landmarkRange = 0; double landmarkBearing = 0; double landmarkPitch;
        this->calculateRangeBearingPitchToLandmark(state,landmarks_[i],landmarkRange,landmarkBearing,landmarkPitch);

        //cout<<"Trying to resize"<<endl;
        z.resize((counter+1)*singleObservationDim ,  1);

        colvec noise = zeros<colvec>(landmarkInfoDim);

        if(isSimulation)
        {

            // generate gaussian noise
            //get standard deviations of noise (sqrt of covariance matrix)
            //extract state from Cfg and normalize
            //generate noise scaling/shifting factor

            colvec noise_std = this->etaD_*landmarkRange + this->sigma_;

            //generate raw noise
            colvec randNoiseVec = randn<colvec>(landmarkInfoDim);

            //generate noise from a distribution scaled and shifted from
            //normal distribution N(0,1) to N(0,eta*range + sigma)
            //(shifting was done in GetNoiseCovariance)
            noise = noise_std%randNoiseVec;
        }

        z[singleObservationDim*counter] = landmarks_[i](0) ; // id of the landmark
        z[singleObservationDim*counter+1] = landmarkRange + noise[0]; // distance to landmark
        z[singleObservationDim*counter+2] = landmarkBearing + noise[1];
        z[singleObservationDim*counter+3] = landmarkPitch + noise[2];
        // bearing to landmark


        counter++;
    }

  return z;

}

bool CarrotObservationModel::isUnique(const arma::colvec z)
{

    //int singleObservationDim = 3;
    //Need to check if observations are not repeated in Zg.
    for(unsigned int k = 0; k< z.n_rows / singleObservationDim ;k++)
    {
        for(unsigned int p = 0; p< z.n_rows / singleObservationDim ;p++)
        {
            if(k!=p && z[singleObservationDim*k]==z[singleObservationDim*p])
            {
                return false;
            }
        }

    }
    return true;
}

typename CarrotObservationModel::ObservationType
CarrotObservationModel::getObservationPrediction(const ompl::base::State *state, const ObservationType& Zg)
{

    using namespace arma;

    //colvec xVec =  state->as<CarrotBeliefSpace::StateType>()->getArmaData();

    ObservationType z;

    for(unsigned int k = 0; k< Zg.n_rows / singleObservationDim ;k++)
    {
        // The candidate landmark is the closest landmark in the list with the same ID as that of what real robot sees
        colvec candidate;

        int candidateIndx = this->findCorrespondingLandmark(state,
        Zg.subvec(singleObservationDim*k,singleObservationDim*k+3), candidate);

        z.resize((k+1)*singleObservationDim ,  1);
        z[singleObservationDim*k]     = candidate(0)  ; // id of the landmark
        z[singleObservationDim*k + 1] = candidate(1) ;  // distance to landmark
        z[singleObservationDim*k + 2] = candidate(2) ;  // bearing to landmark
        z[singleObservationDim*k+3]   = candidate(3);   // pitch to landmark


    }

  return z;

}

void CarrotObservationModel::calculateRangeBearingPitchToLandmark(const
ompl::base::State *state, const arma::colvec& landmark, double& range,
double& bearing, double& pitch)
{
    using namespace arma;

    colvec xVec =  state->as<CarrotBeliefSpace::StateType>()->getArmaData();
    colvec diff =  landmark.subvec(1,3) - xVec.subvec(0,2);
    //std::cout << "landmark: " << landmark.subvec(1,3) << std::endl;

    //norm is the 2nd Holder norm, i.e. the Euclidean norm
    range = norm(diff,2);
    bearing = atan2(diff[1],diff[0]);
    pitch = -atan2(diff[2],norm(diff.subvec(0,1),2));

    FIRMUtils::normalizeAngleToPiRange(bearing);
    FIRMUtils::normalizeAngleToPiRange(pitch);

}

double CarrotObservationModel::getDataAssociationLikelihood(const arma::colvec trueObs, const arma::colvec predictedObs)
{
    // Find the most likely landmark to predict associated with the true observation
    double weight = 0.0;

    arma::colvec noise = this->sigma_;

    arma::mat covariance = arma::diagmat(arma::pow(noise,2));

    arma::colvec innov = trueObs-predictedObs;
   // std::cout << "Innovation: " << innov << std::endl;

    FIRMUtils::normalizeAngleToPiRange(innov(1));
    FIRMUtils::normalizeAngleToPiRange(innov(2));



    arma::mat t = -0.5*trans(innov)*covariance.i()*innov;

    weight = std::exp(t(0,0));

    //std::cout << weight << std::endl;

    assert(weight >= 0 && "Weight cannot be less than 0!");

    return weight;
}


int CarrotObservationModel::findCorrespondingLandmark(const ompl::base::State *state, const arma::colvec &observedLandmark, arma::colvec &candidateObservation)
{
    using namespace arma;

    int landmarkID = observedLandmark[0];

    double maxLikelihood = -1.0;

    double candidatelandmarkRange = 0;
    double candidatelandmarkBearing = 0;
    double candidatelandmarkPitch = 0;

    int candidateIndx = -1;

    for(unsigned int i = 0; i < landmarks_.size() ; i++)
    {
        if(landmarks_[i](0) == landmarkID)
        {
            double landmarkRange = 0; double landmarkBearing = 0; double landmarkPitch = 0;

            // get range and bearing to landmark
            this->calculateRangeBearingPitchToLandmark(state, landmarks_[i],
            landmarkRange, landmarkBearing, landmarkPitch);

            arma::colvec prediction;

            prediction<<landmarkRange<<landmarkBearing<<landmarkPitch<<endr;

            // calculate the likelihood
            double lkhd = getDataAssociationLikelihood(observedLandmark.subvec(1,3), prediction);

            if(lkhd > maxLikelihood)
            {
                candidateIndx = i;
                maxLikelihood = lkhd;
                candidatelandmarkRange = landmarkRange;
                candidatelandmarkBearing = landmarkBearing;
                candidatelandmarkPitch = landmarkPitch;
            }
        }

    }

     //std::cout << "max likelihood: " << maxLikelihood << std::endl;

    assert(candidateIndx >= 0 && "Candidate index cannot be negative");

    // The observation is id, range of the landmark
    candidateObservation<<landmarkID<<candidatelandmarkRange<<candidatelandmarkBearing<<candidatelandmarkPitch<<endr;

    assert(candidateIndx>=0);

    return candidateIndx;

}

typename CarrotObservationModel::JacobianType
CarrotObservationModel::getObservationJacobian(const ompl::base::State *state, const ObsNoiseType& v,
  const ObservationType& z)
{
    using namespace arma;

    unsigned int number_of_landmarks = z.n_rows / singleObservationDim ;

    colvec xVec = state->as<CarrotBeliefSpace::StateType>()->getArmaData();

    mat H( (landmarkInfoDim)* number_of_landmarks, stateDim); // Since we are passing the common id list

    for(unsigned int i = 0; i < number_of_landmarks ; ++i)
    {
        colvec candidate;

        int Indx = this->findCorrespondingLandmark(state,
                        z.subvec(i*singleObservationDim,i*singleObservationDim+3), candidate);

        colvec diff =  landmarks_[Indx].subvec(1,3) - xVec.subvec(0,2);
        //xVec[0] = 0; xVec[1] = 0; //fix this eventually
        //double phi = atan2(diff[1], diff[0]);

        double r_2 = norm(diff.subvec(0,1),2);
        double r_3 = norm(diff,2);

        mat H_i((landmarkInfoDim),stateDim);

        H_i <<   -diff[0]/r_3    <<  -diff[1]/r_3    <<   -diff[2]/r_3 << endr
            <<   diff[1]/(r_2*r_2)  <<  -diff[0]/(r_2*r_2)  <<  0 << endr
            <<   -diff[0]*diff[2]/(r_2*r_3*r_3)
            <<   -diff[1]*diff[2]/(r_2*r_3*r_3)
            <<    r_2/(r_3*r_3) << endr;

       /* H_i <<  -cos(phi)    <<  -sin(phi)   <<   1 << endr
            <<   sin(phi)/r  <<  -cos(phi)/r  <<  0 << endr;*/


        H.submat((landmarkInfoDim)*i, 0, (landmarkInfoDim)*i+2, 2) = H_i;

    }

    return H;
}


typename CarrotObservationModel::JacobianType
CarrotObservationModel::getNoiseJacobian(const ompl::base::State *state, const ObsNoiseType& _v, const ObservationType& z)
{
  using namespace arma;

  //noise jacobian is just an identity matrix
  int number_of_landmarks = z.n_rows / singleObservationDim ;

  mat M = eye(number_of_landmarks*3, number_of_landmarks*3);

  return M;

}



arma::mat CarrotObservationModel::getObservationNoiseCovariance(const ompl::base::State *state, const ObservationType& z)
{
    using namespace arma;

    //extract state from Cfg and normalize
    colvec xVec = state->as<CarrotBeliefSpace::StateType>()->getArmaData();

    unsigned int number_of_landmarks = z.n_rows/singleObservationDim ;

    //generate noise scaling/shifting factors
    colvec noise( number_of_landmarks*(landmarkInfoDim));

    for(unsigned int i =0; i< number_of_landmarks ; i++)
    {
        colvec candidate;

        this->findCorrespondingLandmark(state,
        z.subvec(i*singleObservationDim,i*singleObservationDim+3), candidate);

        double range = candidate(1);//norm( landmarks_[indx].subvec(1,2) - xVec.subvec(0,1) , 2);

        noise.subvec(3*i, 3*i+2) = this->etaD_*range + this->sigma_;

    }

    //square the factors to get the covariances
    noise = pow(noise,2);

    //return covariance matrix generated from vector of covariances
    return diagmat(noise);

}


typename CarrotObservationModel::ObservationType
CarrotObservationModel::computeInnovation(const ompl::base::State *predictedState, const ObservationType& Zg)
{
    using namespace arma;

    colvec xPrd = predictedState->as<CarrotBeliefSpace::StateType>()->getArmaData();

    //return the discrepancy between the expected observation
    //for a predicted state and the actual observation generated

    ObservationType Zprd = getObservationPrediction(predictedState, Zg);

    if(Zprd.n_rows == 0)
    {
        ObservationType innov;

        return innov;
    }

    ObservationType innov( (landmarkInfoDim)* Zg.n_rows /singleObservationDim ) ;

    assert( Zg.n_rows == Zprd.n_rows);

    for(unsigned int i =0; i< Zg.n_rows/singleObservationDim ; i++)
    {

        assert(Zg(i*singleObservationDim) == Zprd(i*singleObservationDim)) ;

        innov( i*(landmarkInfoDim) ) = Zg(i*singleObservationDim + 1) - Zprd(i*singleObservationDim + 1) ;

        double delta_bearing = Zg(i*singleObservationDim + 2) - Zprd(i*singleObservationDim + 2) ;

        FIRMUtils::normalizeAngleToPiRange(delta_bearing);

        innov( i*(landmarkInfoDim) + 1 ) =  delta_bearing;

        double delta_pitch = Zg(i*singleObservationDim + 3) - Zprd(i*singleObservationDim + 3) ;

        FIRMUtils::normalizeAngleToPiRange(delta_pitch);

        innov( i*(landmarkInfoDim) + 2) =  delta_pitch;

        assert(abs(delta_pitch) <= boost::math::constants::pi<double>() );

    }

    return innov;

}


typename CarrotObservationModel::ObservationType CarrotObservationModel::removeSpuriousObservations(const ObservationType& Zg)
{


    ObservationType Zcorrected;

    int counter  = 0;
    for(unsigned int i=0; i < Zg.n_rows / singleObservationDim ; i++)
    {

        for(unsigned int j=0; j < landmarks_.size() ; j++)
        {

            if(Zg(i*singleObservationDim) == landmarks_[j](0))
            {

                Zcorrected.resize(singleObservationDim*(counter +1));

                Zcorrected(singleObservationDim*counter) =    Zg(i*singleObservationDim);

                Zcorrected(singleObservationDim*counter+1) =  Zg(i*singleObservationDim+1);

                Zcorrected(singleObservationDim*counter+2) =  Zg(i*singleObservationDim+2);

                Zcorrected(singleObservationDim*counter+3) =  Zg(i*singleObservationDim+3);

                counter++;
                break;
            }

        }

    }

    return Zcorrected;

}


void CarrotObservationModel::loadLandmarks(const char *pathToSetupFile)
{
  using namespace arma;
  // Load XML containing landmarks
  TiXmlDocument doc(pathToSetupFile);
  bool loadOkay = doc.LoadFile();

  if ( !loadOkay )
  {
    printf( "Could not load Landmark list . Error='%s'. Exiting.\n", doc.ErrorDesc() );

    exit( 1 );
  }

  TiXmlNode* node = 0;
  TiXmlElement* landmarkElement = 0;
  TiXmlElement* itemElement = 0;

  // Get the landmarklist node
  node = doc.FirstChild( "LandmarkList" );
  assert( node );
  landmarkElement = node->ToElement(); //convert node to element
  assert( landmarkElement  );

  TiXmlNode* child = 0;

  std::ofstream landmarks;
  remove("landmarks.txt");
  landmarks.open("landmarks.txt", ios::app);

  //Iterate through all the landmarks and put them into the "landmarks_" list
  while( (child = landmarkElement ->IterateChildren(child)))
  {
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );

    ObservationType landmark(singleObservationDim);
    landmark.zeros();
    double attributeVal;
    itemElement->QueryDoubleAttribute("id", &attributeVal) ;
    landmark[0] = attributeVal;
    itemElement->QueryDoubleAttribute("x", &attributeVal) ;
    landmark[1] = attributeVal;
    itemElement->QueryDoubleAttribute("y", &attributeVal) ;
    landmark[2] = attributeVal;
    itemElement->QueryDoubleAttribute("z", &attributeVal) ;
    landmark[3] = attributeVal;

    landmarks << landmark[1] << "," << landmark[2] << "," << landmark[3] << "\n";

    this->landmarks_.push_back(landmark);

  }

  landmarks.close();

    OMPL_INFORM("CarrotObservationModel: Total number of landmarks loaded successfully : %u", landmarks_.size() );

    CarrotVisualizer::addLandmarks(landmarks_);
}

void CarrotObservationModel::loadParameters(const char *pathToSetupFile)
{
    using namespace arma;
    // Load XML containing landmarks
    TiXmlDocument doc(pathToSetupFile);
    bool loadOkay = doc.LoadFile();

    if ( !loadOkay )
    {
        printf( "Could not load setup file . Error='%s'. Exiting.\n", doc.ErrorDesc() );

        exit( 1 );
    }

    TiXmlNode* node = 0;

    TiXmlElement* itemElement = 0;

    // Get the landmarklist node
    node = doc.FirstChild( "ObservationModels" );
    assert( node );


    TiXmlNode* child = 0;

    child = node->FirstChild("CarrotObservationModel");
    //Iterate through all the landmarks and put them into the "landmarks_" list
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );


    double sigmaRange=0;
    double sigmaAngle=0;
    double etaRD=0;
    double etaRPhi=0;
    double etaThetaD=0;
    double etaThetaPhi=0;
    itemElement->QueryDoubleAttribute("sigma_range", &sigmaRange) ;
    itemElement->QueryDoubleAttribute("sigma_angle", &sigmaAngle) ;
    itemElement->QueryDoubleAttribute("eta_rd", &etaRD) ;
    itemElement->QueryDoubleAttribute("eta_rphi", &etaRPhi) ;
    itemElement->QueryDoubleAttribute("eta_thetad", &etaThetaD) ;
    itemElement->QueryDoubleAttribute("eta_thetaphi", &etaThetaPhi) ;

    this->sigma_ << sigmaRange << sigmaAngle * boost::math::constants::pi<double>() / 180.0 <<
        sigmaAngle * boost::math::constants::pi<double>() / 180.0 << endr;
    this->etaD_  << etaRD << etaThetaD << etaThetaD << endr;
    this->etaPhi_<< etaRPhi << etaThetaPhi << etaThetaPhi << endr;


    OMPL_INFORM("CarrotObservationModel: sigmaRange = %f", sigmaRange );

    OMPL_INFORM("CarrotObservationModel: sigma_ = ");
    std::cout<<sigma_<<std::endl;

    OMPL_INFORM("CarrotObservationModel: etaD_ = ");
    std::cout<<etaD_<<std::endl;

    OMPL_INFORM("CarrotObservationModel: etaPhi = ");
    std::cout<<etaPhi_<<std::endl;

}

bool CarrotObservationModel::isStateObservable(const ompl::base::State *state)
{
  using namespace arma;

  colvec obs = this->getObservation(state, false);

  if(obs.n_rows >= numLandmarksForObservability*singleObservationDim)
      return true;

  return false;

}
