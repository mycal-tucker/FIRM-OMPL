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
/* Author: Ali-akbar Agha-mohammadi, Saurav Agarwal */

#include "../../include/Planner/CarrotFIRM.h"
#include "ompl/geometric/planners/prm/PRM.h"
#include "ompl/geometric/planners/prm/ConnectionStrategy.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/datastructures/PDF.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/tools/config/MagicConstants.h"
#include <boost/lambda/bind.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>
#include "../../include/Visualization/CarrotVisualizer.h"
#include "../../include/Utils/FIRMUtils.h"
#include <iostream>
#include <fstream>

#define foreach BOOST_FOREACH
#define foreach_reverse BOOST_REVERSE_FOREACH

namespace ompl
{
    namespace magic
    {

        /** \brief The number of steps to take for a random bounce
            motion generated as part of the expansion step of PRM. */
        static const unsigned int MAX_RANDOM_BOUNCE_STEPS   = 5;

        /** \brief The number of nearest neighbors to consider by
            default in the construction of the PRM roadmap */
        static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 10;

        /** \brief The time in seconds for a single roadmap building operation (dt)*/
        static const double ROADMAP_BUILD_TIME = 200;

        static const double NUM_MONTE_CARLO_PARTICLES = 10;

        static const double EXTREMELY_HIGH_EDGE_COST = 1e6;

        static const double NON_OBSERVABLE_NODE_COVARIANCE = 1e2;

        static const float DYNAMIC_PROGRAMMING_DISCOUNT_FACTOR = 1;

        static const int DP_MAX_ITERATIONS = 10000;

        static const double GOAL_COST_TO_GO = 0.0;

        static const double INIT_COST_TO_GO = 2.0;

        static const double OBSTACLE_COST_TO_GO = 500;


        static const double DP_CONVERGENCE_THRESHOLD = 1e-3;

        static const double DEFAULT_NEAREST_NEIGHBOUR_RADIUS = 2.5;

        static const double KIDNAPPING_INNOVATION_CHANGE_THRESHOLD = 5.0; // 50%

        static const unsigned int MAX_MM_POLICY_LENGTH   = 1000;

        static const float MIN_ROBOT_CLEARANCE = 0.25;

        static const unsigned int MIN_STEPS_AFTER_CLEARANCE_VIOLATION_REPLANNING = 100;

        static const int STEPS_TO_ROLLOUT = 100;
    }
}

CarrotFIRM::CarrotFIRM(const firm::CarrotSpaceInformation::SpaceInformationPtr &si,
                        bool debugMode) :
    ompl::base::Planner(si, "CarrotFIRM"),
    siF_(si),
    //si_ROS_(si_ROS),
    stateProperty_(boost::get(vertex_state_t(), g_)),
    totalConnectionAttemptsProperty_(boost::get(vertex_total_connection_attempts_t(), g_)),
    successfulConnectionAttemptsProperty_(boost::get(vertex_successful_connection_attempts_t(), g_)),
    weightProperty_(boost::get(boost::edge_weight, g_)),
    edgeIDProperty_(boost::get(boost::edge_index, g_)),
    disjointSets_(boost::get(boost::vertex_rank, g_),
                  boost::get(boost::vertex_predecessor, g_)),
    maxEdgeID_(0),
    userSetConnectionStrategy_(false),
    addedSolution_(false)
{
    specs_.recognizedGoal = ompl::base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;

    Planner::declareParam<unsigned int>("max_nearest_neighbors", this, &CarrotFIRM::setMaxNearestNeighbors, std::string("8:1000"));

    minFIRMNodes_ = 15;

    //policyGenerator_ = new MMPolicyGenerator(si);

    loadedRoadmapFromFile_ = false;

}

CarrotFIRM::~CarrotFIRM(void)
{
    freeMemory();
    //delete policyGenerator_;
}

void CarrotFIRM::setup(void)
{
    Planner::setup();
    if (!nn_)
    {
        nn_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(si_->getStateSpace()));
        nn_->setDistanceFunction(boost::bind(&CarrotFIRM::distanceFunction, this, _1, _2));
    }
    if (!connectionStrategy_)
    {
        //connectionStrategy_ = ompl::geometric::KStarStrategy<Vertex>(boost::bind(&FIRM::milestoneCount, this), nn_, si_->getStateDimension());
        connectionStrategy_ = FStrategy<Vertex>(ompl::magic::DEFAULT_NEAREST_NEIGHBOUR_RADIUS, nn_);
        //connectionStrategy_ = KStrategy<Vertex>(magic::DEFAULT_NEAREST_NEIGHBORS, nn_);
    }

    int np = ompl::magic::NUM_MONTE_CARLO_PARTICLES;
    numParticles_ = np;

}

void CarrotFIRM::setMaxNearestNeighbors(unsigned int k)
{
    if (!nn_)
    {
        nn_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(si_->getStateSpace()));
        nn_->setDistanceFunction(boost::bind(&CarrotFIRM::distanceFunction, this, _1, _2));
    }
    if (!userSetConnectionStrategy_)
        connectionStrategy_.clear();
    if (isSetup())
        setup();
}

void CarrotFIRM::setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef)
{
    Planner::setProblemDefinition(pdef);
    clearQuery();
}

void CarrotFIRM::clearQuery(void)
{
    startM_.clear();
    goalM_.clear();
    pis_.restart();
}

void CarrotFIRM::clear(void)
{
    Planner::clear();
    sampler_.reset();
    simpleSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    clearQuery();
    maxEdgeID_ = 0;
}

void CarrotFIRM::freeMemory(void)
{
    foreach (Vertex v, boost::vertices(g_))
        si_->freeState(stateProperty_[v]);
    g_.clear();
}

void CarrotFIRM::growRoadmap(double growTime)
{
    growRoadmap(ompl::base::timedPlannerTerminationCondition(growTime));
}

void CarrotFIRM::growRoadmap(const ompl::base::PlannerTerminationCondition &ptc)
{
    if (!isSetup())
        setup();
    if (!sampler_)
        sampler_ = siF_->allocValidStateSampler();

    ompl::base::State *workState = siF_->allocState();
    growRoadmap (ptc, workState);
    siF_->freeState(workState);
}

void CarrotFIRM::growRoadmap(const ompl::base::PlannerTerminationCondition &ptc,
                                       ompl::base::State *workState)
{
    using namespace arma;

    while (ptc == false)
    {
        // search for a valid state
        bool found = false;
        bool stateStable = false;

        while (!found && ptc == false)
        {
            unsigned int attempts = 0;
            do
            {
                found = sampler_->sample(workState);
                stateStable = false;
                if(found)
                {

                    ompl::base::State *lsState = si_->cloneState(workState);

                    CarrotLinearSystem ls(siF_, lsState, siF_->getMotionModel()->getZeroControl(),
                                siF_->getObservationModel()->getObservation(lsState, false), siF_->getMotionModel(), siF_->getObservationModel());

                    arma::mat S;

                    try
                    {
                        stateStable = dare (trans(ls.getA()),trans(ls.getH()),ls.getG() * ls.getQ() * trans(ls.getG()),
                                ls.getM() * ls.getR() * trans(ls.getM()), S );

                        workState->as<CarrotBeliefSpace::StateType>()->setCovariance(S);
                    }
                    catch(int e)
                    {
                        stateStable = false;
                    }

                }
                attempts++;
            } while (attempts < ompl::magic::FIND_VALID_STATE_ATTEMPTS_WITHOUT_TERMINATION_CHECK && !found && !stateStable);
        }
        // add it as a milestone
        if (found && stateStable)
            addStateToGraph(si_->cloneState(workState));
	    //std::cout << "state added: " << workState->as<ompl::base::RealVectorStateSpace::StateType>()->values << std::endl;
    }
}

void CarrotFIRM::checkForSolution(const ompl::base::PlannerTerminationCondition &ptc,
                                            ompl::base::PathPtr &solution)
{
    // Don't do anything for first one second
    boost::this_thread::sleep(boost::posix_time::seconds(1));

    while (!ptc && !addedSolution_)
    {
        // Check for any new goal states
        /*if (goal->maxSampleCount() > goalM_.size())
        {
            const ompl::base::State *st = pis_.nextGoal();
            if (st)
                goalM_.push_back(addStateToGraph(si_->cloneState(st)));
        }
        */
        addedSolution_ = existsPolicy(startM_, goalM_, solution);

        if (!addedSolution_)
        {
            OMPL_INFORM("FIRM: Checking for Solution.");
            boost::this_thread::sleep(boost::posix_time::seconds(10));
        }
    }
}

bool CarrotFIRM::existsPolicy(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals, ompl::base::PathPtr &solution)
{
    ompl::base::Goal *g = pdef_->getGoal().get();
    ompl::base::Cost sol_cost(0.0);

    OMPL_INFORM("%s: Number of current %u states", getName().c_str(), boost::num_vertices(g_));

    if(boost::num_vertices(g_) < minFIRMNodes_) return false;

    foreach (Vertex start, starts)
    {
        foreach (Vertex goal, goals)
        {
            graphMutex_.lock();
            bool same_component = sameComponent(start, goal);
            graphMutex_.unlock();

            if (same_component && g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
            {
                boost::mutex::scoped_lock _(graphMutex_);
                solveDynamicProgram(goal);
                solution = constructFeedbackPath(start, goal);
                sendFeedbackEdgesToViz();
                return true; // return true if solution is found
            }
        }
    }

    return false;
}

bool CarrotFIRM::addedNewSolution(void) const
{
    return addedSolution_;
}

ompl::base::PlannerStatus CarrotFIRM::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    int numMapsToConstruct = 1; //edit value if want more
    int numMapsConstructed = 0;
    ompl::base::PathPtr sol; //added for scope
    const ompl::base::State *startState = si_->cloneState(pis_.nextStart());
    const ompl::base::State *goalState = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
    while (numMapsConstructed < numMapsToConstruct){
        ompl::base::PathPtr tempSol; //reset each time in loop


        checkValidity();
        ompl::base::GoalSampleableRegion *goal = dynamic_cast<ompl::base::GoalSampleableRegion*>(pdef_->getGoal().get());
        if (!goal)
        {
            OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
            return ompl::base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
        }
        // Add the valid start states as milestones
        /*while (const ompl::base::State *st = pis_.nextStart())
               // std::cout << st->as<ompl::base::RealVectorStateSpace::StateType>()->values << std::endl;
            startM_.push_back(addStateToGraph(si_->cloneState(st)));*/
            startM_.clear();
            addStateToGraph(si_->cloneState(startState));
            startM_.push_back(addStateToGraph(si_->cloneState(startState)));
        if (startM_.size() == 0)
        {
            OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
            return ompl::base::PlannerStatus::INVALID_START;
        }

        if (!goal->couldSample())
        {
            OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
            return ompl::base::PlannerStatus::INVALID_GOAL;
        }

        // Ensure there is at least one valid goal state
        //if (goal->maxSampleCount() > goalM_.size() || goalM_.empty())
        //{
            /*const ompl::base::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st)
                goalM_.push_back(addStateToGraph(si_->cloneState(st)));

            if (goalM_.empty())
            {
                OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
                return ompl::base::PlannerStatus::INVALID_GOAL;
            }*/
            //if (goalState){
                goalM_.push_back(addStateToGraph(si_->cloneState(goalState)));

            //}
        //}


        unsigned int nrStartStates = boost::num_vertices(g_);
        OMPL_INFORM("%s: Starting with %u states", getName().c_str(), nrStartStates);

        addedSolution_ = false;

        if (!isSetup())
            setup();
        if (!sampler_)
            sampler_ = si_->allocValidStateSampler();
        if (!simpleSampler_)
            simpleSampler_ = si_->allocStateSampler();

        //ompl::base::PathPtr sol; //commented out by Mycal
        boost::thread slnThread(boost::bind(&CarrotFIRM::checkForSolution, this, ptc, boost::ref(tempSol)));

        ompl::base::PlannerTerminationCondition ptcOrSolutionFound =
            ompl::base::plannerOrTerminationCondition(ptc, ompl::base::PlannerTerminationCondition(boost::bind(&CarrotFIRM::addedNewSolution, this)));

        // If no roadmap was loaded, then construct one
        if(!loadedRoadmapFromFile_ && numMapsConstructed < numMapsToConstruct) //mycal added the and
        {
            constructRoadmap(ptcOrSolutionFound);
        }

        slnThread.join();

        OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(g_) - nrStartStates);

        if (tempSol)
        {
            ompl::base::PlannerSolution psol(tempSol);
            // if the solution was optimized, we mark it as such
            if (addedNewSolution())
                psol.optimized_ = true;
            pdef_->addSolutionPath(psol);

            // If roadmap wasn't loaded from file, then save the newly constructed roadmap
            if(!loadedRoadmapFromFile_)
            {
                if (numMapsToConstruct != 1){
                    this->savePlannerData(numMapsConstructed+2); //this will always number the maps starting at 2. That way it won't reload from FIRMRoadMap in later loops
                } else{
                    this->savePlannerData(); //saves to FIRMRoadMap.xml
                }
            } else {
                sol = tempSol;
            }
        }
        if (numMapsConstructed == numMapsToConstruct - 1){
            sol = tempSol;
        }

        numMapsConstructed ++;

        //wipe nn_ clear
        nn_->clear();
        //wipe the graph clear
        g_.clear();
    }


    return sol ? (addedNewSolution() ? ompl::base::PlannerStatus::EXACT_SOLUTION : ompl::base::PlannerStatus::APPROXIMATE_SOLUTION) : ompl::base::PlannerStatus::TIMEOUT;
}

void CarrotFIRM::constructRoadmap(const ompl::base::PlannerTerminationCondition &ptc)
{

    std::vector<ompl::base::State*> xstates(ompl::magic::MAX_RANDOM_BOUNCE_STEPS);
    si_->allocStates(xstates);

    while (ptc() == false)
    {
        growRoadmap(ompl::base::plannerOrTerminationCondition(ptc, ompl::base::timedPlannerTerminationCondition(ompl::magic::ROADMAP_BUILD_TIME)), xstates[0]);
    }

    si_->freeStates(xstates);
}

CarrotFIRM::Vertex CarrotFIRM::addStateToGraph(ompl::base::State *state, bool addReverseEdge, bool shouldCreateNodeController)
{

    boost::mutex::scoped_lock _(graphMutex_);

    Vertex m;

    //if state already exists in the graph, just return corresponding vertex
    /*
    if(isDuplicateState(state,m))
    {
        return m;
    }
    */

    m = boost::add_vertex(g_);
    addStateToVisualization(state);
    stateProperty_[m] = state;
    NodeControllerType nodeController;
    generateNodeController(state, nodeController); // Generate the node controller
    nodeControllers_[m] = nodeController; // Add it to the list

    totalConnectionAttemptsProperty_[m] = 1;
    successfulConnectionAttemptsProperty_[m] = 0;

    // Initialize to its own (dis)connected component.
    disjointSets_.make_set(m);
    nn_->add(m);

    // Which milestones will we attempt to connect to?
    const std::vector<Vertex>& neighbors = connectionStrategy_(m);
    foreach (Vertex n, neighbors)
    {
        if ( m!=n )
        {
            totalConnectionAttemptsProperty_[m]++;
            totalConnectionAttemptsProperty_[n]++;
	    // convert x,y,z to SE3 (assume no angular displacements)
    	   // ompl::base::StateSpacePtr si(new ompl::base::SE3StateSpace());
	    /*ompl::base::State* startV = si->allocState();
	    ompl::base::State* endV = si->allocState();
            startV->setXYZ(stateProperty_[m]->as<CarrotBeliefSpace::StateType>()->getX(),
		stateProperty_[m]->as<CarrotBeliefSpace::StateType>()->getY(),
		stateProperty_[m]->as<CarrotBeliefSpace::SpaceType>()->getZ() );
            endV->setXYZ(stateProperty_[n]->as<CarrotBeliefSpace::StateType>()->getX(),
		stateProperty_[n]->as<CarrotBeliefSpace::StateType>()->getY(),
		stateProperty_[n]->as<CarrotBeliefSpace::StateType>()->getZ() );
	    ompl::base::SO3StateSpace::StateType &rot_start = startV->rotation();
	    ompl::base::SO3StateSpace::StateType &rot_end = endV->rotation();
	    rot_start.x = 1; rot_start.y = 0; rot_start.z = 0; rot_start.w = 0;
	    rot_end.x = 1; rot_end.y = 0; rot_end.z = 0; rot_end.w = 0;
	    si->printState(startV);
	    si->printState(endV);*/
        if (si_->checkMotion(stateProperty_[m],stateProperty_[n]))
            {
                successfulConnectionAttemptsProperty_[m]++;
                successfulConnectionAttemptsProperty_[n]++;

                addEdgeToGraph(m, n);

                if(addReverseEdge)
                {
                    addEdgeToGraph(n, m);
                }

                uniteComponents(m, n);
            }
	    //si->freeState(start);
	    //si->freeState(end);
        }

    }
//    policyGenerator_->addFIRMNodeToObservationGraph(state);

    return m;

}

void CarrotFIRM::uniteComponents(Vertex m1, Vertex m2)
{
    disjointSets_.union_set(m1, m2);
}

bool CarrotFIRM::sameComponent(Vertex m1, Vertex m2)
{
    return boost::same_component(m1, m2, disjointSets_);
}

ompl::base::PathPtr CarrotFIRM::constructFeedbackPath(const Vertex &start, const Vertex &goal)
{
    CarrotFeedbackPath *p = new CarrotFeedbackPath(siF_);

    //std::cout<<"The start vertex is: "<<start->as<CarrotBeliefSpace::StateType>()->getArmaData()<<std::endl;
    //std::cout<<"The goal vertex is: "<<goal->as<CarrotBeliefSpace::StateType>()->getArmaData()<<std::endl;

    Vertex currentVertex = start;
    std::cout << "Start state: " << stateProperty_[start]->as<CarrotBeliefSpace::StateType>()->getArmaData() << std::endl;

    while(currentVertex!=goal)
    {
        Edge edge = feedback_[currentVertex]; // get the edge
        Vertex target = boost::target(edge, g_); // get the target of this edge
        //std::cout << "Target state: " << stateProperty_[target]->as<CarrotBeliefSpace::StateType>()->getArmaData() << std::endl;

        p->append(stateProperty_[currentVertex],edgeControllers_[edge]); // push the state and controller to take
        if(target == goal)
        {

            p->append(stateProperty_[target]); // because from the goal node you don't need to take a controller
        }
        currentVertex =  target;
    }

    return ompl::base::PathPtr(p);
}

void CarrotFIRM::addEdgeToGraph(const CarrotFIRM::Vertex a, const CarrotFIRM::Vertex b)
{

    EdgeControllerType edgeController;

    const FIRMWeight weight = generateEdgeControllerWithCost(a, b, edgeController);

    assert(edgeController.getGoal() && "The generated controller has no goal");

    const unsigned int id = maxEdgeID_++;

    const Graph::edge_property_type properties(weight, id);

    // create an edge with the edge weight property
    std::pair<Edge, bool> newEdge = boost::add_edge(a, b, properties, g_);

    edgeControllers_[newEdge.first] = edgeController;

    CarrotVisualizer::addGraphEdge(stateProperty_[a], stateProperty_[b]);
}

FIRMWeight CarrotFIRM::generateEdgeControllerWithCost(const CarrotFIRM::Vertex a, const CarrotFIRM::Vertex b, EdgeControllerType &edgeController)
{
    ompl::base::State* startNodeState = siF_->cloneState(stateProperty_[a]);
    ompl::base::State* targetNodeState = siF_->cloneState(stateProperty_[b]);

     // Generate the edge controller for given start and end state
    generateEdgeController(startNodeState,targetNodeState,edgeController);

    // If there exists an edge corresponding to this in the loaded file we use that
    // otherwise evaluate the weight properties
    if(loadedRoadmapFromFile_)
    {
         // find the matching loaded edge and then return its weight
        for(int i=0; i < loadedEdgeProperties_.size(); i++)
        {
            if(loadedEdgeProperties_[i].first.first == a && loadedEdgeProperties_[i].first.second == b)
            {
                return loadedEdgeProperties_[i].second;
            }
        }

    }


    double successCount = 0;

    // initialize costs to 0
    ompl::base::Cost edgeCost(0);
    ompl::base::Cost nodeStabilizationCost(0);

    for(unsigned int i=0; i< numParticles_;i++)
    {
        //std::cout << "MonteCarlo Simulation particle number "<< i<<std::endl;
        siF_->setTrueState(startNodeState);
        siF_->setBelief(startNodeState);

        ompl::base::State* endBelief = siF_->allocState(); // allocate the end state of the controller
        ompl::base::Cost pcost(0);

        if(edgeController.Execute(startNodeState, endBelief, pcost))
        {
           successCount++;

           edgeCost = ompl::base::Cost(edgeCost.value() + pcost.value() );
	//std::cout << "edge cost: " << edgeCost.value() << std::endl;

        }

    }

    if (successCount > 0)  edgeCost = ompl::base::Cost(edgeCost.value() / successCount );
    else edgeCost = ompl::base::Cost(ompl::magic::EXTREMELY_HIGH_EDGE_COST); // extremely high cost if no particle could succeed, we can also simply not add this edge

    double transitionProbability = successCount / numParticles_ ;

    FIRMWeight weight(edgeCost.value(), transitionProbability);

    return weight;
}


void CarrotFIRM::generateEdgeController(const ompl::base::State *start, const ompl::base::State* target, CarrotFIRM::EdgeControllerType &edgeController)
{
    assert(target && "The target state for generating the edge controller is NULL");
    std::vector<ompl::base::State*> intermediates;

    ompl::base::State *intermediate = si_->allocState();

    si_->copyState(intermediate, start);

    std::vector<ompl::control::Control*> openLoopControls;

    // get the open loop controls for this edge
    siF_->getMotionModel()->generateOpenLoopControls(start, target, openLoopControls);

    // generate the intermediate states using the open loop controls
    for(typename std::vector<ompl::control::Control*>::iterator c=openLoopControls.begin(), e=openLoopControls.end(); c!=e; ++c)
    {
        ompl::base::State *x = si_->allocState();
        siF_->getMotionModel()->Evolve(intermediate,*c,siF_->getMotionModel()->getZeroNoise(), x);
        intermediates.push_back(x);
        si_->copyState(intermediate, x);
    }

    // create the edge controller
    EdgeControllerType ctrlr(target, intermediates, openLoopControls, siF_);

    // assign the edge controller
    edgeController =  ctrlr;
}

void CarrotFIRM::generateNodeController(const ompl::base::State *state, CarrotFIRM::NodeControllerType &nodeController)
{
    // Create a copy of the node state
    ompl::base::State *node = si_->allocState();
    siF_->copyState(node, state);

   if(siF_->getObservationModel()->isStateObservable(node))
   {
        // Contruct a linear kalman filter
        CarrotLinearizedKF linearizedKF(siF_);

        //Construct a linear system
        CarrotLinearSystem linearSystem(siF_, node, siF_->getMotionModel()->getZeroControl(),siF_->getObservationModel()->getObservation(state, false), siF_->getMotionModel(), siF_->getObservationModel());

        // Compute the stationary cov at node state using LKF
        arma::mat stationaryCovariance = linearizedKF.computeStationaryCovariance(linearSystem);

        // set the covariance
        node->as<CarrotBeliefSpace::StateType>()->setCovariance(stationaryCovariance);

        // create a node controller
        std::vector<ompl::control::Control*> dummyControl;
        std::vector<ompl::base::State*> dummyStates;
        NodeControllerType ctrlr(node, dummyStates, dummyControl, siF_);

	/*std::cout << "generating node controller at: " <<
	node->as<CarrotBeliefSpace::StateType>()->getArmaData() <<
	" with covariance: " << stationaryCovariance << std::endl;*/

        // assign the node controller
        nodeController = ctrlr;
    }

    else
    {
        // Compute a high stationary cov at node state
        int stateDim = si_->getStateDimension();
        arma::mat stationaryCovariance = arma::eye(stateDim,stateDim)*ompl::magic::NON_OBSERVABLE_NODE_COVARIANCE;

        // create a node controller
        std::vector<ompl::control::Control*> dummyControl;
        std::vector<ompl::base::State*> dummyStates;
        NodeControllerType ctrlr(node, dummyStates, dummyControl, siF_);

        // assign the node controller
        nodeController = ctrlr;
    }

}

template<typename Map>
arma::colvec MapToColvec(const Map& _m) {
  arma::colvec mapData(_m.size());
  int ix = 0;
  for(typename Map::const_iterator i= _m.begin(), e= _m.end(); i!=e; ++i, ++ix)
    mapData[ix] = i->second;
  return mapData;
}

void CarrotFIRM::solveDynamicProgram(const CarrotFIRM::Vertex goalVertex)
{
    OMPL_INFORM("FIRM: Solving DP");

    using namespace arma;

    float discountFactor = ompl::magic::DYNAMIC_PROGRAMMING_DISCOUNT_FACTOR;

    std::map<Vertex, double> newCostToGo;

    costToGo_ = std::map<Vertex, double>();

    /**
    --NOTES--
    Assign a high cost to go initially for all nodes that are not in the goal connected component.
    For nodes that are in the goal cc, we assign goal cost to go for the goal and init cost to go
    for all other nodes.
    */
    foreach (Vertex v, boost::vertices(g_))
    {
        if(boost::out_degree(v,g_) > 0 )
        {

            if(v == goalVertex)
            {
                costToGo_[v] = ompl::magic::GOAL_COST_TO_GO;
                newCostToGo[v] = ompl::magic::GOAL_COST_TO_GO;
            }
            else
            {
                costToGo_[v] = ompl::magic::INIT_COST_TO_GO;
                newCostToGo[v] = ompl::magic::INIT_COST_TO_GO;
            }
        }
    }

    feedback_.clear();

    bool convergenceCondition = false;

    int nIter=0;
    while(!convergenceCondition && nIter < ompl::magic::DP_MAX_ITERATIONS)
    {
        nIter++;

        foreach(Vertex v, boost::vertices(g_))
        {
            //value for goal node stays the same or if has no out edges then ignore it
            if( v == goalVertex || boost::out_degree(v,g_) < 1 )
            {
                continue;
            }

            // Update the costToGo of vertex
            std::pair<Edge,double> candidate = getUpdatedNodeCostToGo(v);

            feedback_[v] = candidate.first;

            newCostToGo[v] = candidate.second * discountFactor;
        }

        convergenceCondition = (norm(MapToColvec(costToGo_)-MapToColvec(newCostToGo), "inf") <= ompl::magic::DP_CONVERGENCE_THRESHOLD);

        costToGo_.swap(newCostToGo);   // Equivalent to costToGo_ = newCostToGo

    }

    OMPL_INFORM("FIRM: Solved DP");

    sendFeedbackEdgesToViz();

    CarrotVisualizer::setMode(CarrotVisualizer::VZRDrawingMode::FeedbackViewMode);

}


struct DoubleValueComp
{
  template<typename KVP1, typename KVP2>
  bool operator()(const KVP1& kvp1, const KVP2& kvp2) const
  {
    return kvp1.second < kvp2.second;
  }
};


std::pair<typename CarrotFIRM::Edge,double> CarrotFIRM::getUpdatedNodeCostToGo(const CarrotFIRM::Vertex node)
{

    std::map<Edge,double > candidateCostToGo;

    foreach(Edge e, boost::out_edges(node, g_))
    {
        // the target of given edge "e"
        const Vertex targetNode = boost::target(e, g_);

        double nextNodeCostToGo = costToGo_[targetNode];

        const FIRMWeight edgeWeight =  boost::get(boost::edge_weight, g_, e);

        const double transitionProbability  = edgeWeight.getSuccessProbability();

        double singleCostToGo = ( transitionProbability*nextNodeCostToGo + (1-transitionProbability)*ompl::magic::OBSTACLE_COST_TO_GO) + edgeWeight.getCost();

        candidateCostToGo[e] =  singleCostToGo ;

    }

    DoubleValueComp dvc;
    std::pair<Edge,double> bestCandidate =   *min_element(candidateCostToGo.begin(), candidateCostToGo.end(), dvc );

    return bestCandidate;

}

void CarrotFIRM::execute(bool prm)
{
    if (prm) {
        this->executePRMPath();
    } else {
        this->executeFeedback();
    }
}

void CarrotFIRM::executePRMPath(void)
{
    /*
    0) Build graph prmG with all nodes and edges we want
    1) Solve all shortest paths from start
    2) Find shortest path from start to goal
    3) Execute path along waypoints from start to goal (TODO)
    */
    struct PRMNode{int id; float x; float y; float z;};
    struct PRMEdge{int id1; int id2; float cost;};

    typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS, PRMNode, PRMEdge> PRMGraph;
    typedef boost::graph_traits<PRMGraph>::vertex_descriptor vertex_t;
    typedef boost::graph_traits<PRMGraph>::edge_descriptor edge_t;
    typedef boost::adjacency_list<>::vertex_iterator vertex_iter;
    typedef boost::property_map<PRMGraph, boost::vertex_index_t>::type IndexMap;
    typedef boost::graph_traits<PRMGraph> Traits;
    typedef std::pair<int, int> Edge;


    PRMGraph prmG;
    std::pair<vertex_iter, vertex_iter> vs;// = boost::vertices(prmG);
    IndexMap index; //=boost::get(vertex_index, prmG);
    vertex_t start; //where to start from
    vertex_t goal; //where to end
    std::vector<float> weightsVec;
    std::vector<Edge> edgesVec;

    //Only from FIRMRoadMap.xml
    std::string line;
    std::ifstream firmMap("FIRMRoadMap.xml");
    bool inNodes = true; //whether or not we are parsing nodes (instead of edges)
    if (firmMap.is_open())
    {
        while (std::getline(firmMap, line))
        {
            if (inNodes)
            {
                if (line.compare("<?xml version=\"1.0\" ?>") == 0)
                {
                    //ignore the first line
                }
                else if (line.compare("<Nodes>") == 0)
                {
                    //start parsign nodes; ignore this one line
                }
                else if (line.compare("</Nodes>") == 0)
                {
                    //done parsing nodes
                    inNodes = false;
                    vs = boost::vertices(prmG);
                    index = boost::get(boost::vertex_index, prmG);
                }
                else
                {
                    //extract id, x, y, z
                    int startOfId = line.find("id") + 4;
                    int endOfId = line.find("\"", startOfId);
                    std::string idString = line.substr(startOfId, endOfId-startOfId);
                    int id;
                    std::stringstream convert(idString);
                    if (!(convert >> id)){ id = -1;}
                    //std::cout<<"got id: "<<id<<std::endl;
                    int startOfX = line.find("x") + 3;
                    int endOfX = line.find("\"", startOfX);
                    std::string xString = line.substr(startOfX, endOfX-startOfX);
                    float x = std::stof(xString, nullptr);
                    //std::cout<<"got x: "<<x<<std::endl;
                    int startOfY = line.find("y") + 3;
                    int endOfY = line.find("\"", startOfY);
                    std::string yString = line.substr(startOfY, endOfY-startOfY);
                    float y = std::stof(yString, nullptr);
                    //std::cout<<"got y: "<<y<<std::endl;
                    int startOfZ = line.find("theta") + 7;
                    int endOfZ = line.find("\"", startOfZ);
                    std::string zString = line.substr(startOfZ, endOfZ-startOfZ);
                    float z = std::stof(zString, nullptr);
                    //std::cout<<"got z: "<<z<<std::endl;

                    vertex_t temp = boost::add_vertex(prmG);

                    prmG[temp].id = id;
                    prmG[temp].x = x;
                    prmG[temp].y = y;
                    prmG[temp].z = z;

                    if (id == 0) //start node
                    {
                        start = temp;
                    }
                    else if (id == 2)//TODO this is just because roadmap gets built weirdly
                    {
                        goal = temp;
                    }
                }
            }
            else //if not inNodes, must be parsing edges instead
            {
                if (line.compare("<Edges>") == 0 || line.compare("</Edges>") == 0)
                {
                    //ignore start and end marker
                }
                else
                {
                    int startOfStartVertex = line.find("startVertexID") + 15;
                    int endOfStartVertex = line.find("\"", startOfStartVertex);
                    std::string startVertexIdString = line.substr(startOfStartVertex, endOfStartVertex-startOfStartVertex);
                    int startId;
                    std::stringstream convert(startVertexIdString);
                    if (!(convert >> startId)){ startId = -1;}
                    //std::cout<<"start edgeId: "<<startId<<std::endl;
                    int startOfEndVertex = line.find("endVertexID") + 13;
                    int endOfEndVertex = line.find("\"", startOfEndVertex);
                    std::string endVertexIdString = line.substr(startOfEndVertex, endOfEndVertex-startOfEndVertex);
                    int endId;
                    std::stringstream convert2(endVertexIdString);
                    if (!(convert2 >> endId)){ endId = -1;}
                    //std::cout<<"end edgeId: "<<endId<<std::endl;


                    vertex_t u;//the vertex corresponding to start of edge
                    vertex_t v;//the vertex corresponding to end of edge
                    for (vs = boost::vertices(prmG); vs.first != vs.second; ++vs.first)
                    {
                        vertex_t tempV = boost::vertex(index[*vs.first], prmG);
                        int tempId = prmG[tempV].id;
                        if (tempId == startId) {u = tempV;}
                        else if (tempId == endId) {v = tempV;}
                    }
                    edge_t e; bool b;
                    boost::tie(e, b) = boost::add_edge(u, v, prmG);
                    float weight = sqrt((prmG[u].x - prmG[v].x)*(prmG[u].x - prmG[v].x) + (prmG[u].y - prmG[v].y)*(prmG[u].y - prmG[v].y) + (prmG[u].z - prmG[v].z)*(prmG[u].z - prmG[v].z));
                    weightsVec.push_back(weight);
                    edgesVec.push_back(Edge(prmG[u].id, prmG[v].id));
                    prmG[e].id1 = startId;
                    prmG[e].id2 = endId;
                    //cost is euclidean distance between endpoints
                    prmG[e].cost = weight;
                }
            }
            //std::cout<<line<<std::endl;
            //std::cout<<"vertices in graph: "<<boost::num_vertices(prmG)<<std::endl;
            //std::cout<<"edges in graph: "<<boost::num_edges(prmG)<<std::endl;
        }
        firmMap.close();
    }
    else
    {
        std::cout<<"unable to open FIRMRoadMap.xml"<<std::endl;
    }

    const int num_nodes = boost::num_vertices(prmG);
    //float* weights = &weightsVec[0];
    //TODO review with Chris that this makes sense.
    //I need weights to be an array of ints, not floats, so I multiply by 1000 and then round.
    //Multiplying by a bigger number would reduce error risk
    int weights[weightsVec.size()];
    for (int i = 0; i < weightsVec.size(); ++i)
    {
        weights[i] = std::floor(weightsVec[i]*1000);
    }

    std::vector<vertex_t> parents(boost::num_vertices(prmG)); //will be used to store predecessor map
    std::vector<double> distances(boost::num_vertices(prmG)); //will be used to store distances map

    auto p_map = boost::make_iterator_property_map(&parents[0], boost::get(boost::vertex_index, prmG));

    boost::dijkstra_shortest_paths(prmG, start,
                                    boost::weight_map(boost::get(&PRMEdge::cost, prmG))
                                    .predecessor_map(p_map)
                                    .distance_map(boost::make_iterator_property_map(distances.begin(),
                                            boost::get(boost::vertex_index, prmG))));

    //Now let's find the shortest path from start to goal
    std::vector<vertex_t> path; //note that this will be backwards (so it goes along edges backwards from goal to start)

    vertex_t v = goal;
    for (vertex_t u = p_map[v];
        u != v;
        v = u, u = p_map[v])
    {
        path.push_back(v);
    }

    std::reverse(path.begin(), path.end()); //now in the right order

    std::cout<<"Printing path"<<std::endl;
    for (int i = 0; i < path.size(); ++i){
    std::cout<<path[i]<<std::endl;
    }

    //execute the shortest path

    std::ofstream myfile;
    remove("loggingPRM.txt");//remove the log from the last prm trial
    myfile.open("loggingPRM.txt", std::ios::app);

    bool simulation = false;
    siF_->setSimulation(simulation);
    siF_->initializeSubscriber();
    siF_->initializePublisher();
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    //already have vertex_t start declared and set way before
    vertex_t currentLocation = start;
    vertex_t nextWaypoint = path[0];

    double goalX = prmG[goal].x;
    double goalY = prmG[goal].y;
    double goalZ = prmG[goal].z;
    float distToGoal = sqrt((prmG[start].x - goalX)*(prmG[start].x - goalX) + (prmG[start].y - goalY)*(prmG[start].y - goalY) + (prmG[start].z - goalZ)*(prmG[start].z - goalZ));
    //How close to goal is close enough
    float nearThreshold = 0.1;

    //where to fly next
    double wayX = prmG[nextWaypoint].x;
    double wayY = prmG[nextWaypoint].y;
    double wayZ = prmG[nextWaypoint].z;

    //keep track of old waypoint to compute straight line path
    double oldWayX = wayX;
    double oldWayY = wayY;
    double oldWayZ = wayZ;


    siF_->setTrueState(stateProperty_[startM_[0]]);

    //say where we are starting
    std::vector<double> startingPose = siF_->getQuadLocation();
    double startX = startingPose[0];
    double startY = startingPose[1];
    double startZ = startingPose[2];
    std::cout<<"Starting position: x: "<<startX<<", y: "<<startY<<", z: "<<startZ<<std::endl;

    double currX = startX;
    double currY = startY;
    double currZ = startZ;

    double controllerGain = 0.001;
    int pathIndex = 0;
    int loopNumber = 1;
    siF_->flyToWaypoint(wayX, wayY, wayZ, true, 1); //send first command
    while (distToGoal > nearThreshold) //until we are pretty close to goal
    {
        /*
        1. Try to go straight to next waypoint (send waypoint command via ROS)
        2. Update currentLocation (from ROS)
        3. Update distToGoal
        4. If close to waypoint:
            4.1 Increment pathIndex up by 1
            4.2 Get new waypoint target
        */
        ros::spinOnce();
        //std::vector<double> curr = siF_->flyAlongVector(wayX-currX, wayY-currY, wayZ-currZ);
        std::vector<double> curr = siF_->flyToWaypoint(wayX, wayY, wayZ, false, loopNumber);
        loopNumber = loopNumber + 1;
        currX = curr[0];
        currY = curr[1];
        currZ = curr[2];

        myfile<<currX<<", "<<currY<<", "<<currZ<<"\n"; //only logs current location, not desired location

        //std::cout<<"Current position: x: "<<currX<<", y: "<<currY<<", z: "<<currZ<<std::endl;

        //check if quad true state has deviated from plan enough to no longer be in a valid state
        if (!(siF_->checkTrueStateValidity()))
        {
            std::cout<<"PRM true state not valid"<<std::endl;
        }

        //update distance to goal for big while-loop
        distToGoal = sqrt((goalX-currX)*(goalX-currX) + (goalY-currY)*(goalY-currY) + (goalZ-currZ)*(goalZ-currZ));

        //see if need to update to next waypoint
        float distToWaypoint = sqrt((wayX-currX)*(wayX-currX) + (wayY-currY)*(wayY-currY) + (wayZ-currZ)*(wayZ-currZ));
        if (distToWaypoint < nearThreshold)
        {
            std::cout<<"Reached waypoint: x: "<<wayX<<", y: "<<wayY<<", z: "<<wayZ<<std::endl;
            //close enough; aim for next waypoint (but check that won't get index out of bounds if there is no next waypoint
            if (pathIndex == path.size()){/*close to goal, will exit while loop right away*/}
            else
            {
                pathIndex = pathIndex + 1;
                nextWaypoint = path[pathIndex];
                //tell quad to fly to next waypoint
                oldWayX = wayX;
                oldWayY = wayY;
                oldWayZ = wayZ;
                wayX = prmG[nextWaypoint].x;
                wayY = prmG[nextWaypoint].y;
                wayZ = prmG[nextWaypoint].z;
            }
        }
    }
    myfile.close();
    std::cout<<"Reached goal"<<std::endl;
}

void CarrotFIRM::executeFeedback(void)
{

    siF_->setSimulation(false); // running in hardware now
    siF_->initializeSubscriber();
    siF_->initializePublisher();
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    Vertex start = startM_[0];
    Vertex goal  = goalM_[0] ;

    siF_->setTrueState(stateProperty_[start]);
    siF_->setBelief(stateProperty_[start]);

    Vertex currentVertex =  start;

    EdgeControllerType controller;

    ompl::base::State *cstartState = si_->allocState();
    si_->copyState(cstartState, stateProperty_[start]);

    ompl::base::State *cendState = si_->allocState();

    std::cout << "[CarrotFIRM.cpp] Uninitialized start State" << cendState->as<CarrotBeliefSpace::StateType>()->getArmaData() << std::endl;

    OMPL_INFORM("CarrotFIRM: Running policy execution");

    bool kidnapped_flag = false;

    int kidnappingCounter  = 0;

    while(currentVertex != goal)
    {

        Edge e = feedback_[currentVertex];
        controller = edgeControllers_[e];
        ompl::base::Cost cost;
        std::cout << "edge info: " << e << std::endl;
        //std::cout << "Execute cstartState: " << cstartState << " cendState: " << cendState << std::endl;

        if(controller.Execute(cstartState, cendState, cost, false))
        {
            currentVertex = boost::target(e, g_);
            std::cout << "Updated currentVertex: " << currentVertex << std::endl;

        }

        else
        {
            std::cout << "[CarrotFIRM.cpp] Could not execute edge..." << std::endl;
            // get a copy of the true state
            ompl::base::State *tempTrueStateCopy = si_->allocState();
            //ompl::base::State *tempROSTrueStateCopy = si_ROS_->allocState();


            siF_->getTrueState(tempTrueStateCopy);
            //si_ROS_->getTrueState(tempROSTrueStateCopy);

            int numVerticesBefore = boost::num_vertices(g_);

            currentVertex = addStateToGraph(cendState);

            int numVerticesAfter = boost::num_vertices(g_);

            // Set true state back to its correct value after Monte Carlo (happens during adding state to Graph)
            siF_->setTrueState(tempTrueStateCopy);


//            std::cout << "true state of deviation: " << tempTrueStateCopy->as<CarrotMotionModel::StateType>()->getArmaData() << std::endl;

            siF_->freeState(tempTrueStateCopy);
            //si_ROS_->freeState(tempROSTrueStateCopy);

            assert(numVerticesAfter-numVerticesBefore >0);

            solveDynamicProgram(goal);


        }
        si_->copyState(cstartState, cendState);
    }
}

// Experimental
void CarrotFIRM::executeFeedbackWithRollout(void)
{
    sendFeedbackEdgesToViz();
    std::cout << "In executeFeedbackWithRollout" << std::endl;

    const Vertex start = startM_[0];
    const Vertex goal  = goalM_[0] ;

    ompl::base::State *goalState = si_->cloneState(stateProperty_[goal]);

    siF_->setTrueState(stateProperty_[start]);
    siF_->setBelief(stateProperty_[start]);

    Vertex currentVertex =  start;

    EdgeControllerType controller;

    ompl::base::State *cstartState = si_->allocState();
    si_->copyState(cstartState, stateProperty_[start]);

    ompl::base::State *cendState = si_->allocState();

    OMPL_INFORM("FIRM: Running policy execution");

    Edge e = feedback_[currentVertex];

    Vertex tempVertex;

    OMPL_INFORM("Goal State is: \n");
    si_->printState(goalState);

    // While the robot state hasn't reached the goal state, keep running
    while(!goalState->as<CarrotBeliefSpace::StateType>()->isReached(cstartState) /*si_->distance(stateProperty_[currentVertex], stateProperty_[goal]) > 0.5*/)
    {
        //Edge e = feedback_[currentVertex];
        //Vertex targetNode = boost::target(e, g_);

        double dist = si_->distance(stateProperty_[currentVertex], stateProperty_[goal]);
        std::cout << "[CarrotFIRM.cpp] Distance to goal: " << dist << std::endl;

        controller = edgeControllers_[e];

        ompl::base::Cost cost;

        /**
            Instead of executing the entire controller, we need to
            execute one step, then calculate the cost to go through the neighboring nodes.
            Whichever gives the lowest cost to go, is our new path.
            Do this at each step.
        */
        controller.executeUpto(ompl::magic::STEPS_TO_ROLLOUT,cstartState,cendState,cost,false);

        ompl::base::State *tState = si_->allocState();

        siF_->getTrueState(tState);

        tempVertex = addStateToGraph(cendState, false);

        siF_->setTrueState(tState);

        si_->freeState(tState);

        e = generateRolloutPolicy(tempVertex);

        boost::remove_vertex(tempVertex, g_);

        sendFeedbackEdgesToViz();

        si_->copyState(cstartState, cendState);

        OMPL_INFORM("Goal State is: \n");
        si_->printState(goalState);

    }


}

void CarrotFIRM::addStateToVisualization(ompl::base::State *state)
{
    CarrotVisualizer::addState(state);
}

void CarrotFIRM::sendFeedbackEdgesToViz()
{
    CarrotVisualizer::ClearFeedbackEdges();

    for(typename std::map<Vertex, Edge>::const_iterator i=feedback_.begin(), e=feedback_.end(); i!=e; ++i)
    {
        Vertex sourceVertex, targetVertex;
        Edge edge;
        sourceVertex = i->first;
        edge = i->second;
        targetVertex = boost::target(edge, g_);
        //TODO: some random target vertex which is not in graph is being assigned, why?
        CarrotVisualizer::addFeedbackEdge(stateProperty_[sourceVertex], stateProperty_[targetVertex], 0);
    }
}


CarrotFIRM::Edge CarrotFIRM::generateRolloutPolicy(const CarrotFIRM::Vertex currentVertex)
{
    /**
        For the given node, find the out edges and see the total cost of taking that edge

        The cost of taking the edge is cost to go from the target of the edge + the cost of the edge itself
    */
    double minCost = 1e10;
    Edge edgeToTake;
    // Iterate over the out edges
    foreach(Edge e, boost::out_edges(currentVertex, g_))
    {

        // Get the target node of the edge
        Vertex targetNode = boost::target(e, g_);

        // The cost to go from the target node
        double nextNodeCostToGo = costToGo_[targetNode];

        // Find the weight of the edge
        FIRMWeight edgeWeight =  boost::get(boost::edge_weight, g_, e);

        // The transition prob of the edge
        double transitionProbability  = edgeWeight.getSuccessProbability();

        // the cost of taking the edge
        double edgeCostToGo = (transitionProbability*nextNodeCostToGo + (1-transitionProbability)*ompl::magic::OBSTACLE_COST_TO_GO) + edgeWeight.getCost();

        if(edgeCostToGo < minCost)
        {
            minCost  = edgeCostToGo;
            edgeToTake = e;

        }

    }

    return edgeToTake;
}

void CarrotFIRM::simulateKidnapping()
{
    siF_->setTrueState(kidnappedState_);
}

bool CarrotFIRM::detectKidnapping(ompl::base::State *previousState, ompl::base::State *newState)
{

    using namespace arma;

    mat previousCov = previousState->as<CarrotBeliefSpace::StateType>()->getCovariance();

    mat newCov = newState->as<CarrotBeliefSpace::StateType>()->getCovariance();

    double innovSignal = (trace(newCov) - trace(previousCov)) / trace(previousCov);

    if(innovSignal >= ompl::magic::KIDNAPPING_INNOVATION_CHANGE_THRESHOLD )
    {
        return true;
    }

    return false;

}

void CarrotFIRM::savePlannerData(int trialNumber/* = 1*/)
{

    std::vector<std::pair<int,std::pair<arma::colvec,arma::mat> > > nodes;

    foreach(Vertex v, boost::vertices(g_))
    {

        arma::colvec xVec = stateProperty_[v]->as<CarrotBeliefSpace::StateType>()->getArmaData();

        arma::mat cov = stateProperty_[v]->as<CarrotBeliefSpace::StateType>()->getCovariance();

        std::pair<int,std::pair<arma::colvec,arma::mat> > nodeToWrite = std::make_pair(v, std::make_pair(xVec, cov)) ;

        nodes.push_back(nodeToWrite);

    }

    std::vector<std::pair<std::pair<int,int>,FIRMWeight> > edgeWeights;

    foreach(Edge e, boost::edges(g_))
    {
        Vertex start = boost::source(e,g_);
        Vertex goal  = boost::target(e,g_);

        const FIRMWeight w = boost::get(boost::edge_weight, g_, e);

        edgeWeights.push_back(std::make_pair(std::make_pair(start,goal),w));

    }

    FIRMUtils::writeFIRMGraphToXML(nodes, edgeWeights, trialNumber);

}


void CarrotFIRM::loadRoadMapFromFile(const std::string pathToFile)
{
    std::vector<std::pair<int,std::pair<arma::colvec,arma::mat> > > FIRMNodeList;

    if(FIRMUtils::readFIRMGraphFromXML(pathToFile, FIRMNodeList, loadedEdgeProperties_))
    {

        loadedRoadmapFromFile_ = true;

        this->setup();

        //std::cout << "[CarrotFIRM] Total number of FIRM Nodes: " << FIRMNodeList.size() << std::endl;

        for(int i = 0; i < FIRMNodeList.size() ; i++)
        {
            ompl::base::State *newState = siF_->allocState();

            arma::colvec xVec = FIRMNodeList[i].second.first;
            arma::mat     cov = FIRMNodeList[i].second.second;
            //std::cout << "[CarrotFIRM] node state: " << xVec << std::endl;

            newState->as<CarrotBeliefSpace::StateType>()->setXYZ(xVec(0),xVec(1),xVec(2));
            newState->as<CarrotBeliefSpace::StateType>()->setCovariance(cov);

            //std::cout<<"Adding state from XML --> \n";
            //siF_->printState(newState);

            Vertex v = addStateToGraph(siF_->cloneState(newState));

            siF_->freeState(newState);

            assert(v==FIRMNodeList[i].first && "IDS DONT MATCH !!");
            //std::cout << "[CarrotFIRM] Nodes added: " << i << std::endl;
        }
    }
}

bool CarrotFIRM::isStartVertex(const Vertex v)
{

    for(int i=0; i <  startM_.size(); i++)
    {
        if(v == startM_[i])
            return true;
    }

    return false;

}

bool CarrotFIRM::isGoalVertex(const Vertex v)
{
    for(int i=0; i <  goalM_.size(); i++)
    {
        if(v == goalM_[i])
            return true;
    }

    return false;

}

void CarrotFIRM::recoverLostRobot(ompl::base::State *recoveredState)
{
   /* // clear the visualization
    CarrotVisualizer::clearStates();

    CarrotVisualizer::setMode(CarrotVisualizer::VZRDrawingMode::NodeViewMode);

    policyGenerator_->sampleNewBeliefStates();

    ompl::base::State *currentTrueState = siF_->allocState();
    siF_->getTrueState(currentTrueState);

    int counter = 0;

    while(!policyGenerator_->isConverged())
    {
        std::vector<ompl::control::Control*> policy;

        policyGenerator_->generatePolicy(policy);

        int rndnum = FIRMUtils::generateRandomIntegerInRange(100, ompl::magic::MAX_MM_POLICY_LENGTH);

        int hzn = rndnum > policy.size()? policy.size() : rndnum;

        for(int i=0; i < hzn ; i++)
        {
            siF_->applyControl(policy[i],true);

            policyGenerator_->propagateBeliefs(policy[i]);

            siF_->getTrueState(currentTrueState);

            // If the robot's clearance gets below the threshold, break loop & replan
            if(!policyGenerator_->areCurrentBeliefsValid() || siF_->getStateValidityChecker()->clearance(currentTrueState) < ompl::magic::MIN_ROBOT_CLEARANCE)
            {
                break;

            boost::this_thread::sleep(boost::posix_time::milliseconds(20));
        }

    }

    std::vector<ompl::base::State*> bstates;

    policyGenerator_->getCurrentBeliefStates(bstates);

    siF_->copyState(recoveredState, bstates[0]);

    CarrotVisualizer::setMode(CarrotVisualizer::VZRDrawingMode::PRMViewMode);
*/
}
