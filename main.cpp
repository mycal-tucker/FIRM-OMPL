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

#include "FIRMCarrotSetup.h"
//#include "ros.h"
//#include "FIRM2DSetup.h"
#include "MultiModalSetup.h"
#include "Tests.h"
#include <QApplication>
#include <QtGui/QDesktopWidget>
#include "include/Visualization/CarrotWindow.h"
#include "include/Visualization/CarrotVisualizer.h"
//#include "include/Visualization/Visualizer.h"
#include <boost/thread.hpp>
#include <iostream>
#include <istream>


using namespace std;

void plan(bool prm)
{

    FIRMCarrotSetup *mySetup(new FIRMCarrotSetup);

    std::string setupFilePath = "./SetupFiles/CarrotWorld.xml";

    OMPL_INFORM("Loaded Setup File");

    mySetup->setPathToSetupFile(setupFilePath.c_str());

    mySetup->setup( prm );

    CarrotVisualizer::updateRenderer(*dynamic_cast<const ompl::app::RigidBodyGeometry*>(mySetup), mySetup->getGeometricStateExtractor());

    CarrotVisualizer::updateSpaceInformation(mySetup->getSpaceInformation());

    CarrotVisualizer::setMode(CarrotVisualizer::VZRDrawingMode::PRMViewMode);

    bool solved = false;

    if(mySetup->solve(prm))
    {
        mySetup->executeSolution(prm);

        OMPL_INFORM("Plan Executed Successfully");
        solved = true;
    }

    if (solved == false)
    {
        OMPL_INFORM("Unable to find Solution in given time.");

    }

    delete mySetup;

}


int main(int argc, char *argv[])
{

    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <PLANNER>" << std::endl;
        return 1;
    }
    std::string planner = argv[1];
    bool prm;
    if (planner.compare("prm") == 0)
    {
        prm = true;
        std::cout << "Planning with PRM..." << std::endl;
    }
    else if (planner.compare("firm") == 0)
    {
        prm = false;
        std::cout << "Planning with FIRM..." << std::endl;
    }
    else
    {
        std::cerr << "For planner, type either 'firm' or 'prm'" << std::endl;
        return 1;
    }

    srand(1234567);

    arma_rng::set_seed(1234567);

    QApplication app(argc, argv);

    MyCarrotWindow window;

    window.resize(window.sizeHint());

    window.showMaximized();

    window.resetCamera();

    boost::thread solveThread(plan, prm);

    app.exec();

    solveThread.join();

    OMPL_INFORM("Task Complete");

    return 0;


}
