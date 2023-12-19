#include "M2ProDemoStates.h"
#include "M2ProDemoMachine.h"

#define OWNER ((M2ProDemoMachine *)owner)

double timeval_to_sec(struct timespec *ts)
{
    return (double)(ts->tv_sec + ts->tv_nsec / 1000000000.0);
}

VM2 impedance(Eigen::Matrix2d K, Eigen::Matrix2d D, VM2 X0, VM2 X, VM2 dX, VM2 dXd=VM2::Zero()) {
    return K*(X0-X) + D*(dXd-dX);
}

//minJerk(X0, Xf, T, t, &X, &dX)
double JerkIt(VM2 X0, VM2 Xf, double T, double t, VM2 &Xd, VM2 &dXd) {
    t = std::max(std::min(t, T), .0); //Bound time
    double tn=std::max(std::min(t/T, 1.0), .0);//Normalised time bounded 0-1
    double tn3=pow(tn,3.);
    double tn4=tn*tn3;
    double tn5=tn*tn4;
    Xd = X0 + ( (X0-Xf) * (15.*tn4-6.*tn5-10.*tn3) );
    dXd = (X0-Xf) * (4.*15.*tn4-5.*6.*tn5-10.*3*tn3)/t;
    return tn;
}


void M2DemoState::entryCode(void) {
    //robot->applyCalibration();
    //robot->initPositionControl();
    //robot->initVelocityControl();
    robot->initTorqueControl();
    qi=robot->getPosition();
    Xi=robot->getEndEffPosition();
    //robot->setJointVelocity(VM2::Zero());
    //robot->setEndEffForceWithCompensation(VM2::Zero(), false);
    robot->printJointStatus();
    tau = VM2::Zero();
}
void M2DemoState::duringCode(void) {
    if(iterations()%100==1) {
        //std::cout << "Doing nothing for "<< elapsedTime << "s..." << std::endl;
        std::cout << running() << " ";
        robot->printJointStatus();
        robot->printStatus();
    }
    robot->setEndEffForceWithCompensation(tau, false);
    /*VM2 q = robot->getJointPos();
    q(1)=68*M_PI/180.-0.1*elapsedTime;*/
    //std::cout << q.transpose() <<std::endl;
    //robot->setJointPos(qi-VM2(0.03,0.03,0.03));
    //double v=-sin(2*M_PI*1./10*elapsedTime);
    //double v=-0.1;
    //robot->setJointVel(VM2(0,0,0));

    //robot->printStatus();

    /*VM2 dX(-0.02,0.05,0.1);
    if(robot->getEndEffPosition()(2)<0) {
        robot->setEndEffVel(dX);
    }
    else {
        robot->setEndEffVel(VM2(0,0,0));
    }*/


    /*VM2 Dq;
    if(elapsedTime<5)
        Dq={0,0.015*elapsedTime,0.015*elapsedTime};
    else
        Dq={0,0.015*5.,0.015*5.};
    robot->setJointPos(qi-Dq);*/

    /*VM2 tau(0,-5.0,0);*/
    //robot->setJointTor(robot->calculateGravityTorques());


    /*float k_i=1.;
    VM2 Xf(-0.4, 0, 0);
    VM2 Xd, dXd;
    JerkIt(Xi, Xf, 5., elapsedTime, Xd, dXd);
    robot->setEndEffVelocity(dXd+k_i*(Xd-robot->getEndEffPosition()));
    std::cout << (Xd-robot->getEndEffPosition()).norm() << std::endl;*/
}
void M2DemoState::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero());
}



void M2CalibState::entryCode(void) {
    calibDone=false;
    for(unsigned int i=0; i<2; i++) {
        stop_reached_time[i] = .0;
        at_stop[i] = false;
    }
    robot->decalibrate();
    robot->initTorqueControl();
    robot->printJointStatus();
    std::cout << "Calibrating (keep clear)..." << std::flush;
}
//Move slowly on each joint until max force detected
void M2CalibState::duringCode(void) {
    VM2 tau(0, 0);

    //Apply constant torque (with damping) unless stop has been detected for more than 0.5s
    VM2 vel=robot->getVelocity();
    double b = 16;
    for(unsigned int i=0; i<vel.size(); i++) {
        tau(i) = -std::min(std::max(8- b * vel(i), .0), 8.);
        if(stop_reached_time(i)>1) {
            at_stop[i]=true;
        }
        if(abs(vel(i))<0.005) {
            stop_reached_time(i) += dt();
        }
    }

    //Switch to gravity control when done
    if(robot->isCalibrated()) {
        robot->setEndEffForceWithCompensation(VM2::Zero(), false);
        calibDone=true; //Trigger event
    }
    else {
        //If all joints are calibrated
        if(at_stop[0] && at_stop[1]) {
            robot->applyCalibration();
            std::cout << "OK." << std::endl;
        }
        else {
            tau(0)=tau(0)/2;
            robot->setJointTorque(tau);
            if(iterations()%100==1) {
                std::cout << "." << std::flush;
            }
        }
    }
}
void M2CalibState::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero());
}



void M2Transparent::entryCode(void) {
    robot->initTorqueControl();
}
void M2Transparent::duringCode(void) {

    //Apply corresponding force
    robot->setEndEffForceWithCompensation(VM2::Zero(), true);

    if(iterations()%100==1) {
        robot->printStatus();
    }
}
void M2Transparent::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero());
}



void M2EndEffDemo::entryCode(void) {
    robot->initVelocityControl();
}
void M2EndEffDemo::duringCode(void) {

    //Joystick driven
    VM2 dXd = VM2::Zero();
    for(unsigned int i=0; i<dXd.size(); i++) {
        dXd(i)=robot->joystick->getAxis(i)/2.;
    }

    //Apply
    robot->setEndEffVelocity(dXd);

    if(iterations()%100==1) {
        std::cout << dXd.transpose() << "  ";
        robot->printStatus();
    }
}
void M2EndEffDemo::exitCode(void) {
    robot->setEndEffVelocity(VM2::Zero());
}


void M2DemoPathState::entryCode(void) {
    robot->initTorqueControl();
    robot->setEndEffForceWithCompensation(VM2::Zero(), false);
    Xi=robot->getEndEffPosition();
}
void M2DemoPathState::duringCode(void) {
    //TODO: TBD
}
void M2DemoPathState::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero());
}


void M2DemoMinJerkPosition::entryCode(void) {
    //Setup velocity control for position over velocity loop
    robot->initVelocityControl();
    robot->setJointVelocity(VM2::Zero());
    //Initialise to first target point
    TrajPtIdx=0;
    startTime=running();
    Xi=robot->getEndEffPosition();
    Xf=TrajPt[TrajPtIdx];
    T=TrajTime[TrajPtIdx];
    k_i=1.;
}
void M2DemoMinJerkPosition::duringCode(void) {

    VM2 Xd, dXd;
    //Compute current desired interpolated point
    double status=JerkIt(Xi, Xf, T, running()-startTime, Xd, dXd);
    //Apply position control
    robot->setEndEffVelocity(dXd+k_i*(Xd-robot->getEndEffPosition()));


    //Have we reached a point?
    if(status>=1.) {
        //Go to next point
        TrajPtIdx++;
        if(TrajPtIdx>=TrajNbPts){
            TrajPtIdx=0;
        }
        //From where we are
        Xi=robot->getEndEffPosition();
        //To next point
        Xf=TrajPt[TrajPtIdx];
        T=TrajTime[TrajPtIdx];
        startTime=running();
    }


   /* //Display progression
    if(iterations%100==1) {
        std::cout << "Progress (Point "<< TrajPtIdx << ") |";
        for(int i=0; i<round(status*50.); i++)
            std::cout << "=";
        for(int i=0; i<round((1-status)*50.); i++)
            std::cout << "-";

        std::cout << "| (" << status*100 << "%)  ";
        robot->printStatus();
    }*/
}
void M2DemoMinJerkPosition::exitCode(void) {
    robot->setJointVelocity(VM2::Zero());
}

void M2ProStiffnessEst::entryCode(void) {
    //generate the displacement trajectory
    VM2 startPoint = {1,0};
    VM2 endPoint = {2,0};
    
    // generate a random interference position, defined by a position thresould of x axis
    srand(time(NULL));
    int randomNum = rand() % 100; // generate a random number between 0 and 99
    xThreshold = 0.01*randomNum*(endPoint(0)-startPoint(0))+startPoint(0); // generate a random position threshold

    //initialise the robot to torque control mode
    robot->flag=0;
    robot->initTorqueControl();
}
void M2ProStiffnessEst::duringCode(void) {
    // get current position
    VM2 currPosition =robot->getEndEffPosition();

    //judge if getting ready at the start position, if arrived, change flag to 1
    float distance = sqrt(pow(currPosition(0)-startPoint(0),2)+pow(currPosition(1)-startPoint(1),2));
    if ( distance <= 0.01 && robot->flag==0) {
        robot->flag=1;
    }
    
    //judge if arriving at the interference position, if arrived, change to velocity control, change flag to 2
    if (currPosition(0) > xThreshold && robot->flag==1) {
        robot->flag=2;
        robot->initVelocityControl();
        robot->setEndEffVelocity(VM2::Zero());
    
        //calculate the trajectory path
        displacement = 0.1;
        TrajPt[TrajNbPts]={VM2(Xi(0), Xi(1)+displacement), VM2(Xi(0), Xi(1))};

        //Initialise to first target point
        TrajPtIdx=0;
        startTime=running();
        Xf=TrajPt[TrajPtIdx];
        Xi=robot->getEndEffPosition();
        T=TrajTime[TrajPtIdx];
        k_i=1.;
    }
    // after completing the interference, change to torque control, change flag to 3

    //judge if arriving at the end position, if arrived, change flag to 4
    distance = sqrt(pow(currPosition(0)-endPoint(0),2)+pow(currPosition(1)-endPoint(1),2));
    if ( distance <= 0.01 && robot->flag==1) {
        robot->flag=4;
    }

    //
    switch (robot->flag)
    {
    case 0: // transparent mode
        robot->setEndEffForceWithCompensation(VM2::Zero(), true);
        if(iterations()%100==1) {
            std::cout << "Please go to the starting position" << std::endl;
        }
    case 1: // transparent mode
        robot->setEndEffForceWithCompensation(VM2::Zero(), true);
        if(iterations()%100==1) {
            robot->printStatus();
        }
        break;
    case 2: //minimum jerk trajectory to apply a displacement
        VM2 Xd, dXd;
        //Compute current desired interpolated point
        double status=JerkIt(Xi, Xf, T, running()-startTime, Xd, dXd);
        //Apply position control
        robot->setEndEffVelocity(dXd+k_i*(Xd-robot->getEndEffPosition()));

        //Have we reached a point?
        if(status>=1.) {
            //Go to next point
            TrajPtIdx++;
            //Have we reached the end of the trajectory?
            if(TrajPtIdx>=TrajNbPts){
                robot->flag=3;
                robot->initTorqueControl();
            }
            //From where we are
            Xi=robot->getEndEffPosition();
            //To next point
            Xf=TrajPt[TrajPtIdx];
            T=TrajTime[TrajPtIdx];
            startTime=running();
        }
        break;
    case 3: // transparent mode
        robot->setEndEffForceWithCompensation(VM2::Zero(), true);
        if(iterations()%100==1) {
            robot->printStatus();
        }
    
    case 4: // arrive at the target position
        robot->setEndEffForceWithCompensation(VM2::Zero(), true);
        if(iterations()%100==1) {
            std::cout << "Arriving !" << std::endl;
        }
    default:
        break;
    }

}
void M2ProStiffnessEst::exitCode(void) {
    robot->flag=0;
    robot->setEndEffForceWithCompensation(VM2::Zero(), true);
}

