#include "M1DemoMachineROS.h"

#define OWNER ((M1DemoMachineROS *)owner)

M1DemoMachineROS::M1DemoMachineROS(int argc, char *argv[]){
    spdlog::debug("M1DemoMachineROS::constructed!");

    ros::init(argc, argv, "m1", ros::init_options::NoSigintHandler);
    ros::NodeHandle nodeHandle("~");

    // Get robot name from the node name
    robotName_ = ros::this_node::getName();
    robotName_.erase(0,1); // erase the first character which is '/'

    // create robot
    robot_ = new RobotM1(robotName_);

    // Create ros object
    M1MachineRos_ = new M1MachineROS(robot_);

    // Pass nodeHandle to the classes that use ROS features
    M1MachineRos_->setNodeHandle(nodeHandle);
    M1MachineRos_->initialize();

    // Create states with ROS features // This should be created after ros::init()
    multiControllerState_ = new MultiControllerState(this, robot_, M1MachineRos_);

    //Initialize the state machine with first state of the designed state machine, using baseclass function.
    StateMachine::initialize(multiControllerState_);
}

M1DemoMachineROS::~M1DemoMachineROS() {
    currentState->exit();
    robot_->disable();
    delete M1MachineRos_;
    delete robot_;
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */

void M1DemoMachineROS::init() {
//    ros::init(argc, argv, "m1", ros::init_options::NoSigintHandler);
//    ros::NodeHandle nodeHandle("~");

//    // Pass nodeHandle to the classes that use ROS features
//    M1MachineRos_->setNodeHandle(nodeHandle);

    if(robot_->initialise()) {
        initialised = true;
    }
    else {
        initialised = false;
        std::cout /*cerr is banned*/ << "Failed robot initialisation. Exiting..." << std::endl;
        std::raise(SIGTERM); //Clean exit
    }
    running = true;

    time0_ = std::chrono::steady_clock::now();

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::stringstream logFileName;

//    std::string robotName_ = ros::this_node::getName();

    logFileName << "spdlogs/" << robotName_<< std::put_time(&tm, "/%d-%m-%Y_%H-%M-%S") << ".csv";

    logHelper.initLogger("test_logger", logFileName.str(), LogFormat::CSV, true);
    logHelper.add(time_, "time");
    logHelper.add(multiControllerState_->controller_mode_, "mode");

    logHelper.add(robot_->getPosition(), "JointPositions");
    logHelper.add(robot_->getVelocity(), "JointVelocities");
    logHelper.add(robot_->getTorque(), "JointTorques");
    logHelper.add(robot_->getJointTor_s(), "SensorTorques");
    logHelper.add(multiControllerState_->tau_raw, "SensorTorques_raw");
    logHelper.add(multiControllerState_->tau_filtered, "SensorTorques_filtered");

    logHelper.add(multiControllerState_->q_raw, "q_raw");
    logHelper.add(multiControllerState_->q_filtered, "q_filtered");

    logHelper.add(multiControllerState_->spk_, "SpringStiffness");
    logHelper.add(multiControllerState_->spring_tor, "SpringTorque");
    logHelper.add(multiControllerState_->tau_cmd, "CommandTorque");      // motor_torque = command_torque + compensation_torque
    logHelper.add(robot_->tau_motor, "MotorTorque");

    logHelper.add(M1MachineRos_->jointTorqueCommand_, "MM1_DesiredJointTorques");
    logHelper.add(M1MachineRos_->jointPositionCommand_, "MM1_DesiredJointPositions");
    logHelper.add(M1MachineRos_->interactionTorqueCommand_, "MM1_DesiredInteractionTorques");

    logHelper.add(multiControllerState_->digitalInValue_, "digitalIn");
    logHelper.add(multiControllerState_->digitalOutValue_, "digitalOut");

    logHelper.startLogger();
}

void M1DemoMachineROS::end() {
    if(initialised) {
        currentState->exit();
        robot_->stop();
        logHelper.endLog();
        delete M1MachineRos_;
        delete robot_;
    }
}

bool M1DemoMachineROS::configureMasterPDOs() {
    spdlog::debug("M1DemoMachine::configureMasterPDOs()");
    return robot_->configureMasterPDOs();
}

/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void M1DemoMachineROS::hwStateUpdate(void) {
    robot_->updateRobot();
    M1MachineRos_->update();
    time_ = (std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - time0_).count()) / 1e6;
    ros::spinOnce();
}