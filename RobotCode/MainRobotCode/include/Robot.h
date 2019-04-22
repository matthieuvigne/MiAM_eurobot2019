/// \file Robot.h
/// \brief This file defines global variables representing the hardware on the robot, that needs sharing between
///        different files.
///
/// \details This header is included in all other source files. It defines a structure representing the physical robot
///          in the code. Note that these variables are directly available, unprotected: it is to the user to ensure
///          no race condition may occur, for instance when writing the current robot position.
#ifndef ROBOT_H
     #define ROBOT_H

    ///< Global includes
    #include <MiAMEurobot/MiAMEurobot.h>
    #include <MiAMEurobot/drivers/USBLCDDriver.h>
    #include <MiAMEurobot/raspberry_pi/RaspberryPi.h>
    #include <MiAMEurobot/trajectory/PointTurn.h>
    #include <MiAMEurobot/trajectory/Utilities.h>
    #include <MiAMEurobot/trajectory/DrivetrainKinematics.h>
    #include <math.h>
    #include <stdlib.h>
    #include <stdio.h>

    #include <memory>
    #include <vector>
    #include <mutex>

    #include "uCListener.h"
    #include "ServoHandler.h"
    #include "LoggerFields.h"

    // Right and left macros, for array addressing.
    #define RIGHT 0
    #define LEFT 1

    using miam::RobotPosition;
    using miam::ProtectedPosition;
    using miam::trajectory::Trajectory;

    // Dimensions of the robot
    namespace robotdimensions
    {
        double const wheelRadius = 49.3; ///< Wheel radius, in mm - identified during open loop experiments.
        double const wheelSpacing = 100.5; ///< Wheel spacing from robot center, in mm - identified during open loop experiments.
        double const encoderWheelRadius = 25.3; ///< Radius of encoder wheels, in mm.
        //~ double const encoderWheelSpacing = 140.0; ///< Encoder wheel spacing from robot center, in mm.
        double const encoderWheelSpacing = 141.5; ///< Encoder wheel spacing from robot center, in mm.

        double const stepSize = 2 * M_PI / 600.0; ///< Size of a motor step, in rad.

        double const maxWheelSpeed = 600; ///< Maximum wheel speed, in mm/s.
        double const maxWheelAcceleration = 800; ///< Maximum wheel acceleration, in mm/s^2.

        double const maxWheelSpeedTrajectory = 400; ///< Maximum wheel speed, in mm/s, for trajectory generation.
        double const maxWheelAccelerationTrajectory = 400; ///< Maximum wheel acceleration, in mm/s^2, for trajectory generation.
    }

    // Controller parameters
    namespace controller
    {
        //~ double const transverseKp = 0.1;

        //~ double const linearKp = 3.0;
        double const linearKp = 0.0;
        double const linearKd = 0.0;
        double const linearKi = 0.1;

        double const transverseKp = 0.0;

        //~ double const rotationKp = 10.0;
        double const rotationKp = 0.0;
        double const rotationKd = 0.01;
        double const rotationKi = 0.0;
    }

    class Robot : public AbstractRobot
    {
        public:

            /// \brief Constructor: do nothing for now.
            Robot();

            /// \brief Initialize every system of the robot.
            /// \details This function tries to initialize every component of the robot, by creating the software
            ///          object and, when applicable, testing the connection with the real component.
            ///          This function can be called several times, and only tries to re-initialize a component
            ///          if previous initializations failed.
            /// \return true if all components were initialized successfully, false otherwise.
            bool initSystem();

            /// \brief The low-level thread of the robot.
            /// \details This thread runs a periodic loop. At each iteration, it updates sensor values,
            ///          estimates the position of the robot on the table, and performs motor servoing.
            ///          It also logs everything in a log file.
            void lowLevelLoop();

            /// \brief Move the rail.
            ///
            /// \param position Relative rail position, form 0 (down) to 1 (up).
            void moveRail(double position);

            // List of all system on the robot, public for easy external access (they might be moved latter on).
            ServoHandler servos_; ///< Interface for the servo driver.
            USBLCD screen_; ///< LCD screen and buttons.
        private:
            /// \brief Update the logfile with current values.
            void updateLog();

            /// \brief Update the target of the trajectory following algorithm.
            /// \details This function is responsible for handling new trajectories, and switching through
            ///          the trajectory vector to follow.
            /// \param[in] dt Time since this function was last called, for PD controller.
            void updateTrajectoryFollowingTarget(double const& dt);

            /// \brief Follow a trajectory.
            /// \details This function computes motor velocity to reach a specific trajectory point, and sends
            ///          it to the motors.
            /// \param[in] traj Current trajectory to follow.
            /// \param[in] timeInTrajectory Current time since the start of the trajectory.
            /// \param[in] dt Time since last servoing call, for PID controller.
            /// \return True if trajectory following should continue, false if trajectory following is completed.
            bool followTrajectory(Trajectory *traj, double const& timeInTrajectory, double const& dt);

            /// \brief Perform robot setup, return wheather the match has started or not.
            ///
            /// \details This function is called periodically before the match starts. It is responsible for
            ///          updating the display and status according to user input. It returns true whenever the match
            ///          has started.
            bool setupBeforeMatchStart();



            uCData microcontrollerData_; ///< Data structure containing informations from the arduino board.
            Logger logger_; ///< Logger object.

            // Traking errors.
            double trackingLongitudinalError_; ///< Tracking error along tangent to trajectory.
            double trackingTransverseError_; ///< Tracking error along normal to trajectory.
            double trackingAngleError_; ///< Tracking angle error.

            // Tracking PIDs
            miam::PID PIDLinear_; ///< Longitudinal PID.
            miam::PID PIDAngular_; ///< Angular PID.

            // Kinematics
            DrivetrainKinematics kinematics_;

            // Init variables.
            bool isScreenInit_; ///< Boolean representing the initialization of the screen motors.
            bool isStepperInit_; ///< Boolean representing the initialization of the stepper motors.
            bool isServosInit_; ///< Boolean representing the initialization of the stepper motors.
            bool isArduinoInit_; ///< Boolean representing the initialization of the slave arduino board.
    };

    extern Robot robot;    ///< The robot instance, representing the current robot.

    void matchStrategy(); ///< Robot strategy, to be run as a separate thread.
 #endif
