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
    #include <MiAMEurobot/beaglebone/BeagleboneBlack.h>
    #include <math.h>
    #include <stdlib.h>
    #include <stdio.h>

    #include <memory>
    #include <vector>
    #include <mutex>

    #include "LoggerFields.h"

    // Right and left macros, for array addressing.
    #define RIGHT 0
    #define LEFT 1

    using miam::RobotPosition;
    using miam::trajectory::Trajectory;


    /// \brief Simple class to provide thread-safe access to the robot position.
    class ProtectedPosition{
        public:
            ProtectedPosition():
                position_(),
                mutex_()
            {
            }

            RobotPosition get()
            {
                RobotPosition p;
                mutex_.lock();
                p = position_;
                mutex_.unlock();
                return p;
            }

            void set(RobotPosition const& p)
            {
                mutex_.lock();
                position_ = p;
                mutex_.unlock();
            }

        private:
            RobotPosition position_;
            std::mutex mutex_;
    };

    // Dimensions of the robot
    namespace robotdimensions
    {
        double const wheelRadius = 49.3; ///< Wheel radius, in mm - identified during open loop experiments.
        double const wheelSpacing = 98.6; ///< Wheel spacing from robot center, in mm - identified during open loop experiments.
        double const encoderWheelRadius = 25.3; ///< Radius of encoder wheels, in mm.
        double const encoderWheelSpacing = 140.0; ///< Encoder wheel spacing from robot center, in mm.

        double const stepSize = 2 * M_PI / 200.0; ///< Size of a motor step, in rad.

        double const maxWheelSpeed = 400; ///< Maximum wheel speed, in mm/s.
        double const maxWheelAcceleration = 400; ///< Maximum wheel acceleration, in mm/s^2.
    }

    // Controller parameters
    namespace controller
    {
        //~ double const transverseKp = 0.1;

        //~ double const linearKp = 0.5;
        //~ double const linearKd = 0.0;
        //~ double const linearKi = 0.05;

        //~ double const rotationKp = 0.15;
        //~ double const rotationKd = 0.0;
        //~ double const rotationKi = 0.05;
        double const transverseKp = 0.0;

        double const linearKp = 0.0;
        double const linearKd = 0.0;
        double const linearKi = 0.00;

        double const rotationKp = 0.0;
        double const rotationKd = 0.0;
        double const rotationKi = 0.0;
    }

    /// \brief Class representing the robot wheeled base.
    /// \details This class simply centralizes variables linked to robot motion.
    ///          It implements the low-level thread of the robot, responsible for driving it around the table, and logging.
    ///          Comunication with this thread is done through this class, in a thread-safe way when needed.
    class Robot
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

            /// \brief Get current robot position.
            /// \return Current robot position.
            RobotPosition getCurrentPosition();

            /// \brief Get current robot base speed.
            /// \return Current robot base speed.
            BaseSpeed getCurrentBaseSpeed();

            /// \brief Reset the position of the robot on the table.
            ///
            /// \details This function might be used for example when the robot is put in contact with a side of the table,
            ///             to gain back absolute position accuracy.
            ///
            /// \param[in] resetPosition The position to which reset the robot.
            /// \param[in] resetX Wheather or not to reset the X coordinate.
            /// \param[in] resetY Wheather or not to reset the Y coordinate.
            /// \param[in] resetTheta Wheather or not to reset the angle.
            void resetPosition(RobotPosition const& resetPosition, bool const& resetX, bool const& resetY, bool const& resetTheta);

            /// \brief Set new trajectory set to follow.
            /// \details This function is used to set the trajectories which will be followed by
            ///          the low-level thread. This function simply gives the input to the low-level thread
            ///          and returns immediately: use waitForTrajectoryFinish
            ///
            /// \param[in] trajectories Vector of trajectory to follow.
            void setTrajectoryToFollow(std::vector<std::shared_ptr<Trajectory>> const& trajectories);

            /// \brief Wait for the current trajectory following to be finished.
            /// \return true if trajectory following was successful, false otherwise.
            bool waitForTrajectoryFinished();

            /// \brief The low-level thread of the robot.
            /// \details This thread runs a periodic loop. At each iteration, it updates sensor values,
            ///          estimates the position of the robot on the table, and performs motor servoing.
            ///          It also logs everything in a log file.
            void lowLevelLoop();

            /// \brief Move the two servos of the robot.
            /// \details The secondary robot only has two servos, that always move together between only two positions:
            ///          there is no need for a specific handler.
            ///
            /// \param[in] down If true, lower the servos, else raise them.
            void moveServos(bool down = true);

            // List of all system on the robot, public for easy external access (they might be moved latter on).
            miam::L6470 stepperMotors_; ///< Robot driving motors.
            IMU imu_; ///< Robot driving motors.
            LCD screen_;


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


            /// \brief Opponent robot detection.
            ///
            /// \details This function polls the IR sensors, computes obstacle data, and returns true if the robot should
            ///          stop because of an obstacle.
            ///
            bool handleDetection();

            // Current robot status.
            ProtectedPosition currentPosition_; ///< Current robot position, thread-safe.
            BaseSpeed currentBaseSpeed_; ///< Current robot base speed.
            miam::trajectory::TrajectoryPoint trajectoryPoint_; ///< Current trajectory point.
            double currentTime_; ///< Current robot time, counted by low-level thread.
            std::vector<double> motorSpeed_; ///< Current motor speed.
            std::vector<int> motorPosition_; ///< Current motor position.
            Logger logger_; ///< Logger object.

            // Trajectory definition.
            std::vector<std::shared_ptr<Trajectory>> newTrajectories_; ///< Vector of new trajectories to follow.
            std::vector<std::shared_ptr<Trajectory>> currentTrajectories_; ///< Current trajectories being followed.

            // Trajectory following timing.
            double trajectoryStartTime_; ///< Time at which the last trajectory following started.
            double lastTrajectoryFollowingCallTime_; ///< Time at which the last trajectory following started.

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
            bool isStepperInit_; ///< Boolean representing the initialization of the stepper motors.
            bool isIMUInit_; ///< Boolean representing the initialization of the IMU.
            bool isScreenInit_; ///< Boolean representing the initialization of the LCD screen.
            int IRFrontLeft_; ///< Value of the front left IR sensor.
            int IRFrontRight_; ///< Value of the front right IR sensor.
            int IRBackLeft_; ///< Value of the back left IR sensor.
            int IRBackRight_; ///< Value of the back right IR sensor.
            bool isFrontDetectionActive_; ///< True if a robot is visible in front of the robot.
            bool isBackDetectionActive_; ///< True if a robot is visible behind of the robot.
            bool hasDetectionStoppedRobot_; ///< True if robot is stopped due to a detection.
            double detectionStopTime_; ///< Time at which the robot stopped due to a detection.

            bool isPlayingRightSide_; ///< True if robot is playing on the right (purple) side of the field.
            bool hasMatchStarted_;    ///< Boolean flag to indicate match status.
            double matchStartTime_;   ///< Start time of the match, for end timer.
    };

    extern Robot robot;    ///< The robot instance, representing the current robot.

    void strategy(Robot *robot); ///< Robot strategy, to be run as a separate thread.

 #endif
