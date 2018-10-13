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
	#include <MiAMEurobot/raspberry_pi/RaspberryPi.h>
	#include <MiAMEurobot/trajectory/Utilities.h>
	#include <math.h>
	#include <stdlib.h>
	#include <stdio.h>
	#include <glib.h>

	#include <memory>
	#include <vector>
	#include <mutex>

	#include "uCListener.h"

	// Right and left macros, for array addressing.
	#define RIGHT 0
	#define LEFT 1

	using miam::trajectory::RobotPosition;
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
		double const wheelRadius = 50.8; ///< Wheel radius, in mm.
		double const wheelSpacing = 106.0; ///< Wheel spacing from robot center, in mm.
		double const encoderWheelRadius = 26.5; ///< Radius of encoder wheels, in mm.
		double const encoderWheelSpacing = 133.0; ///< Encoder wheel spacing from robot center, in mm.

		double const stepSize = 2 * G_PI / 600.0; ///< Size of a motor step, in rad.

		double const maxWheelSpeed = 700; ///< Maximum wheel speed, in mm/s.
		double const maxWheelAcceleration = 1000; ///< Maximum wheel acceleration, in mm/s^2.
	}

	/// \brief Class representing the robot wheeled base.
	/// \details This class simply centralizes variables linked to robot motion.
	///          It implements the low-level thread of the robot, responsible for driving it around the table, and logging.
	///          Comunication with this thread is done through this class, in a thread-safe way when needed.
	class Robot
	{
		public:
			dualL6470 stepperMotors_; ///< Robot driving motors.

			/// \brief Constructor: do nothing for now.
			Robot();

			/// \brief Initialize the robot.
			/// \details This function only performs object initialization, but does not start the low-level thread.
			/// \return true if initialize is successful, false otherwise.
			bool init();

			/// \brief Get current robot position.
			/// \return Current robot position.
			RobotPosition getCurrentPosition();

			/// \brief Reset the position of the robot on the table.
			///
			/// \details This function might be used for example when the robot is put in contact with a side of the table,
			///			 to gain back absolute position accuracy.
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
			void lowLevelThread();

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

			// Current robot status.
			ProtectedPosition currentPosition_; ///< Current robot position, thread-safe.
			std::mutex positionMutex_;	///< Mutex for writing to currentPosition_
			miam::trajectory::TrajectoryPoint trajectoryPoint_; ///< Current trajectory point.
			double currentTime_; ///< Current robot time, counted by low-level thread.
			double motorSpeed_[2]; ///< Current motor speed.
			int motorPosition_[2]; ///< Current motor position.
			uCData microcontrollerData_; ///< Data structure containing informations from the arduino board.
			Logger logger_; ///< Logger object.

			// Trajectory definition.
			std::vector<std::shared_ptr<Trajectory>> newTrajectories_; ///< Vector of new trajectories to follow.
			std::vector<std::shared_ptr<Trajectory>> currentTrajectories_; ///< Current trajectories being followed.

			// Trajectory following timing.
			double trajectoryStartTime_; ///< Time at which the last trajectory following started.
			double lastTrajectoryFollowingCallTime_; ///< Time at which the last trajectory following started.
	};

	extern Robot robot;	///< The robot instance, representing the current robot.
 #endif
