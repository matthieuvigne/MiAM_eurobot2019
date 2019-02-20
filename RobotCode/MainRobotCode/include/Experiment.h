/// \file Experiment.h
/// \brief Bluetooth communciation with experiment.

#ifndef EXPERIMENT_H
     #define EXPERIMENT_H

	/// \brief Class to communicate via bluetooth the the experiment.
	/// \details This class initializes the connection, checks that it is alive and that the arduino is responding,
	///          and sends a start signal to the experiment.
	class Experiment
	{
		public:
			/// \param Constructor.
			Experiment();

			/// \brief Try to connect to the experiment.
			///
			/// \return True on success, false otherwise.
			bool startConnection();

			/// \brief Test connection with the experiment.
			///
			/// \return True if experiment is connected, false otherwise.
			bool isConnected();

			/// \brief Send start message to the experiment.
			///
			/// \return True if start acknoledge is well recieved, false otherwise.
			bool start();

		private:
			int port_;	///< The port file descriptor.
	};
 #endif
