/// \file RPLidarHandler.h
/// \brief Handler for RPLidar sensor.
///
/// \details Class RPLidarHandler deals directly with the lidar measurement, and performs robot detection based on LIDAR
///          data.
///             Robot detection works as follow: whenever a new data point is aquired, we look at its distance from the current
///             blob average. A point more than BLOB_THICKNESS distance away from the average is not added. If more than
///          BLOB_BREAK consecutive points are not added, the current blob is considered terminated. If its width matches
///          a robot beacon, the corresponding position is stored. In any case, a new blob is created by the current point.

#ifndef RPLIDAR_HANDLER
#define RPLIDAR_HANDLER
    #include <rplidar.h>
    #include <iostream>
    #include <cmath>
    #include <deque>

    // Constant parameters.
    #define DEBUGGING_BUFFER_LENGTH 2000

    const double LIDAR_RPM = 600.0;    ///< Lidar velocity, in rpm.
    const double MAX_DISTANCE = 2000.0; ///< Maximum distance for processing, in mm: points above that distance are discarded.

    const double BLOB_THICKNESS = 100.0; //80 ///< Distance from bloc center to consider a point in the blob, in mm.
    const double BLOB_MIN_SIZE = 60.0; //60 ///< Minimum size of the blob to consider it as a robot.
    const double BLOB_MAX_SIZE = 140.0; //140 ///< Maximum size of the blob to consider it as a robot.

    const int BLOB_BREAK = 2; //2 ///< Number of points needed to consider that a block has come to an endMinimum number of points to be a valid obstacle.

    const int MIN_POINTS = 4; // 4 ///< Minimum number of points inside a blob to be considered a robot.
                             ///< At 1.5m, 600rpm, 8ksamples/s, a circle of 70mm corresponds to 6 points.

    /// \brief Structure representing a data point returned by the lidar.
    struct LidarPoint
    {
        LidarPoint():
            r(1000.0),
            theta(2 * M_PI),
            x(1000.0),
            y(0)
        {
        }

        LidarPoint(double const& rIn, double const& thetaIn):
            r(rIn),
            theta(thetaIn),
            x(rIn * std::cos(thetaIn)),
            y(rIn * std::sin(thetaIn))
        {
        }

        double r;
        double theta;
        double x;
        double y;
    };

    //~ struct Blob {

        //~ Blob(
            //~ const double r_min,
            //~ const double r_max,
            //~ const double theta_min,
            //~ const double theta_max)
        //~ :    r_min       (r_min),
            //~ r_max       (r_max),
            //~ theta_min   (theta_min),
            //~ theta_max   (theta_max)
        //~ {
            //~ assert(r_max > r_min);
            //~ assert(theta_max > theta_min);
        //~ }

        //~ LidarPoint getCenter()
        //~ {
            //~ return LidarPoint(
                //~ r_max - r_min,
                //~ theta_max - theta_min);
        //~ }

        //~ LidarPoint* getBoundingBoxCorners()
        //~ {
            //~ LidarPoint corners[4];

            //~ corners[0] = LidarPoint(r_min, theta_min);
            //~ corners[1] = LidarPoint(r_min, theta_max);
            //~ corners[2] = LidarPoint(r_max, theta_max);
            //~ corners[3] = LidarPoint(r_max, theta_min);

            //~ return corners;
        //~ }

        //~ r_min;
        //~ r_max;
        //~ theta_min;
        //~ theta_max;
    //~ }:

    class RPLidarHandler
    {
        public:
            /// \brief Constructor.
            ///
            /// \details The constructor does not perform any communication operation: use init for that.
            RPLidarHandler();

            /// \brief Destructor.
            ~RPLidarHandler();

            /// \brief Init communication with the sensor.
            /// \details This function initialises communication with the RPLidar, and starts scan mode.
            ///
            /// \param[in] portNameIn Name of the port to use.
            /// \return True on successful sensor initialization.
            bool init(std::string const& portNameIn);

            /// \brief Stop the lidar (motor and scanning core).
            void stop();

            /// \brief Start the lidar (motor and scanning core).
            bool start();

            /// \brief Update sensor data.
            /// \details This function should be called frequently in a loop.
            void update();

            // For debugging - display of the last scan data.
            LidarPoint debuggingBuffer_[DEBUGGING_BUFFER_LENGTH];
            int debuggingBufferPosition_; ///< Position in the debugging buffer.

            std::deque<LidarPoint> detectedRobots_;    ///< Vector holding the detected robots, as fifo of robot angle.

        private:
            rp::standalone::rplidar::RPlidarDriver *lidar; ///< The lidar instance.
            int lidarMode_; ///< Scan mode for the lidar.

            double lastPointAngle_; ///< Angle of the last point, used to discard point in the wrong order.
            int blobNPoints_; ///< Number of points in current blob.
            double blobDistance_; ///< Current blob distance.
            double blobStartAngle_; ///< Start angle of the current blob.
            int nPointsOutsideBlob_; ///< Number of consecutive points outside of the current blob.
    };
#endif
