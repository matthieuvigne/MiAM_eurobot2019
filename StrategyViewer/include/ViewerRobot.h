// ViewerRobot.h
// Represent a robot in the viewer, following a trajectory.

#ifndef VIEWER_ROBOT_H
    #define VIEWER_ROBOT_H

    #include <gtkmm.h>
    #include <iostream>
    #include <MiAMEurobot/trajectory/Trajectory.h>

    // Replay timestep.
    static double const TIMESTEP = 0.01;

    using miam::RobotPosition;


    struct ViewerTrajectoryPoint{
        double time;
        RobotPosition position;
        double linearVelocity;
        double angularVelocity;

        ViewerTrajectoryPoint():
            time(0.0),
            position(),
            linearVelocity(0.0),
            angularVelocity(0.0)
        {}
    };

    class ViewerRobot
    {
        public:
            /// \brief Constructor.
            ViewerRobot(std::string const& imageFileName, double const& r = 1.0, double const& g = 0.0, double const& b = 0.0);

            /// \brief Get current robot position.
            RobotPosition getPosition();

            /// \brief Get viewer position.
            ViewerTrajectoryPoint getViewerPoint(int const& index);

            /// \brief Mock trajectory following.
            void followTrajectory(miam::trajectory::Trajectory * traj);
            void followTrajectory(std::vector<std::shared_ptr<miam::trajectory::Trajectory>>  trajectories);

            /// \brief Set robot position (velocity is set to 0, this is mostly done for position reset and init).
            void setPosition(RobotPosition const& position);

            /// \brief Draw the robot and trajectory on the surface.
            void draw(const Cairo::RefPtr<Cairo::Context>& cr, double const& mmToCairo, int const& currentIndex);

            /// \brief Get length (i.e. number of samples) of the trajectory.
            int getTrajectoryLength();

            /// \brief Pad trajectory with the last point (zero velocity) to the desired length.
            void padTrajectory(int const& desiredLength);

        private:
            std::vector<ViewerTrajectoryPoint> trajectory_;
            Glib::RefPtr<Gdk::Pixbuf> image_;

            double r_;  ///< Trajectory color.
            double g_;  ///< Trajectory color.
            double b_;  ///< Trajectory color.
    };

#endif
