#include "ViewerRobot.h"
#include <string>


ViewerRobot::ViewerRobot(std::string const& imageFileName, double const& r, double const& g, double const& b):
    r_(r),
    g_(g),
    b_(b)
{
    image_ = Gdk::Pixbuf::create_from_file(imageFileName, -1, -1);
}


RobotPosition ViewerRobot::getPosition()
{
    return trajectory_.back().position;
}


void ViewerRobot::followTrajectory(miam::trajectory::Trajectory *traj)
{
    double currentTrajectoryTime = 0.0;
    ViewerTrajectoryPoint viewerPoint = trajectory_.back();

    miam::trajectory::TrajectoryPoint currentPoint;
    while(currentTrajectoryTime < traj->getDuration())
    {
        currentPoint = traj->getCurrentPoint(currentTrajectoryTime);

        viewerPoint.time += TIMESTEP;
        viewerPoint.position = currentPoint.position;
        viewerPoint.linearVelocity = currentPoint.linearVelocity;
        viewerPoint.angularVelocity = currentPoint.angularVelocity;

        trajectory_.push_back(viewerPoint);
        currentTrajectoryTime += TIMESTEP;
    }
}


void ViewerRobot::followTrajectory(std::vector<std::shared_ptr<miam::trajectory::Trajectory>>  trajectories)
{
    for(std::shared_ptr<miam::trajectory::Trajectory> t: trajectories)
        followTrajectory(t.get());
}


void ViewerRobot::setPosition(RobotPosition const& position)
{
    ViewerTrajectoryPoint p;
    if (trajectory_.empty())
        p.time = 0.0;
    else
        p.time = trajectory_.back().time + TIMESTEP;
    p.position = position;
    p.linearVelocity = 0.0;
    p.angularVelocity = 0.0;
    trajectory_.push_back(p);
}

void ViewerRobot::draw(const Cairo::RefPtr<Cairo::Context>& cr, double const& mmToCairo, int const& currentIndex)
{
    // Draw robot at current point.
    ViewerTrajectoryPoint currentViewerPoint = trajectory_.at(currentIndex);
    RobotPosition p = currentViewerPoint.position;

    double robotImageSize = 500 * mmToCairo;
    double robotOriginX = p.x * mmToCairo;
    double robotOriginY = (2000 - p.y) * mmToCairo;

    cr->save();
    cr->translate(robotOriginX, robotOriginY);
    // Minus sign: indirect convention is used in Cairo.
    cr->rotate(-p.theta);
    Gdk::Cairo::set_source_pixbuf(cr,
                                  image_->scale_simple(robotImageSize, robotImageSize, Gdk::INTERP_BILINEAR ),
                                  -250 * mmToCairo,
                                  -250 * mmToCairo);
    cr->paint();
    cr->restore();

    // Draw robot path.
    double pointX =  mmToCairo * trajectory_.at(0).position.x;
    double pointY =  mmToCairo * (2000 - trajectory_.at(0).position.y);

    cr->move_to(pointX, pointY);
    for(int i = 0; i < currentIndex; i+=10)
    {
        pointX =  mmToCairo * trajectory_.at(i).position.x;
        pointY =  mmToCairo * (2000 - trajectory_.at(i).position.y);
        cr->line_to(pointX, pointY);
    }
    cr->set_source_rgb(1.0, 1.0, 1.0);
    cr->set_line_width(5.0);
    cr->stroke_preserve();
    cr->set_source_rgb(r_, g_, b_);
    cr->set_line_width(2.0);
    cr->stroke();
}


int ViewerRobot::getTrajectoryLength()
{
   return trajectory_.size();
}

void ViewerRobot::padTrajectory(int const& desiredLength)
{
    ViewerTrajectoryPoint lastPoint = trajectory_.back();
    lastPoint.linearVelocity = 0.0;
    lastPoint.angularVelocity = 0.0;
    while(trajectory_.size() < desiredLength)
    {
        lastPoint.time += TIMESTEP;
        trajectory_.push_back(lastPoint);
    }
}

ViewerTrajectoryPoint ViewerRobot::getViewerPoint(int const& index)
{
    return trajectory_.at(index);
}
