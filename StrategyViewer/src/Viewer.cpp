/*
 * GameWindow.cpp
 *
 * Copyright 2016 Matthieu <matthieu@Matthieu-M4500>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 *
 */

#include "Viewer.h"
#include <iomanip>
#include <sstream>


// Build window from Glade.
Viewer::Viewer(BaseObjectType* cobject, const Glib::RefPtr<Gtk::Builder>& refGlade) : Gtk::Window(cobject)
{
    currentTrajectoryIndex_ = 0;
    // Associate key press to entry.
    Gtk::Widget *window;
    refGlade->get_widget("mainWindow", window);

    // Associate widgets.
    refGlade->get_widget("replayTime", replayTime);
    refGlade->get_widget("velocityLabel", velocityLabel);
    refGlade->get_widget("drawingArea", drawingArea);
    drawingArea->signal_draw().connect(sigc::mem_fun(this, &Viewer::redraw));

    // Configure slider based on viewerTrajectory.
    refGlade->get_widget("playbackSpeed", playbackSpeed);
    refGlade->get_widget("timeSlider", timeSlider_);
    timeSlider_->signal_change_value().connect(sigc::mem_fun(this, &Viewer::timeChanged));

    refGlade->get_widget("playButton", playButton);
    playButton->signal_toggled().connect(sigc::mem_fun(this, &Viewer::toggleReplayState));

    // Load images.
    tableImage = Gdk::Pixbuf::create_from_file("./config/table.png", -1, -1);
    trajectoryLength_ = 0;
}


Viewer::~Viewer()
{

}

void Viewer::addRobot(ViewerRobot robot)
{
    robots_.push_back(robot);
    // Re-equalize time vectors.
    for(auto r : robots_)
        trajectoryLength_ = std::max(trajectoryLength_, r.getTrajectoryLength());
    std::vector<ViewerRobot>::iterator it = robots_.begin();
    for(int i = 0; i < robots_.size(); i++)
    {
        (&(*it))->padTrajectory(trajectoryLength_);
        it ++;
    }
    // Update config.
    double endTime = TIMESTEP * (trajectoryLength_ - 1);

    Glib::RefPtr<Gtk::Adjustment> adjustment = timeSlider_->get_adjustment();
    adjustment->set_lower(0.0);
    adjustment->set_upper(endTime);
    adjustment->set_step_increment(TIMESTEP);
    timeSlider_->set_fill_level(endTime);
}

bool Viewer::redraw(const Cairo::RefPtr<Cairo::Context>& cr)
{
    // Check replay index.
    if(currentTrajectoryIndex_ < 0)
        currentTrajectoryIndex_ = 0;
    if(currentTrajectoryIndex_ >= trajectoryLength_)
        currentTrajectoryIndex_ = trajectoryLength_ - 1;


    // Put scaled table, keeping ratio.
    double heightToWidthRatio = tableImage->get_height() / (1.0 * tableImage->get_width());
    int widgetWidth = drawingArea->get_allocated_width();
    int widgetHeight = drawingArea->get_allocated_height();

    double newWidth = std::min(widgetWidth, static_cast<int>(widgetHeight / heightToWidthRatio));
    double newHeight = std::min(widgetHeight, static_cast<int>(heightToWidthRatio * widgetWidth));
    Gdk::Cairo::set_source_pixbuf(cr,
                                  tableImage->scale_simple(newWidth, newHeight, Gdk::INTERP_BILINEAR ),
                                  (widgetWidth - newWidth) / 2,
                                  (widgetHeight - newHeight) / 2);
    cr->paint();

    // Get table origin and scaling.
    double mmToCairo = newWidth / 4000.0;

    double originX = (widgetWidth - newWidth) / 2 + 500.0 * mmToCairo;
    double originY = (widgetHeight - newHeight) / 2 + 500.0 * mmToCairo;

    cr->translate(originX, originY);

    for(ViewerRobot robot : robots_)
        robot.draw(cr, mmToCairo, currentTrajectoryIndex_);



    // Update labels.
    double currentTime = currentTrajectoryIndex_ * TIMESTEP ;
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << currentTime << "/" << ((trajectoryLength_ - 1) * TIMESTEP);
    replayTime->set_text("Time: " + stream.str());
    timeSlider_->set_value(currentTime);

    ViewerTrajectoryPoint p = robots_.at(0).getViewerPoint(currentTrajectoryIndex_);
    std::stringstream velocityStream;
    velocityStream << "First velocity:\n\t" << std::fixed
        << std::setprecision(2) << p.linearVelocity / 1000.0 << " m/s\n\t"
        << std::setprecision(2) << p.angularVelocity << " rad/s";
    velocityLabel->set_text(velocityStream.str());


    return TRUE;
}

bool Viewer::timeChanged(Gtk::ScrollType scroll, double new_value)
{
    currentTrajectoryIndex_ = static_cast<int>(new_value / TIMESTEP);
    drawingArea->queue_draw();
    return true;
}


bool Viewer::playTrajectory()
{
    // Increment currentTrajectoryIndex_ based on time elapsed: increment as much as needed to catch back with real
    // clock.
    std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();
    double realElapsedTime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - lastTime_).count();
    lastTime_ = currentTime;

    currentTrajectoryIndex_ += static_cast<int>(std::floor(realElapsedTime * playbackSpeed->get_value() / TIMESTEP));
    // If end of the trajectory is reached, stop the connection.
    if(currentTrajectoryIndex_ >= trajectoryLength_)
        toggleReplayState();
    drawingArea->queue_draw();

    return true;
}

void Viewer::toggleReplayState()
{
    if(!replayConnection_.connected())
    {
        replayConnection_ = Glib::signal_timeout().connect(sigc::mem_fun(*this,&Viewer::playTrajectory), 50);
        lastTime_ = std::chrono::high_resolution_clock::now();
    }
     else
        replayConnection_.disconnect();
}
