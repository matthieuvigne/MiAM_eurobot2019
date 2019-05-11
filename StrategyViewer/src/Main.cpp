/*
 * Main.cpp
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


#include <gtkmm/application.h>
#include <cstdlib>
#include <iostream>
#include <MiAMEurobot/Logger.h>

#include "Viewer.h"
#include "Strategy.h"

Glib::RefPtr<Gtk::Application> app;

int main (int argc, char *argv[])
{
    srand (time(NULL));
    app = Gtk::Application::create(argc, argv);

    //load main window layout from glade
    auto refBuilder = Gtk::Builder::create();
    try
    {
        refBuilder->add_from_file("./config/MainWindow.glade");
    }
    catch(const Glib::FileError& ex)
    {
        std::cerr << "FileError: " << ex.what() << std::endl;
    }
    catch(const Glib::MarkupError& ex)
    {
        std::cerr << "MarkupError: " << ex.what() << std::endl;
    }
    catch(const Gtk::BuilderError& ex)
    {
        std::cerr << "BuilderError: " << ex.what() << std::endl;
    }

    // Create robots.
    ViewerRobot mainRobot("./config/mainRobot.png", mainRobotStrategy);

    ViewerRobot secondaryRobot("./config/secondaryRobot.png", secondaryRobotStrategy, 0, 0, 1);

    // Create handler.
    Viewer *viewer = nullptr;
    refBuilder->get_widget_derived("mainWindow", viewer);
    viewer->addRobot(mainRobot);
    viewer->addRobot(secondaryRobot);

    return app->run(*viewer);
}


