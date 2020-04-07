/*
* Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*
* This file and others part of the original PACMOD Game Control package were altered slightly to become the West Point IGVC Teleop system.
*/

#include "teleop/publish_control_factory.h"
#include "teleop/globals.h"
#include "teleop/startup_checks.h"

using namespace AS::Joystick;  // NOLINT

/*
 * Main method running the ROS Node
 */
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "teleop");
  ros::AsyncSpinner spinner(2);
  ros::NodeHandle priv("~");

  // Wait for time to be valid
  ros::Time::waitForValid();

  if (run_startup_checks_error(&priv) == true)
    return 0;

  // Check ROS params for board type
  int board_rev = 1;
  priv.getParam("pacmod_board_rev", board_rev);

  // Create an instance of the appropriate board type
  std::unique_ptr<PublishControl> board;

  try
  {
    board = PublishControlFactory::create(board_rev);
  }
  catch (const std::invalid_argument& ia)
  {
    ROS_ERROR("Invalid PACMod Board Version received. Board requested was %d", board_rev);
    return 0;
  }

  spinner.start();
  ros::waitForShutdown();

  return 0;
}
