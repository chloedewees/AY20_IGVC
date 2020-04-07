/*
* Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*
* This file and others part of the original PACMOD Game Control package were altered slightly to become the West Point IGVC teleop system.
*/

#ifndef PACMOD_GAME_CONTROL_PUBLISH_CONTROL_FACTORY_H
#define PACMOD_GAME_CONTROL_PUBLISH_CONTROL_FACTORY_H

#include "teleop/publish_control.h"
#include "teleop/publish_control_board_rev2.h"
#include "teleop/publish_control_board_rev3.h"

namespace AS
{
namespace Joystick
{

class PublishControlFactory
{
public:
    PublishControlFactory();
    static std::unique_ptr<PublishControl> create(int board_rev);
};

}  // namespace Joystick
}  // namespace AS

#endif  // PACMOD_GAME_CONTROL_PUBLISH_CONTROL_FACTORY_H
