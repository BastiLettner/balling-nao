//
// Created by hrs_b on 31.01.19.
//

#include "brain.h"


Brain::Brain(ros::NodeHandle &node_handle):
    _motion(node_handle),
    _vision(node_handle),
    _tactile(node_handle),
    _speech(node_handle),
    _search(_motion)
{

}
