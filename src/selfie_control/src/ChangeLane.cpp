#include "ChangeLane.h"

//constructor of ChangeLane class
ChangeLane::ChangeLane(float lane_width_param, float error_margin_param):
	lane_width(lane_width_param),
	error_margin(error_margin_param),
	target_position(-lane_width_param / 2)
{
}

ChangeLane::~ChangeLane()
{
}


//main fuction for computing value for target position
//input lane_width, error_margin, position_offset
//output taget_positions, left_turn_indicator, right_turn_indicator
//left +, right -
bool ChangeLane::process_target_position(void)
{
	//check at which lane you want to be on
	if (be_on_left_lane)
	{
		target_position = lane_width / 2;
		if (position_offset > lane_width / 2 - error_margin && position_offset < lane_width / 2 + error_margin)
		{
			maneuver_done = true;
		}
		else
		{
			maneuver_done = false;
			left_turn_indicator = true;
			right_turn_indicator = false;
		}
	}
	else
	{
		target_position = -lane_width / 2;
		if (position_offset > -lane_width / 2 - error_margin && position_offset < -lane_width / 2 + error_margin)
		{
			maneuver_done = true;
		}
		else
		{
			maneuver_done = false;
			left_turn_indicator = false;
			right_turn_indicator = true;
		}
	}

	if (maneuver_done)
	{
		left_turn_indicator = false;
		right_turn_indicator = false;
	}
	return maneuver_done;

}