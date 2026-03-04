#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"
#include "../custom/include/robot-config.h"

// Modify autonomous, driver, or pre-auton code below

void runAutonomous()
{
  int auton_selected = 8;
  switch (auton_selected)
  {
  case 1:
    exampleAuton();
    break;
  case 2:
    exampleAuton2();
    break;
  case 3:
    redGoalRush();
    break;
  case 4:
    left_7ball();
    break;
  case 5:
    right_7ball();
    break;
  case 6:
    newauton();
    break;
  case 7:
    newnewauton();
    break;
  case 8:
    left_finalauton();
    break;
  case 9:
    right_finalauton();
    break;
  }
}

// controller_1 input variables (snake_case)
int ch1, ch2, ch3, ch4;
bool l1, l2, r1, r2;
bool button_a, button_b, button_x, button_y;
bool button_up_arrow, button_down_arrow, button_left_arrow, button_right_arrow;
int chassis_flag = 0;

void runDriver()
{
  // code - boolean for newmatix
  bool matchloader_state = false;
  bool middle_state = false;
  bool descorer_state = false;

  stopChassis(coast);
  heading_correction = false;
  while (true)
  {
    // [-100, 100] for controller stick axis values
    ch1 = controller_1.Axis1.value();
    ch2 = controller_1.Axis2.value();
    ch3 = controller_1.Axis3.value();
    ch4 = controller_1.Axis4.value();

    // true/false for controller button presses
    l1 = controller_1.ButtonL1.pressing();
    l2 = controller_1.ButtonL2.pressing();
    r1 = controller_1.ButtonR1.pressing();
    r2 = controller_1.ButtonR2.pressing();
    button_a = controller_1.ButtonA.pressing();
    button_b = controller_1.ButtonB.pressing();
    button_x = controller_1.ButtonX.pressing();
    button_y = controller_1.ButtonY.pressing();
    button_up_arrow = controller_1.ButtonUp.pressing();
    button_down_arrow = controller_1.ButtonDown.pressing();
    button_left_arrow = controller_1.ButtonLeft.pressing();
    button_right_arrow = controller_1.ButtonRight.pressing();

    // lee's code
    if (l1) // score (intake the ball 아래에서 위)
    {
      lower_intake.spin(reverse, 100, pct);
      upper_intake.spin(fwd, 100, pct);
    }
    else if (l2) // descore (위에서 아래)
    {
      lower_intake.spin(fwd, 100, pct);
      upper_intake.spin(reverse, 100, pct);
    }
    else if (r1) // keep the ball
    {
      lower_intake.spin(reverse, 100, pct);
      upper_intake.spin(reverse, 100, pct);
    }
    else if (button_down_arrow) // matchloader
    {
      matchloader.set(matchloader_state);
      matchloader_state = !matchloader_state;
      wait(300, msec);
    }
    else if (button_b) // middle
    {
      middle.set(middle_state);
      middle_state = !middle_state;
      wait(300, msec);
    }
    else if (button_up_arrow) // descorer
    {
      descorer.set(descorer_state);
      descorer_state = !descorer_state;
      wait(300, msec);
    }
    else
    {
      lower_intake.stop(coast);
      upper_intake.stop(coast);
    }

    // default tank drive or replace it with your preferred driver code here:
    // driveChassis(ch3 * 0.12, ch2 * 0.12); // tank mode
    // lee's code
    driveChassis(ch3 * 0.12 + ch1 * 0.08, ch3 * 0.12 - ch1 * 0.08); // drive mode
    wait(10, msec);
  }
}

void runPreAutonomous()
{
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // Calibrate inertial sensor
  inertial_sensor.calibrate();

  // Wait for the Inertial Sensor to calibrate
  while (inertial_sensor.isCalibrating())
  {
    wait(10, msec);
  }

  double current_heading = inertial_sensor.heading();
  Brain.Screen.print(current_heading);

  // odom tracking
  resetChassis();
  if (using_horizontal_tracker && using_vertical_tracker)
  {
    thread odom = thread(trackXYOdomWheel);
  }
  else if (using_horizontal_tracker)
  {
    thread odom = thread(trackXOdomWheel);
  }
  else if (using_vertical_tracker)
  {
    thread odom = thread(trackYOdomWheel);
  }
  else
  {
    thread odom = thread(trackNoOdomWheel);
  }
}