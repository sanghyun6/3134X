#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>

#include "../include/autonomous.h"
#include "motor-control.h"

void left_7ball()
{
  middle.set(true);
  // get 3ball
  descorer.set(true);
  lower_intake.spin(reverse, 100, pct); // keep spins until I stop
  driveTo(12, 2000, true, 6);           // drive little
  turnToAngle(330, 2000, true);         // turn left little
  driveTo(16, 2000, true, 6);           // arrive at the ball, wait a bit to take the ball

  // go to matchloader
  turnToAngle(230, 2000, true);
  driveTo(30.3, 2000, true, 6);

  turnToAngle(175.5, 2000, true);
  matchloader.set(true);
  driveTo(22, 4000, true, 4); // 4000
  // turnToAngle(180, 2000, true);
  wait(2000, msec);

  // go to the longgoal
  driveTo(-20, 2000, false, 6);
  lower_intake.spin(reverse, 150, pct);
  upper_intake.spin(fwd, 150, pct);
  wait(2000, msec);
}

void right_7ball()
{
  middle.set(true);
  // get 3ball
  descorer.set(true);
  lower_intake.spin(reverse, 100, pct); // keep spins until I stop
  driveTo(12, 2000, true, 6);           // drive little
  turnToAngle(30, 2000, true);          // turn left little
  driveTo(16, 2000, true, 6);           // arrive at the ball, wait a bit to take the ball

  // go to matchloader
  turnToAngle(130, 2000, true);
  driveTo(30.3, 2000, true, 6);

  turnToAngle(183, 2000, true);
  matchloader.set(true);
  driveTo(22, 4000, true, 6); // 4000
  turnToAngle(180, 2000, true);
  wait(2000, msec);

  // go to the longgoal
  driveTo(-20, 2000, false, 6);
  lower_intake.spin(reverse, 150, pct);
  upper_intake.spin(fwd, 150, pct);
  wait(2000, msec);
}
