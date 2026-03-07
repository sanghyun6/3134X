#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>

#include "../include/autonomous.h"
#include "motor-control.h"

void newauton()
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
  driveTo(27.2, 2000, true, 6);

  turnToAngle(174.5, 2000, true);
  driveTo(-15, 2000, true, 6);
  lower_intake.spin(reverse, 150, pct);
  upper_intake.spin(fwd, 150, pct);
  wait(2000, msec);
  driveTo(12, 2000, true, 6);
  turnToAngle(270, 2000, true);

  driveTo(10, 2000, true, 6);
  lower_intake.stop();
  upper_intake.stop();

  turnToAngle(0, 2000, true);

  driveTo(30, 2000, true, 6);
  descorer.set(false);
  driveTo(5.5, 2000, true, 6);
}

void firstball()
{
  middle.set(true);
  // get 3ball
  descorer.set(true);
  lower_intake.spin(reverse, 120, pct); // keep spins until I stop
  driveTo(10, 2000, true, 6);           // drive little
  turnToAngle(330, 2000, true);         // turn left little
  driveTo(18, 2000, true, 4);           // arrive at the ball, wait a bit to take the ball
}

void left_finalauton()
{
  middle.set(false);
  // get 3ball
  descorer.set(true);
  lower_intake.spin(reverse, 100, pct); // keep spins until I stop
  driveTo(25, 1000, true, 6);           // drive little
  turnToAngle(330, 500, true);          // turn left little
  driveTo(16, 1000, true, 3);           // arrive at the ball, wait a bit to take the ball

  // go to matchloader
  turnToAngle(230, 1000, true);
  driveTo(28.5, 1000, true, 6);

  // go to the longgoal
  turnToAngle(180, 400, true); // prev 175.5

  driveTo(-24, 1000, true, 6);
  lower_intake.spin(reverse, 150, pct);
  upper_intake.spin(fwd, 150, pct);
  wait(2000, msec);

  lower_intake.stop();
  upper_intake.stop();

  driveTo(12, 500, true, 6);
  turnToAngle(90, 500, true);
  driveTo(7.3, 500, true, 6);
  turnToAngle(180, 500, true);

  driveTo(-8, 2000, true, 6);
  descorer.set(false);
  driveTo(-38, 2000, true, 6);
}

void right_finalauton()
{
  middle.set(false);
  // get 3ball
  descorer.set(true);
  lower_intake.spin(reverse, 100, pct); // keep spins until I stop
  driveTo(25, 1000, true, 6);           // drive little
  turnToAngle(30, 500, true);           // turn left little
  driveTo(16, 2000, true, 3);           // arrive at the ball, wait a bit to take the ball

  // go to matchloader
  turnToAngle(130, 1000, true);
  driveTo(28.5, 1000, true, 6);

  // go to the longgoal
  turnToAngle(180, 400, true); // prev 175.5

  driveTo(-24, 1000, true, 6);
  lower_intake.spin(reverse, 150, pct);
  upper_intake.spin(fwd, 150, pct);
  wait(2000, msec);

  lower_intake.stop();
  upper_intake.stop();

  driveTo(12, 500, true, 6);
  turnToAngle(270, 500, true);
  driveTo(-7, 500, true, 6);
  turnToAngle(180, 500, true);

  driveTo(-7, 2000, true, 6);
  descorer.set(false);
  driveTo(-38, 2000, true, 6);
}

void newnewauton()
{
  middle.set(true);
  // get 3ball
  descorer.set(true);
  lower_intake.spin(reverse, 100, pct); // keep spins until I stop
  driveTo(10, 2000, true, 6);           // drive little
  turnToAngle(330, 2000, true);         // turn left little
  driveTo(18, 2000, true, 6);           // arrive at the ball, wait a bit to take the ball

  // go to matchloader
  turnToAngle(230, 2000, true);
  driveTo(31.2, 2000, true, 6);

  turnToAngle(174.5, 2000, true);
  driveTo(-15, 2000, true, 6);
  lower_intake.spin(reverse, 150, pct);
  upper_intake.spin(fwd, 150, pct);
  wait(2000, msec);

  lower_intake.stop();
  upper_intake.stop();

  driveTo(12, 2000, true, 6);
  turnToAngle(90, 2000, true);
  driveTo(7, 2000, true, 6);
  turnToAngle(180, 2000, true);

  driveTo(-8, 2000, true, 6);
  descorer.set(false);
  driveTo(-18, 2000, true, 6);
}

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
  driveTo(31.2, 2000, true, 6);

  // go to the longgoal
  turnToAngle(174.5, 2000, true); // prev 175.5
  matchloader.set(true);
  driveTo(22, 4000, true, 4);
  // turnToAngle(180, 2000, true);   maybe put it because of the angle

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

// get 3ball

// go to the match loader

//