// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static double ELEVATOR_HOME_POSITION = 0.0;
  public static double ELEVATOR_LEVEL_1 = 500.0;
  public static double ELEVATOR_LEVEL_2 = 1000.0;
  public static double ELEVATOR_LEVEL_3 = 1500.0;
  public static double ELEVATOR_LEVEL_4 = 2000.0;

  public static int ELEVATOR_LEFT_ID = 40;
  public static int ELEVATOR_RIGHT_ID = 41;

  public static double ELEVATOR_P = 0;
  public static double ELEVATOR_I = 0;
  public static double ELEVATOR_D = 0;

  public static double TURNING_OBJECT_DECTION_P = 0.05;
  public static double TURNING_OBJECT_DECTION_I = 0.0;
  public static double TURNING_OBJECT_DECTION_D = 0.0;

  public static double FORWARD_OBJECT_DECTION_P = 0.05;
  public static double FORWARD_OBJECT_DECTION_I = 0.0;
  public static double FORWARD_OBJECT_DECTION_D = 0.0;

  public static int ARM_ID = 33;
}
