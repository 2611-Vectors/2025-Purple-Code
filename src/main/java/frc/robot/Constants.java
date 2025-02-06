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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  public static class AutonConstants {
    public static final Rotation2d START_ROTATION = Rotation2d.fromDegrees(0); // 180

    public static final Pose2d RIGHT_START = new Pose2d(8.0, 5.13, START_ROTATION);
    public static final Pose2d MIDDLE_START = new Pose2d(8.5, 1.9, START_ROTATION);
    public static final Pose2d LEFT_START = new Pose2d(8.5, 0.8, START_ROTATION);

    public static final Pose2d BACK_RIGHT_SCORE = new Pose2d(5.2, 2.7, Rotation2d.fromDegrees(120));
    public static final Pose2d BACK_LEFT_SCORE =
        new Pose2d(3.7, 2.7, new Rotation2d(Math.toRadians(60)));
    public static final Pose2d LEFT_SCORE = new Pose2d(3.0, 4.0, new Rotation2d(Math.toRadians(0)));
    public static final Pose2d TOP_LEFT_SCORE =
        new Pose2d(3.8, 5.3, new Rotation2d(Math.toRadians(-60)));
    public static final Pose2d TOP_RIGHT_SCORE =
        new Pose2d(5.3, 5.3, new Rotation2d(Math.toRadians(-120)));
    public static final Pose2d RIGHT_SCORE =
        new Pose2d(6.0, 4.0, new Rotation2d(Math.toRadians(180)));

    public static final Pose2d RIGHT_LOAD_STATION =
        new Pose2d(1.5, 6.6, Rotation2d.fromDegrees(-60));
    public static final Pose2d LEFT_LOAD_STATION = new Pose2d(1.5, 1.4, Rotation2d.fromDegrees(60));
  }
}
