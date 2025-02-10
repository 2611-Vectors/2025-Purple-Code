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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
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

  public static double TURNING_OBJECT_DECTION_P = 0.02;
  public static double TURNING_OBJECT_DECTION_I = 0.0;
  public static double TURNING_OBJECT_DECTION_D = 0.0;

  public static double FORWARD_OBJECT_DECTION_P = 0.25;
  public static double FORWARD_OBJECT_DECTION_I = 0.0;
  public static double FORWARD_OBJECT_DECTION_D = 0.0;

  public static int ARM_ID = 33;

  public static class VisionConstants {
    // Apriltag Field Layout
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    // Name of the PhotonVision Reef Camera
    public static String reefCamName = "ReefTagCam";

    // Position of the PhotonVision Reef Camera
    public static Transform3d robotToReefCam =
        new Transform3d(
            Units.inchesToMeters(-12.0),
            Units.inchesToMeters(0.0),
            Units.inchesToMeters(7.75),
            new Rotation3d(0.0, 0.0, Math.toRadians(180)));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.1;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {1.0};

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available
  }

  public static class AutonConstants {
    public static final Rotation2d START_ROTATION = Rotation2d.fromDegrees(0); // 180

    public static final Pose2d START_LEFT = new Pose2d(8.0, 7.29, START_ROTATION);
    public static final Pose2d START_CENTER = new Pose2d(8.0, 6.20, START_ROTATION);
    public static final Pose2d START_RIGHT = new Pose2d(8.0, 5.13, START_ROTATION);

    public static final Pose2d AB = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(0));
    public static final Pose2d CD = new Pose2d(3.7, 2.7, Rotation2d.fromDegrees(60));
    public static final Pose2d EF = new Pose2d(5.2, 2.7, Rotation2d.fromDegrees(120));
    public static final Pose2d GH = new Pose2d(6.0, 4.0, Rotation2d.fromDegrees(180));
    public static final Pose2d IJ = new Pose2d(5.3, 5.3, Rotation2d.fromDegrees(-120));
    public static final Pose2d KL = new Pose2d(3.8, 5.3, Rotation2d.fromDegrees(-60));

    public static final Pose2d R1 = new Pose2d(2, 6.1, Rotation2d.fromDegrees(-150));
    public static final Pose2d R0 = new Pose2d(1.5, 1.4, Rotation2d.fromDegrees(60));

    public static final double MAX_VELOCITY = 1; // 5.1
    public static final double MAX_ACCELERATION = 0.75; // 2.9
  }
}
