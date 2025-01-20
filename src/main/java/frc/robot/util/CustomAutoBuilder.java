// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Add your docs here. */
public class CustomAutoBuilder {
  private static final Rotation2d START_ROTATION = Rotation2d.fromDegrees(180); // 180

  private static final Pose2d RIGHT_START = new Pose2d(8.5, 3.0, START_ROTATION);
  private static final Pose2d MIDDLE_START = new Pose2d(8.5, 1.9, START_ROTATION);
  private static final Pose2d LEFT_START = new Pose2d(8.5, 0.8, START_ROTATION);

  private static final Pose2d BACK_RIGHT_SCORE = new Pose2d(5.2, 2.7, Rotation2d.fromDegrees(120));
  private static final Pose2d BACK_LEFT_SCORE =
      new Pose2d(3.7, 2.7, new Rotation2d(Math.toRadians(60)));
  private static final Pose2d LEFT_SCORE = new Pose2d(3.0, 4.0, new Rotation2d(Math.toRadians(0)));
  private static final Pose2d TOP_LEFT_SCORE =
      new Pose2d(3.8, 5.3, new Rotation2d(Math.toRadians(-60)));
  private static final Pose2d TOP_RIGHT_SCORE =
      new Pose2d(5.3, 5.3, new Rotation2d(Math.toRadians(-120)));
  private static final Pose2d RIGHT_SCORE =
      new Pose2d(6.0, 4.0, new Rotation2d(Math.toRadians(180)));

  public static LoggedDashboardChooser<Pose2d> startChooser;
  public static LoggedDashboardChooser<Pose2d> scoreOneChooser;
  public static Field2d m_field = new Field2d();

  public static void chooserBuilder() {
    startChooser = new LoggedDashboardChooser<Pose2d>("Start Position");

    startChooser.addOption("Right", RIGHT_START);
    startChooser.addOption("Middle", MIDDLE_START);
    startChooser.addOption("Left", LEFT_START);

    startChooser.addDefaultOption("Right", RIGHT_START);

    scoreOneChooser = new LoggedDashboardChooser<Pose2d>("Score 1 Position");

    scoreOneChooser.addOption("Back Right", BACK_RIGHT_SCORE);
    scoreOneChooser.addOption("Back Left", BACK_LEFT_SCORE);
    scoreOneChooser.addOption("Left", LEFT_SCORE);
    scoreOneChooser.addOption("Top Left", TOP_LEFT_SCORE);
    scoreOneChooser.addOption("Top Right", TOP_RIGHT_SCORE);
    scoreOneChooser.addOption("Right", RIGHT_SCORE);

    scoreOneChooser.addDefaultOption("Back Right", BACK_RIGHT_SCORE);
    SmartDashboard.putData(m_field);
  }

  public static Command autonPath;

  public static void update() {
    PathConstraints constraints = new PathConstraints(1.0, 0.75, 2 * Math.PI, 4 * Math.PI);
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(startChooser.get(), scoreOneChooser.get());

    // System.out.println("Way point 0");
    // System.out.println(waypoints.get(0).prevControl());
    // System.out.println(waypoints.get(0).anchor());
    // System.out.println(waypoints.get(0).nextControl());

    // System.out.println("Way point 1");
    // System.out.println(waypoints.get(1).prevControl());
    // System.out.println(waypoints.get(1).anchor());
    // System.out.println(waypoints.get(1).nextControl());

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned
            // paths, so can
            // be null for on-the-fly paths.
            new GoalEndState(
                0.0,
                Rotation2d.fromDegrees(
                    -90)) // Goal end state. You can set a holonomic rotation here. If
            // using a differential drivetrain, the rotation will have no
            // effect.
            );

    Pose2d[] posesPath1 = path.getPathPoses().toArray(new Pose2d[path.getPathPoses().size()]);
    m_field.getObject("traj").setPoses(posesPath1);
    // m_field.setRobotPose(posesPath1[posesPath1.length - 1]);

    // SmartDashboard.put("Path", path.getPathPoses().toArray(new
    // Pose2d[path.getPathPoses().size()]));
    autonPath = AutoBuilder.followPath(path);
  }

  public static Command getAutonCommand() {
    return autonPath;
  }
}
