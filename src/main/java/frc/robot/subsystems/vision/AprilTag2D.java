// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class AprilTag2D extends SubsystemBase {
  // Select this years field layout for April Tags
  private static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  // Select the Pose Strategy
  private static PoseStrategy poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

  // Define cameras and pose estimators
  private PhotonCamera reefTagCam = new PhotonCamera("ReefTagCam");

  private static final Transform3d REEF_TAG_CAM_TO_CENTER = new Transform3d(
    new Translation3d(0, 0, 0),
    new Rotation3d(Math.toDegrees(0),
    Math.toRadians(0), Math.toRadians(0)));

  PhotonPoseEstimator reefTagCamEstimator = new PhotonPoseEstimator(fieldLayout, poseStrategy, REEF_TAG_CAM_TO_CENTER);
  Drive m_Drive;

  /** Creates a new AprilTag2D. */
  public AprilTag2D(Drive drive) {
    m_Drive = drive;
  }

  @Override
  public void periodic() {
    List<PhotonPipelineResult> results = reefTagCam.getAllUnreadResults();
    for (PhotonPipelineResult result : results) {
      EstimatedRobotPose pose = reefTagCamEstimator.update(result, null, null).get();
      m_Drive.addVisionMeasurement(pose.estimatedPose.toPose2d(), result.getTimestampSeconds(), null);
    }
  }
}
