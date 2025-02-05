// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTag2D extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("ReefTagCam");
  PIDController strafePID;
  public double yaw, area;
  public boolean bTarget = false;

  /** Creates a new AprilTag2D. */
  public AprilTag2D() {
    strafePID =
        new PIDController(
            Constants.FORWARD_OBJECT_DECTION_P,
            Constants.FORWARD_OBJECT_DECTION_I,
            Constants.FORWARD_OBJECT_DECTION_D);
  }

  public double getRawStrafe(double setpoint) {
    if (bTarget) {
      return MathUtil.clamp(strafePID.calculate(yaw, setpoint), -0.5, 0.5);
    }
    return 0;
  }

  public double getRawForward(double setpoint) {
    if (bTarget) {
      return MathUtil.clamp(strafePID.calculate(area, setpoint), -0.5, 0.5);
    }
    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Getting target results from the camera
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    /*
     * The Reason you would multiple results or AKA data from the pi is because the
     * pi can run faster than the robot so this makes sure we are considering all
     * the data
     */

    // Iterates through all the results
    for (PhotonPipelineResult result : results) {
      Logger.recordOutput("AprilTag/Targets", result.targets.size());
      if (result.targets.isEmpty()) {
        bTarget = false;
        yaw = 0;
      }
      // Iterates through the "targets" or game pieces the pi dectects
      for (PhotonTrackedTarget tracked : result.targets) {
        if (tracked.getFiducialId() == 6) {
          // Getting tracked target properties
          area = tracked.area;
          yaw = tracked.yaw;
          bTarget = true;
          double pitch = tracked.pitch;
          // Logging target properties
          Logger.recordOutput("AprilTag/Area", area);
          Logger.recordOutput("AprilTag/Yaw", yaw);
          Logger.recordOutput("AprilTag/Pitch", pitch);
        } else {
          bTarget = false;
          yaw = 0;
        }
      }
    }
  }
}
