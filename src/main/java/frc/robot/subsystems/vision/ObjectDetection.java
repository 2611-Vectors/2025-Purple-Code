// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.TunablePIDController;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetection extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("Front Camera");
  TunablePIDController controllerPID;
  Drive drive;
  CommandXboxController controller;
  double yaw;

  /** Creates a new ObjectDetection. */
  public ObjectDetection(Drive drive, CommandXboxController controller) {
    this.drive = drive;
    this.controller = controller;
    controllerPID =
        new TunablePIDController(
            Constants.OBJECT_DECTION_P,
            Constants.OBJECT_DECTION_I,
            Constants.OBJECT_DECTION_D,
            "/Tuning/ObjectionDection/");
  }

  public double getRotation() {
    return controllerPID.calculate(yaw, 0);
  }

  public void updateYaw() {
    // Getting target results from the camera
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    /*
     * The Reason you would multiple results or AKA data from the pi is because the
     * pi can run faster than the robot so this makes sure we are considering all
     * the data
     */

    // Iterates through all the results
    for (PhotonPipelineResult result : results) {
      // Iterates through the "targets" or game pieces the pi dectects
      for (PhotonTrackedTarget tracked : result.targets) {
        // Getting tracked target properties
        double area = tracked.area;
        yaw = tracked.yaw;
        double pitch = tracked.pitch;
        // Logging target properties
        Logger.recordOutput("Vision/Area", area);
        Logger.recordOutput("Vision/Yaw", yaw);
        Logger.recordOutput("Vision/Pitch", pitch);
      }
    }
    // https://www.desmos.com/3d/m5tc4yloem
    // Next step I want you to do is only rotate to the closest target
    // Hint:
    // distance = yaw ^ 2 + sign(pitch)*(pitch/cos(CAMERA_ANGLE)) ^ 2

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    controllerPID.update();
    updateYaw();
  }
}
