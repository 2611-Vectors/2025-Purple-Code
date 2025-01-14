// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetection extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("Front Camera");
  PIDController controllerPID = new PIDController(0.05, 0, 0);
  Drive drive;
  CommandXboxController controller;
  double yaw;
  double tunableP = 0, tunableI = 0, tunableD = 0;
  /** Creates a new ObjectDetection. */
  public ObjectDetection(Drive drive, CommandXboxController controller) {
    this.drive = drive;
    this.controller = controller;
    SmartDashboard.putNumber("pValue", tunableP);
    SmartDashboard.putNumber("iValue", tunableI);
    SmartDashboard.putNumber("dValue", tunableD);
  }

  public double getRotation() {
    return controllerPID.calculate(yaw, 0);
  }

  // void rotateToGamePiece(DoubleSupplier yaw) {
  //   DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> getRotation(yaw.getAsDouble()));
  //   Logger.recordOutput("Vision/Rotation", controllerPID.calculate(yaw.getAsDouble(), 0));
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Logger.recordOutput("Vision/Area", -1);
    // Logger.recordOutput("Vision/Yaw", 0);
    // Logger.recordOutput("Vision/Pitch", 0);
    tunableP = SmartDashboard.getNumber("pValue", 0);

    tunableI = SmartDashboard.getNumber("iValue", 0);

    tunableD = SmartDashboard.getNumber("dValue", 0);
    controllerPID.setP(tunableP);
    controllerPID.setI(tunableI);
    controllerPID.setD(tunableD);
    Logger.recordOutput("Vision/ButtonB", controller.b().getAsBoolean());
    if (controller.b().getAsBoolean()) {
      // Getting target results from the camera
      List<PhotonPipelineResult> results = camera.getAllUnreadResults();
      // Looping throught the targets of the camera
      for (int i = 0; i < results.size(); i++) {
        PhotonPipelineResult result = results.get(i);
        // This is an list of all the properties of the i target
        List<PhotonTrackedTarget> targets = result.targets;
        // Looping to get the properties of the tracked target
        for (int p = 0; p < targets.size(); p++) {
          PhotonTrackedTarget tracked = targets.get(p);
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
    } else {
      yaw = 0;
    }
  }
}
/*




*/
