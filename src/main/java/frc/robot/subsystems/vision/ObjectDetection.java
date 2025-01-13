// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetection extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("Front Camera");
  /** Creates a new ObjectDetection. */
  public ObjectDetection() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Vision/Area", -1);
    Logger.recordOutput("Vision/Yaw", 0);
    Logger.recordOutput("Vision/Pitch", 0);
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
        double yaw = tracked.yaw;
        double pitch = tracked.pitch;
        // Logging target properties
        Logger.recordOutput("Vision/Area", area);
        Logger.recordOutput("Vision/Yaw", yaw);
        Logger.recordOutput("Vision/Pitch", pitch);
      }
    }
  }
}
/*




*/
