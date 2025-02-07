// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.CustomAutoBuilder;

public class Field extends SubsystemBase {
  private Field2d m_field;
  private Drive m_drive;
  /** Creates a new field. */
  public Field(Drive drive) {
    m_drive = drive;
    m_field = CustomAutoBuilder.m_field;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_field.setRobotPose(m_drive.getPose());
  }
}
