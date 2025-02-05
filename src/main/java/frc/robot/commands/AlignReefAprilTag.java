// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.AprilTag2D;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignReefAprilTag extends Command {
  Drive drive;
  AprilTag2D m_AprilTag2D;
  boolean isLeft;

  /** Creates a new AlignReefAprilTag. */
  public AlignReefAprilTag(Drive drive, AprilTag2D m_AprilTag2D, boolean isLeft) {
    this.drive = drive;
    this.m_AprilTag2D = m_AprilTag2D;
    this.isLeft = isLeft;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double strafeSetpoint = isLeft ? 17 : -15;

    DriveCommands.robotRelativeDrive(
        drive, -m_AprilTag2D.getRawForward(7), -m_AprilTag2D.getRawStrafe(strafeSetpoint), 0);
    SmartDashboard.putNumber("Raw Forward: ", m_AprilTag2D.getRawForward(4.7));
    SmartDashboard.putNumber("Strafe Forward: ", m_AprilTag2D.getRawStrafe(strafeSetpoint));
    SmartDashboard.putNumber("Setpoint: ", strafeSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveCommands.robotRelativeDrive(drive, 0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
