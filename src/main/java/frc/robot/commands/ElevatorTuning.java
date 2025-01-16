// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.mechanism.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorTuning extends Command {
  Elevator m_Elevator;
  CommandXboxController operatorController;
  /** Creates a new ElevatorTuning. */
  public ElevatorTuning(Elevator m_Elevator, CommandXboxController operatorController) {
    this.m_Elevator = m_Elevator;
    this.operatorController = operatorController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (operatorController.y().getAsBoolean()) {
      m_Elevator.setElevatorPosition(0);
    } else if (operatorController.b().getAsBoolean()) {
      m_Elevator.setElevatorPosition(0.25);
    } else if (operatorController.a().getAsBoolean()) {
      m_Elevator.setElevatorPosition(0.5);
    } else if (operatorController.x().getAsBoolean()) {
      m_Elevator.setElevatorPosition(0.75);
    }
    // if (Math.abs(operatorController.getLeftY()) > 0.1) {
    // m_Elevator.setVoltage(operatorController.getLeftY() * 8);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
