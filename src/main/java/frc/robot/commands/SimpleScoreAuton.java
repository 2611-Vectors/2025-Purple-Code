// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.util.CustomAutoBuilder;
import frc.robot.util.MechanismSimulator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleScoreAuton extends SequentialCommandGroup {
  /** Creates a new SimpleScoreAuton. */
  public SimpleScoreAuton() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(() -> MechanismSimulator.updateArm(90)),
        Commands.runOnce(() -> MechanismSimulator.updateElevator(0.6)),
        new WaitCommand(2),
        Commands.parallel(
            Commands.sequence(
                new WaitCommand(4),
                Commands.runOnce(() -> MechanismSimulator.updateArm(0)),
                Commands.runOnce(() -> MechanismSimulator.updateElevator(1.65))),
            CustomAutoBuilder.getAutonCommand()),
        new WaitCommand(2),
        Commands.runOnce(() -> MechanismSimulator.updateArm(-90)));
  }
}
