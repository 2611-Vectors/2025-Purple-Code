// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.ObjectDetection;
import frc.robot.util.CustomAutoBuilder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralDetectionTest extends SequentialCommandGroup {
  /** Creates a new CoralDetectionTest. */
  public CoralDetectionTest(Drive drive, ObjectDetection m_ObjectDetection) {
    Command[] drivePaths = CustomAutoBuilder.getDrivePaths();
    Command autonPath = drivePaths[0];
    for (int i = 1; i < drivePaths.length; i++) {
      if (i % 2 == 1) {
        autonPath =
            Commands.sequence(
                autonPath,
                drivePaths[i],
                Commands.deadline(
                    new WaitCommand(3),
                    DriveCommands.robotRelativeDrive(
                        drive,
                        () -> 0,
                        () -> -m_ObjectDetection.getRawForward(),
                        () -> m_ObjectDetection.getRotation())));
      } else {
        autonPath = Commands.sequence(autonPath, drivePaths[i]);
      }
    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(autonPath);
  }
}
