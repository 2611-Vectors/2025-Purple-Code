// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.mechanism;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TunablePIDController;

public class Elevator extends SubsystemBase {
  private TalonFX leftElevatorMotor, rightElevatorMotor;
  TunablePIDController controllerPID;

  /** Creates a new Elevator. */
  public Elevator() {
    leftElevatorMotor = new TalonFX(Constants.ELEVATOR_LEFT_ID);
    rightElevatorMotor = new TalonFX(Constants.ELEVATOR_RIGHT_ID);
    rightElevatorMotor.setInverted(true);

    controllerPID =
        new TunablePIDController(
            Constants.ELEVATOR_P,
            Constants.ELEVATOR_I,
            Constants.ELEVATOR_D,
            "/Tuning/ObjectionDection/");
  }

  public void setElevatorPower(double power) {
    rightElevatorMotor.set(power);
    leftElevatorMotor.set(power);
  }

  public void setElevatorPosition(double position) {
    setElevatorPower(controllerPID.calculate(getElevatorPosition(), position));
  }

  public double getElevatorPosition() {
    return leftElevatorMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
