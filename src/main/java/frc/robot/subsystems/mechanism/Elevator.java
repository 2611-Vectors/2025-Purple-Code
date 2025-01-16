// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanism;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.TunablePIDController;

public class Elevator extends SubsystemBase {
  private TalonFX leftElevatorMotor, rightElevatorMotor;
  TunablePIDController controllerPID;
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
  DigitalInput limitSwitch;

  /** Creates a new Elevator. */
  public Elevator() {
    leftElevatorMotor = new TalonFX(Constants.ELEVATOR_LEFT_ID);
    rightElevatorMotor = new TalonFX(Constants.ELEVATOR_RIGHT_ID);
    rightElevatorMotor.setInverted(true);

    limitSwitch = new DigitalInput(0);

    // configMotors(leftElevatorMotor, false);
    // configMotors(rightElevatorMotor, true);

    controllerPID =
        new TunablePIDController(
            Constants.ELEVATOR_P,
            Constants.ELEVATOR_I,
            Constants.ELEVATOR_D,
            "/Tuning/ObjectionDection/");
  }

  public void configMotors(TalonFX motor, boolean inverted) {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

    configs.Slot0.kS = 0; // Baseline voltage required to overcome static forces like friction
    configs.Slot0.kG = 0; // Voltage to overcome gravity
    configs.MotorOutput.Inverted =
        inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;

    // Peak output of 8 V
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    configs.CurrentLimits.StatorCurrentLimit = 60;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;

    // Example on how you would do break mode / coast mode
    // configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(configs, 0.25));
    motor.setPosition(0);
  }

  public void setElevatorPower(double power) {
    rightElevatorMotor.set(power);
    leftElevatorMotor.set(power);
  }

  public void setVoltage(double voltage) {
    rightElevatorMotor.setVoltage(voltage);
    leftElevatorMotor.setVoltage(voltage);
  }

  public void setElevatorPosition(double position) {
    setElevatorPower(controllerPID.calculate(getElevatorPosition(), position));
    // leftElevatorMotor.setControl(m_positionVoltage.withPosition(position));
    // rightElevatorMotor.setControl(m_positionVoltage.withPosition(position));
  }

  public double getElevatorPosition() {
    return leftElevatorMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    controllerPID.update();
  }
}
