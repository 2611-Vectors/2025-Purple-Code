// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanism;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MechanismSimulator;
import frc.robot.util.PhoenixUtil;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private TalonFX armMotor;
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

  /** Creates a new Arm. */
  public Arm() {
    armMotor = new TalonFX(Constants.ARM_ID);
    configMotors(armMotor, false);
  }

  public void configMotors(TalonFX motor, boolean inverted) {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 5; // An error of 1 rotation results in 2.4 V output
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

  public void setVoltage(double voltage) {
    armMotor.setVoltage(voltage);
  }

  public void setArmAngle(double angle) {
    Logger.recordOutput("Elevator/Setpoint", angle);
    // setElevatorPower(controllerPID.calculate(getElevatorPosition(), position));
    armMotor.setControl(m_positionVoltage.withPosition(angle));
    MechanismSimulator.updateArm(angle);
  }

  public double getArmAngle() {
    return armMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
