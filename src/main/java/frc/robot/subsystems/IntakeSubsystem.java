// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax intakeAngle;
  private CANSparkMax intakeMotor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeAngle = new CANSparkMax(Constants.IntakeConstants.kIntakeAngleCanId, MotorType.kBrushless);
    intakeMotor = new CANSparkMax(Constants.IntakeConstants.kIntakeMotorCanId, MotorType.kBrushless);
  }

  public void setIntakeAnglePower(double power){
    intakeAngle.set(power);
  }

  public void setIntakeMotorPower(double power){
    intakeMotor.set(power);
  }

  public void runIntake() {
    intakeMotor.set(-.6);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command getRunIntakeCommand(double power) {
    return this.startEnd(
      () -> {
        setIntakeMotorPower(power);
      }, 
      () -> {
        setIntakeMotorPower(0);
      });
  }

}
