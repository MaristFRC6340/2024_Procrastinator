// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax intakeAngle;

  private SparkPIDController intakeAnglePID;
  private RelativeEncoder intakeAngleRelativeEncoder;
  private CANSparkMax intakeMotor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeAngle = new CANSparkMax(Constants.IntakeConstants.kIntakeAngleCanId, MotorType.kBrushless);
    intakeMotor = new CANSparkMax(Constants.IntakeConstants.kIntakeMotorCanId, MotorType.kBrushless);

    //Set Current Limits
    intakeAngle.setSmartCurrentLimit(40);
    intakeMotor.setSmartCurrentLimit(40);
    // getting encoder
    intakeAngleRelativeEncoder = intakeAngle.getEncoder();

    // getting PID controller
    intakeAnglePID = intakeAngle.getPIDController();

    intakeAnglePID.setP(Constants.IntakeConstants.intakeAngleKp);
  }

  public void goToPosition(double encoderCounts) {
    // set pid Controller position reference
    intakeAnglePID.setReference(encoderCounts, CANSparkMax.ControlType.kPosition);
  }

  public void setPower(double pow) {
    intakeMotor.set(-pow);
  }

  public double getPosition() {
    return intakeAngleRelativeEncoder.getPosition();
  }

  public void resetEncoder() {
    intakeAngleRelativeEncoder.setPosition(0.0);
  }

  public void setIntakeAnglePower(double power){
    intakeAngle.set(power);
  }

  public void setIntakeMotorPower(double power){
    intakeMotor.set(-power);
  }

  public void runIntake() {
    intakeMotor.set(IntakeConstants.kIntakePower);
  }

  public void stop() {
    intakeMotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Encoder", getPosition());
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

  public Command getStopIntakeCommand() {
    return this.runOnce(() -> {stop();});
  }

  public Command getIntakeDownCommand() {
    return this.startEnd( () -> {
      goToPosition(IntakeConstants.kIntakeAngleDown);
    }, () ->{});
  }

  public Command getRunToPositionCommand(double position) {
    return this.startEnd(() -> {
      goToPosition(position);
    }, () -> {});
  }
}
