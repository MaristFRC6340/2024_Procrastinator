// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class JohnShooter extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;

  private CANSparkMax leftFeeder;
  private CANSparkMax rightFeeder;

  public JohnShooter() {
    leftMotor = new CANSparkMax(20, MotorType.kBrushless);  // NEO Motor
    rightMotor = new CANSparkMax(21, MotorType.kBrushless); // NEO Motor
    leftFeeder = new CANSparkMax(22, MotorType.kBrushed);   // BAG Motor
    rightFeeder = new CANSparkMax(23, MotorType.kBrushed);  // BAG Motor
  }

  public void shooterPower(double power) {
    leftMotor.set(power);
    rightMotor.set(-power);
  }

  public void feederPower(double power) {
    leftFeeder.set(power);
    rightFeeder.set(-power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
