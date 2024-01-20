// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  //private CANSparkMax topMotor;
  //private CANSparkMax lowerMotor;

  public ShooterSubsystem() {
    //topMotor = new CANSparkMax(6, MotorType.kBrushed);
    //lowerMotor = new CANSparkMax(5, MotorType.kBrushed);
  }

  public void intake(double power) {
     //topMotor.set(power);
     //lowerMotor.set(power);
  }

  public void setTopMotor(double power) {
    //lowerMotor.set(-power);
  }
  public void setLowerMotor(double power) {
    //topMotor.set(-power);
  }

  public void shoot(double power) {
    //topMotor.set(-power);
    //lowerMotor.set(-power);
  }
  public void feed(double power) {
    //lowerMotor.set(-power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
