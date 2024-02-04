// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;



public class JohnShooter extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  //private CANSparkMax topMotor;
  //private CANSparkMax bottomMotor;
  //private CANSparkMax bottomIndexer;
  //private CANSparkMax indexer;

  public JohnShooter() {
    // topMotor = new CANSparkMax(ShooterConstants.kTopMotorCanID, MotorType.kBrushless);
    // bottomMotor = new CANSparkMax(ShooterConstants.kBottomMotorCanID, MotorType.kBrushless);
  }

  public void shooterPower(double power) {
    //topMotor.set(power);
  }

  public void feederPower(double power) {
    //bottomMotor.set(power);
  }

  public void indexerPower(double power){
    //indexer.set(power);
    //bottomIndexer.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //COMMAND FACTORIES

  public Command getIndexerIntakeCommand() {
    return this.startEnd(() -> {
      this.shooterPower(-.5);
      this.feederPower(-0.5);
    }, () -> {
      this.shooterPower(0);
      this.feederPower(0);
    });
  }
}
