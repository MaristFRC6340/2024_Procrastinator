// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class JohnShooter extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;

  private CANSparkMax leftFeeder;
  private CANSparkMax rightFeeder;
  private CANSparkMax bottomIndexer;
  private CANSparkMax indexer;

  public JohnShooter() {
    leftMotor = new CANSparkMax(20, MotorType.kBrushless);  // NEO Motor
    rightMotor = new CANSparkMax(22, MotorType.kBrushless); // NEO Motor
    bottomIndexer = new CANSparkMax(23, MotorType.kBrushed);   // Brushed
    rightFeeder = new CANSparkMax(21, MotorType.kBrushless);  // NEO Motor - Deprecated (For now)
    indexer = new CANSparkMax(18, MotorType.kBrushed); //Brushed
  }

  public void shooterPower(double power) {
    leftMotor.set(power);
    rightMotor.set(-power);
  }

  public void feederPower(double power) {
    //leftFeeder.set(power);
    //rightFeeder.set(-power);
  }

  public void indexerPower(double power){
    indexer.set(power);
    bottomIndexer.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //COMMAND FACTORIES

  public Command getIndexerIntakeCommand() {
    return this.startEnd(() -> {
      this.indexerPower(.5);
    }, () -> {
      this.indexerPower(0);
    });
  }
}
