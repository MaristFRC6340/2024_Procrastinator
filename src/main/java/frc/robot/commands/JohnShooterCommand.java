// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.JohnShooter;

public class JohnShooterCommand extends Command {
  /** Creates a new ShooterCommand. */

  private JohnShooter johnShooterSystem;

  public JohnShooterCommand(JohnShooter johnShooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    johnShooterSystem = johnShooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Listers
    // Joystic 
    if (Robot.getDriveControlJoystick().getLeftTriggerAxis() >= .5 && Robot.getDriveControlJoystick().getYButton()){
      johnShooterSystem.indexerPower(1);
      johnShooterSystem.feederPower(1);
      johnShooterSystem.shooterPower(1);

    }
    else if (Robot.getDriveControlJoystick().getYButton()) {  // Y
      johnShooterSystem.shooterPower(1);
      johnShooterSystem.feederPower(1);
    }
    else if(Robot.getDriveControlJoystick().getAButton()){
      johnShooterSystem.feederPower(-.5);
      johnShooterSystem.shooterPower(-.2);
    }
    else if (Robot.getDriveControlJoystick().getRightTriggerAxis() >= .5){
      johnShooterSystem.feederPower(-.5);
      johnShooterSystem.shooterPower(0);
      johnShooterSystem.indexerPower(-.3);
    }
    else{
      johnShooterSystem.shooterPower(0);
      johnShooterSystem.feederPower(0);
      johnShooterSystem.indexerPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
