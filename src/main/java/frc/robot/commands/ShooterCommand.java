// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
  /** Creates a new ShooterCommand. */

  private ShooterSubsystem shooterSystem;

  public ShooterCommand(ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooterSystem = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Listers
    // Joystic 
    if (Robot.getShooterLogi().getYButton()) {  // Y
      shooterSystem.setLowerMotor(0.9);
      shooterSystem.setTopMotor(0.9);
    }
    if(Robot.getShooterLogi().getXButton()) { // X
      shooterSystem.setTopMotor(0.9);
    }
    if(Robot.getShooterLogi().getBButton()) { // B
      shooterSystem.setLowerMotor(0.2);
      shooterSystem.setTopMotor(0.2);
    }

    else if (Robot.getShooterLogi().getAButton()){  // A
      shooterSystem.setLowerMotor(-0.5);
      shooterSystem.setTopMotor(-0.5);
    } 
    else{
      shooterSystem.setLowerMotor(0.0);
      shooterSystem.setTopMotor(0.0);
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
