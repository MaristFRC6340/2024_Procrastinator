// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.JohnShooter;
import frc.robot.subsystems.ShooterSubsystem;

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
    if (Robot.getShooterLogi().getYButton()) {  // Y
      johnShooterSystem.shooterPower(0.9);
      johnShooterSystem.feederPower(0.8);
    }
    else{
      johnShooterSystem.shooterPower(0.0);
      johnShooterSystem.feederPower(0.0);
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
