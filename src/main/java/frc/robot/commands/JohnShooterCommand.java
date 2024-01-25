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

  private Long storedTime;
  private Long waitTime = (long) 2000;
  private Long stopTime = (long) 3000;
  private boolean isShooting = false;

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
    if (Robot.getDriveControlJoystick().getLeftTriggerAxis() >= .5 && !isShooting){
      isShooting = true;
      storedTime = System.currentTimeMillis();
    }    
  

    if (isShooting) {
        johnShooterSystem.shooterPower(1);
        if (System.currentTimeMillis() > waitTime + storedTime) {
          johnShooterSystem.indexerPower(1);
        }
        if (System.currentTimeMillis() > waitTime + storedTime + stopTime) {
          isShooting = false;
        }
    }
    else {
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
