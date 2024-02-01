// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

  private IntakeSubsystem m_IntakeSubsystem;
  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intakeSubsystem) {
    m_IntakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get Value and Set Intake Power
    double intakePower = Robot.getShooterControlJoystick().getLeftTriggerAxis() - Robot.getShooterControlJoystick().getRightTriggerAxis();

    // Get Value and Set Angle Power
    double anglePower = Robot.getShooterControlJoystick().getLeftY();

    // Speed Controller
    if (anglePower > 0.2) {
      anglePower = 0.2;
    }
    if (anglePower < -0.2) {
      anglePower = -0.2;
    }

    //Intake Power Controller
    if (intakePower > 0.5) {
      intakePower = 0.5;
    }
    if (intakePower < -0.5) {
      intakePower = -0.5;
    }

    m_IntakeSubsystem.setIntakeAnglePower(anglePower);
    m_IntakeSubsystem.setIntakeMotorPower(intakePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.setIntakeAnglePower(0);
    m_IntakeSubsystem.setIntakeMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
