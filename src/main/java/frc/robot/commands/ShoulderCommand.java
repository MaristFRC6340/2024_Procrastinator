package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.JohnShooter;
import frc.robot.subsystems.Shoulder;



public class ShoulderCommand extends Command{
    private Shoulder shoulder;
    public ShoulderCommand(Shoulder shoulder){
        this.shoulder = shoulder;
    }
    @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Listers
    // Joystic 
    if (Robot.getDriveControlJoystick().getXButton()) {  // Dpad UP
        shoulder.shoulderPower(-.2);
    }
    else if(Robot.getDriveControlJoystick().getAButton()){
        shoulder.shoulderPower(.2);
    }
    else{
        shoulder.shoulderPower(0);
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

