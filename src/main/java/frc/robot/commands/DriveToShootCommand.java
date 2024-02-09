
package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.JohnShooter;
import frc.robot.subsystems.Shoulder;


//Change this name to the name of the Command you are creating from "ExampleCommand"
public class DriveToShootCommand extends Command{
    private NetworkTable limTable;
    private NetworkTableEntry tx;
    private NetworkTableEntry ledMode;
    private NetworkTableEntry ty;
    private DriveSubsystem m_DriveSubsystem;
    //Create an instance of whatever subsystems you will use for this command. Example: private ExampleSubsystem m_ExampleSubsystem;
    
    //Change this from "ExampleCommand" to whatever you are naming this command
    //Add a paremeter to this constructor for any subsystems you will need. Example: public ExampleCommand(ExampleSubsystem example) {...}
    public DriveToShootCommand(DriveSubsystem drive) {
        m_DriveSubsystem = drive;
        addRequirements(drive);

        //Limelight init
        limTable = NetworkTableInstance.getDefault().getTable("limelight");
        ledMode = limTable.getEntry("ledMode");
        tx = limTable.getEntry("tx");
        ty = limTable.getEntry("ty");
    }

    //Called once when the command is initialized. 
  @Override
  public void initialize() {
    ledMode.setDouble(3); //3 is on, 1 is off
    tx = limTable.getEntry("tx");
    ty = limTable.getEntry("ty");
    
  }

  double xError;
  double yError;
  // Called every time the scheduler runs while the command is scheduled. There should not be any listeners in this method.
  @Override
  public void execute() {
    if(tx.exists()&&ty.exists()) {
        xError = (tx.getDouble(0)-LimelightConstants.speakerAimtx);
        yError = LimelightConstants.speakerAimty-ty.getDouble(0);
        m_DriveSubsystem.drive(
            -1*yError*LimelightConstants.kPY,
            xError*LimelightConstants.kPX,
            0,
            false,
            false);
    }
    else {
        m_DriveSubsystem.drive(
            -1*yError*LimelightConstants.kPY,
            xError*LimelightConstants.kPX,
            0,
            false,
            false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ledMode.setDouble(1);
    m_DriveSubsystem.drive(0, 0, 0, false, false);
  }

  // Returns true when the command should end. In most cases it will always return false, and the Command will be interupted before
  //it is completed.
  @Override
  public boolean isFinished() {
    if((Math.abs(xError)<LimelightConstants.tolerance && Math.abs(yError)<LimelightConstants.tolerance)) {
        return true;
    }
    return false;
  }
}

