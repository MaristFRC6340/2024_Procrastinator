
package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.JohnShooter;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.WilliamShooterSubsystem;


//Change this name to the name of the Command you are creating from "ExampleCommand"
public class DriveToSourceCommand extends Command{
    private NetworkTable limTable;
    private NetworkTableEntry tx;
    private NetworkTableEntry ledMode;
    private NetworkTableEntry ty;
    private DriveSubsystem m_DriveSubsystem;
    private WilliamShooterSubsystem m_WilliamShooterSubsystem;
    private IndexerSubsystem m_IndexerSubsystem;
    //Create an instance of whatever subsystems you will use for this command. Example: private ExampleSubsystem m_ExampleSubsystem;
    
    //Change this from "ExampleCommand" to whatever you are naming this command
    //Add a paremeter to this constructor for any subsystems you will need. Example: public ExampleCommand(ExampleSubsystem example) {...}
    public DriveToSourceCommand(DriveSubsystem drive, WilliamShooterSubsystem shooter, IndexerSubsystem indexer) {
        m_DriveSubsystem = drive;
        m_WilliamShooterSubsystem = shooter;
        m_IndexerSubsystem = indexer;
        addRequirements(drive, m_WilliamShooterSubsystem, m_IndexerSubsystem);

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

    if(tx.getDouble(0)!=0)
    {
        xError = tx.getDouble(0);
        m_DriveSubsystem.drive(-.2, xError*LimelightConstants.kPX, 0, false, false);
        m_WilliamShooterSubsystem.setShooter(0);
        m_IndexerSubsystem.setIndexerPower(0);
    }
    else {
        //m_DriveSubsystem.drive(-.2, xError*LimelightConstants.kPX, 0, false, false);
        m_WilliamShooterSubsystem.setShooter(ShooterConstants.kWilliamShooterIntake);
        m_IndexerSubsystem.setIndexerPower(.1);
        m_DriveSubsystem.drive(0, 0, 0, false, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ledMode.setDouble(1);
    m_DriveSubsystem.drive(0, 0, 0, false, false);
    m_WilliamShooterSubsystem.setShooter(0);
    m_IndexerSubsystem.setIndexerPower(0);
  }

  // Returns true when the command should end. In most cases it will always return false, and the Command will be interupted before
  //it is completed.
  @Override
  public boolean isFinished() {
    return false;
  }
}

