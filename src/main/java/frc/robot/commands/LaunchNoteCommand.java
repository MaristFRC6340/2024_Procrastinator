package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.JohnShooter;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.WilliamShooterSubsystem;


//Change this name to the name of the Command you are creating from "ExampleCommand"
public class LaunchNoteCommand extends Command{

    //Create an instance of whatever subsystems you will use for this command. Example: private ExampleSubsystem m_ExampleSubsystem;
    private WilliamShooterSubsystem m_WilliamShooterSubsystem;
    private IndexerSubsystem m_IndexerSubsystem;

    //Change this from "ExampleCommand" to whatever you are naming this command
    //Add a paremeter to this constructor for any subsystems you will need. Example: public ExampleCommand(ExampleSubsystem example) {...}
    public LaunchNoteCommand(WilliamShooterSubsystem shooter, IndexerSubsystem indexer){
      m_IndexerSubsystem = indexer;
      m_WilliamShooterSubsystem = shooter;

      addRequirements(indexer);
      addRequirements(shooter);

    }

    //Called once when the command is initialized. 
  @Override
  public void initialize() {
    m_WilliamShooterSubsystem.spinUp();
    m_IndexerSubsystem.setIndexerPower(IndexerConstants.kIndexerLaunchPower);
    // m_JohnShooterSubsystem.indexerPower(1); Removed due to change in shooter design
  }

  // Called every time the scheduler runs while the command is scheduled. There should not be any listeners in this method.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_WilliamShooterSubsystem.stop();
    m_IndexerSubsystem.stop();
  }

  // Returns true when the command should end. In most cases it will always return false, and the Command will be interupted before
  //it is completed.
  @Override
  public boolean isFinished() {
    return false;
  }
}