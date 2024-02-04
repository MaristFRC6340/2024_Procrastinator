package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JohnShooter;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.WilliamShooterSubsystem;


//Change this name to the name of the Command you are creating from "ExampleCommand"
public class TransferToIndexerCommand extends Command{

    //Create an instance of whatever subsystems you will use for this command. Example: private ExampleSubsystem m_ExampleSubsystem;
    WilliamShooterSubsystem m_WilliamShooterSubsystem;
    IntakeSubsystem m_IntakeSubsystem;
    //Change this from "ExampleCommand" to whatever you are naming this command
    //Add a paremeter to this constructor for any subsystems you will need. Example: public ExampleCommand(ExampleSubsystem example) {...}
    public TransferToIndexerCommand(WilliamShooterSubsystem shooter, IntakeSubsystem intake){
        
        //In this method, initialize the field subsystem to the parameter. Example: m_ExampleSubsystem = example;
        m_WilliamShooterSubsystem = shooter;
        m_IntakeSubsystem = intake;
        //Now, call addRequirements on any fields that are subsystems. Example: addRequirements(m_ExampleSubsystem);
        addRequirements(m_WilliamShooterSubsystem);
        addRequirements(m_IntakeSubsystem);
    }

    //Called once when the command is initialized. 
  @Override
  public void initialize() {
    m_IntakeSubsystem.runIntake();
    m_WilliamShooterSubsystem.setIndexer(-.6);
  }

  // Called every time the scheduler runs while the command is scheduled. There should not be any listeners in this method.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_WilliamShooterSubsystem.setIndexer(0);
    m_IntakeSubsystem.setIntakeMotorPower(0);
  }

  // Returns true when the command should end. In most cases it will always return false, and the Command will be interupted before
  //it is completed.
  @Override
  public boolean isFinished() {
    return false;
  }
}

