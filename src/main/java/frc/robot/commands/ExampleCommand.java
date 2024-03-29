package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.JohnShooter;
import frc.robot.subsystems.Shoulder;


//Change this name to the name of the Command you are creating from "ExampleCommand"
public class ExampleCommand extends Command{

    //Create an instance of whatever subsystems you will use for this command. Example: private ExampleSubsystem m_ExampleSubsystem;
    
    //Change this from "ExampleCommand" to whatever you are naming this command
    //Add a paremeter to this constructor for any subsystems you will need. Example: public ExampleCommand(ExampleSubsystem example) {...}
    public ExampleCommand(JohnShooter shooter){
        
        //In this method, initialize the field subsystem to the parameter. Example: m_ExampleSubsystem = example;

        //Now, call addRequirements on any fields that are subsystems. Example: addRequirements(m_ExampleSubsystem);

    }

    //Called once when the command is initialized. 
  @Override
  public void initialize() {
      
  }

  // Called every time the scheduler runs while the command is scheduled. There should not be any listeners in this method.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end. In most cases it will always return false, and the Command will be interupted before
  //it is completed.
  @Override
  public boolean isFinished() {
    return false;
  }
}

