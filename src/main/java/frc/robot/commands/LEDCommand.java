package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDPattern;
import frc.robot.subsystems.LEDSubsystem.LEDState;;

public class LEDCommand extends Command{
    
    LEDPattern pattern;
    LEDSubsystem leds;
    public LEDCommand(LEDPattern pattern, LEDSubsystem leds){
        this.pattern = pattern;
        this.leds = leds;
    }

    @Override
    public void execute(){
        leds.setColors(pattern.getCurrentLEDState());
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
