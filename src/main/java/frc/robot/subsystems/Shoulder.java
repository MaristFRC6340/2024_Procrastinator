package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shoulder extends SubsystemBase{
    //private CANSparkMax shoulder;

    public Shoulder(){
        //shoulder = new CANSparkMax(Constants.ShooterConstants.kShootAngleCanId, MotorType.kBrushless);
    }

    public void shoulderPower(double power){
        //shoulder.set(power);
    }

    

}
