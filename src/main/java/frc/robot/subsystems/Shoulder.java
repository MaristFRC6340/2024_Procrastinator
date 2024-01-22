package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoulder extends SubsystemBase{
    private CANSparkMax shoulder;

    public Shoulder(){
        shoulder = new CANSparkMax(24, MotorType.kBrushless);
    }

    public void shoulderPower(double power){
        shoulder.set(power);
    }

    

}
