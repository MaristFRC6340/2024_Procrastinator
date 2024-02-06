package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
public class WilliamShooterSubsystem extends SubsystemBase{
    
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private CANSparkMax indexMotor;
    public WilliamShooterSubsystem() {
        leftMotor = new CANSparkMax(ShooterConstants.kWilliamShooterLeft, MotorType.kBrushless);
        rightMotor = new CANSparkMax(ShooterConstants.kWilliamShooterRight, MotorType.kBrushless);

        indexMotor = new CANSparkMax(ShooterConstants.kWIlliamShooterIndexCanID, MotorType.kBrushed);
    }

    public void setShooter(double power) {
        leftMotor.set(-power);
        rightMotor.set(-power);
    }

    public void spinUp() {
        leftMotor.set(ShooterConstants.kWilliamShooterPower);
        rightMotor.set(ShooterConstants.kWilliamShooterPower);
    }

    public void setIndexer(double power) {
        indexMotor.set(power);
    }
    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
        indexMotor.set(0);
    }

    public Command getStartShooterCommand() {
        return this.startEnd( () -> {
            spinUp();
        }, () -> {
            stop();
        });
    }

    public Command getSpinUpShooterCommand() {
        return this.startEnd(() -> {
            spinUp();
        }, () -> {

        });
    }
    public Command getReverseShooterCommand(){
        return this.startEnd(()->setShooter(-ShooterConstants.kWilliamShooterPower), ()->{});
    }

    public Command getLaunchNoteCommand() {
        return this.startEnd(() -> {
            spinUp();
            setIndexer(1);
        }, () ->{
            stop();
        });
    }

    public Command getReverseIndexerCommand() {
        return this.startEnd(() -> {
            setIndexer(.3);
        }, () -> {
            setIndexer(0);
        });
    }

    public Command getForwardIndexerCommand() {
        return this.startEnd(() -> {
            setIndexer(-.3);
        }, () -> {
            setIndexer(0);
        });
    }
}
