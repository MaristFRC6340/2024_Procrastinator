package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase{
    
    private CANSparkMax indexerMotor;

    public IndexerSubsystem() {
        indexerMotor = new CANSparkMax(IndexerConstants.kIndexerMotorCanID, MotorType.kBrushed);
        indexerMotor.setSmartCurrentLimit(40);
    }

    public void setIndexerPower(double power) {
        indexerMotor.set(power);
    }

    public void stop() {
        indexerMotor.set(0);
    }
    public Command getReverseIndexerCommand() {
        return this.startEnd(() -> {
            setIndexerPower(.3);
        }, () -> {
            stop();
        });
    }

    public Command getForwardIndexerCommand() {
        return this.startEnd(() -> {
            setIndexerPower(-.9);
        }, () -> {
            stop();
        });
    }

    public Command stopIndexerCommand() {
        return this.startEnd( () -> {
            stop();
        }, () -> {
            stop();
        });
    }
}
