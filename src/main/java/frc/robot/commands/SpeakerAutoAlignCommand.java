package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shoulder;


public class SpeakerAutoAlignCommand extends Command{
    

    private NetworkTable limTable;
    private NetworkTableEntry tx;
    private NetworkTableEntry ledMode;
    double leftX = 0;
  double leftY = 0;
  double turnPower = 0;

  DriveSubsystem drive;
  Shoulder Shoulder;
  private double kP = .0375;//was .025

    public SpeakerAutoAlignCommand(DriveSubsystem drive, Shoulder shoulder){

    }   

    @Override
    public void initialize() {
        // Limelight Initialization
        limTable = NetworkTableInstance.getDefault().getTable("limelight");
        tx = limTable.getEntry("tx");
        ledMode = limTable.getEntry("ledMode");
        ledMode.setDouble(3); // On

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Updated Drive Command

        double error = tx.getDouble(0);
        turnPower = kP * error;
        leftX = turnPower;
        
        System.out.println(tx.getDouble(0) + ", " + turnPower);
        
        drive.drive(
                MathUtil.applyDeadband(-leftY, 0.06),
                MathUtil.applyDeadband(-leftX, 0.06),
                MathUtil.applyDeadband(-turnPower, 0.06),
                false, true);
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
