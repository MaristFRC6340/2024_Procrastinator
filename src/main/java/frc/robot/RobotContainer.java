// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JohnShooterCommand;
import frc.robot.commands.LaunchNoteCommand;
import frc.robot.commands.ShoulderCommand;
import frc.robot.commands.SpinUpShooterCommand;
import frc.robot.commands.TransferToIndexerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JohnShooter;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.WilliamShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  //JohnShooter shooter = new JohnShooter();
  Shoulder shoulder = new Shoulder();
  IntakeSubsystem intake = new IntakeSubsystem();
  WilliamShooterSubsystem m_williamShooter = new WilliamShooterSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_shooterController = new XboxController(OIConstants.kShooterControllerPort);

  //Create Triggers
  Trigger y = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  Trigger a = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  Trigger b = new JoystickButton(m_driverController, XboxController.Button.kB.value);
  Trigger x = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  Trigger rBumper = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
  Trigger lBumper = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);


  //Trigger rightStickVertical = new Trigger(() -> {return Math.abs(m_shooterController.getRightY())>.1;});
  Trigger y2 = new JoystickButton(m_shooterController, XboxController.Button.kY.value);
    Trigger b2 = new JoystickButton(m_shooterController, XboxController.Button.kB.value);

  Trigger a2 = new JoystickButton(m_shooterController, XboxController.Button.kA.value);
  Trigger x2 = new JoystickButton(m_shooterController, XboxController.Button.kX.value);
  Trigger lBumper2 = new JoystickButton(m_shooterController, XboxController.Button.kLeftBumper.value);
  Trigger rBumper2 = new JoystickButton(m_shooterController, XboxController.Button.kRightBumper.value);
  private final SendableChooser<Command> autoChooser;

  private final DriveCommand m_DriveCommand = new DriveCommand(m_robotDrive);
  private final IntakeCommand m_IntakeCommand = new IntakeCommand(intake);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();
    // Configure default commands
    registerNamedCommands();

    m_robotDrive.setDefaultCommand(m_DriveCommand);
    intake.setDefaultCommand(m_IntakeCommand);
    //Create sendable chooser and give it to the smartdashboard
    autoChooser = AutoBuilder.buildAutoChooser("Example Auto2");
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

     // Y Shoots the Notes       
    //  y.whileTrue(new SpinUpShooterCommand(shooter).withTimeout(2)
    //               .andThen(new LaunchNoteCommand(shooter)).handleInterrupt(() -> {shooter.feederPower(0); 
    //               shooter.shooterPower(0); shooter.indexerPower(0);} ));
    
    // A Intakes the Notes with Shooter
     //a.whileTrue(shooter.getIndexerIntakeCommand());

     //rightStickVertical.whileTrue(intake.getRunIntakeCommand(m_shooterController.getRightY()));

     y2.whileTrue(m_williamShooter.getSpinUpShooterCommand().withTimeout(2).handleInterrupt(() ->{m_williamShooter.stop();}).andThen(m_williamShooter.getLaunchNoteCommand()));
      b2.whileTrue(m_williamShooter.getReverseShooterCommand().withTimeout(2).handleInterrupt(()->{m_williamShooter.stop();}));

     a2.whileTrue(new TransferToIndexerCommand(m_williamShooter, intake));
     lBumper2.whileTrue(m_williamShooter.getReverseIndexerCommand());
     rBumper2.whileTrue(m_williamShooter.getForwardIndexerCommand());
  }

  /**
   * Register all named commands for use in path planner in this method
   */
  private void registerNamedCommands() {
    NamedCommands.registerCommand("runIntake", new TransferToIndexerCommand(m_williamShooter, intake));
    NamedCommands.registerCommand("stopIntake", intake.getStopIntakeCommand());
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    

    // // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config);

    // var thetaController = new ProfiledPIDController(
    //     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     exampleTrajectory,
    //     m_robotDrive::getPose, // Functional interface to feed supplier
    //     DriveConstants.kDriveKinematics,

    //     // Position controllers
    //     new PIDController(AutoConstants.kPXController, 0, 0),
    //     new PIDController(AutoConstants.kPYController, 0, 0),
    //     thetaController,
    //     m_robotDrive::setModuleStates,
    //     m_robotDrive);

    // // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));

    return autoChooser.getSelected();

    //return this.getOnTheFlyPath();
  }

  public Command getDriveCommand() {
    return m_DriveCommand;
  }
  public Command getShoulderCommand(){
    return new ShoulderCommand(shoulder);
  }


  public Command getIntakeCommand() {
    return m_IntakeCommand;
  }

  public Command getOnTheFlyPath() {
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
      new Pose2d(1, 1, Rotation2d.fromDegrees(0)),
      new Pose2d(2, 1, Rotation2d.fromDegrees(0))

    );

    PathPlannerPath path = new PathPlannerPath(
      bezierPoints,
      new PathConstraints(3, 3, 6.28, 12.56),
      new GoalEndState(0, Rotation2d.fromDegrees(-90))
    );

    path.preventFlipping=true;

    return AutoBuilder.followPath(path);
  }


  


}