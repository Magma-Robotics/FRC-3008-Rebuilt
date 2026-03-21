// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.Autos;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Modules;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Modules;
//import frc.robot.subsystems.Shooter;
import swervelib.SwerveInputStream;

import java.io.File;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Modules;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.*;

import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Modules;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private double slowMultiplier = 1;
  // The robot's subsystems and commands are defined here...
  //Shooter Shooter = new Shooter();
  //private final Intake m_intake = new Intake();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/main"));
  private final Modules modules = new Modules();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverXbox =
      new CommandXboxController(0);
      
  private final CommandXboxController driverXbox2 =
      new CommandXboxController(1);

  public Command resetOdometry() {
    return Commands.run(() -> drivebase.resetOdometry(new Pose2d(new Translation2d(), new Rotation2d())));
  }

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> -driverXbox.getLeftY() * slowMultiplier,
                                                                () -> -driverXbox.getLeftX() * slowMultiplier)
                                                            .withControllerRotationAxis(() -> -driverXbox.getRightX())
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(1)
                                                            .allianceRelativeControl(true);    
  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);


  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);//, () -> driverXbox.rightBumper().getAsBoolean());

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -driverXbox.getLeftY(),
                                                                   () -> -driverXbox.getLeftX())
                                                               .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

  Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);

  Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */


  public RobotContainer() {
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

    driverXbox
      .rightBumper()
      .onTrue(Commands.run(() -> slowMultiplier = 0.5))
      .onFalse(Commands.run(() -> slowMultiplier = 1));

    driverXbox
      .a()
      .onTrue(Commands.runOnce(() -> drivebase.zeroGyro()));

    driverXbox
      .b()
      .onTrue(Commands.runOnce(() -> drivebase.autoAimMode()));

    // driverXbox //liftDown
    //   .povDown()
    //   .onTrue(Commands.run(() -> modules.setLift(-1), modules))
    //   .onFalse(Commands.run(() -> modules.stopLift(), modules));

    // driverXbox //liftUp
    //   .povUp()
    //   .onTrue(Commands.run(() -> modules.setLift(1), modules))
    //   .onFalse(Commands.run(() -> modules.stopLift(), modules));


    driverXbox2 //intakeIN
      .leftBumper()
      .onTrue(Commands.run(() -> modules.setIntake(Math.abs(0.4)), modules))
      .onFalse(Commands.run(() -> modules.stopIntake(), modules));
      
    driverXbox2 //intakeOUT
      .rightBumper()
      .onTrue(Commands.run(() -> modules.setIntake(-Math.abs(0.4)), modules)) //this better work -0.2
      .onFalse(Commands.run(() -> modules.stopIntake(), modules));

    driverXbox2//indexer IN
      .rightTrigger()
      .onTrue(Commands.run(() -> modules.setIndexer(0.75), modules))
      .onFalse(Commands.run(() -> modules.stopIndexing(), modules));

    driverXbox2//indexer OUT
      .leftTrigger()
      .onTrue(Commands.run(() -> modules.setIndexer(-0.75), modules))
      .onFalse(Commands.run(() -> modules.stopIndexing(), modules));

    driverXbox2 //slowspeed fire
      .povDown()
      .onTrue(Commands.run(() -> modules.setflyWheel33(), modules))
      .onFalse(Commands.run(() -> modules.stopflyWheel(), modules));
    
    driverXbox2 //fastspeed fire
      .povUp()
      .onTrue(Commands.run(() -> modules.setflyWheel22(), modules))
      .onFalse(Commands.run(() -> modules.stopflyWheel(), modules));
    
    driverXbox2 //feeder forward
      .b()
      .onTrue(Commands.run(() -> modules.setFeeder(Math.abs(1)), modules))
      .onFalse(Commands.run(() -> modules.stopFeeding(), modules));

    driverXbox2 //feeder reverse
      .a()  
      .onTrue(Commands.run(() -> modules.setFeederBack(-Math.abs(1)), modules))
      .onFalse(Commands.run(() -> modules.stopFeeding(), modules));

    }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
  
  public Command getAutonomousCommand() {
      Command modulesCmd = Autos.exampleAuto(m_exampleSubsystem, modules);
      return Commands.parallel(modulesCmd);
  }

   public Command getPathPlannerAutonomous() {
    // Use the SwerveSubsystem's PathPlanner integration (AutoBuilder/PathPlannerAuto)
    // The subsystem already exposes a helper that builds a PathPlannerAuto command
    // with the given path name (configured via the PathPlanner GUI).
    return drivebase.getAutonomousCommand("Draft-Auto-Blue-1");
  }

  public void initForTeleop(){
    drivebase.setAutoAimMode(false);
  }
}
