// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.Autos;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
//import frc.robot.subsystems.Intake;
//import frc.robot.subsystems.Shooter;
import swervelib.SwerveInputStream;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverXbox =
      new CommandXboxController(0);

  public Command resetOdometry() {
    return Commands.run(() -> drivebase.resetOdometry(new Pose2d(new Translation2d(), new Rotation2d())));
  }

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * slowMultiplier,
                                                                () -> driverXbox.getLeftX() * slowMultiplier)
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
    // Configure the trigger bindings
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
      .onTrue(Commands.run(() -> drivebase.zeroGyro()));

   // driverXbox
      //.leftBumper()
      //.onTrue(Commands.run(() -> drivebase.setHeadingOffset(Rotation2d.fromDegrees(180))))
      //.onFalse(Commands.run(() -> drivebase.setHeadingOffset(Rotation2d.fromDegrees(0))));
    /*
    //odometry/*
    driverXbox
      .a()
      .onTrue(Commands.run(() -> Intake.setFeeder(0.2), Intake))
      .onFalse(Commands.run(() -> Intake.stopFeeder(),Intake));
      
      //.onTrue(resetOdometry());
      */
    //intake
// In RobotContainer.java

/*
// Button B: Move to 90 degrees and stay there
driverXbox.b()
    .onTrue(Commands.runOnce(() -> m_intake.goToDegree(90), m_intake));

// Button A: Move to 180 degrees and stay there
driverXbox.a()
    .onTrue(Commands.runOnce(() -> m_intake.goToDegree(180), m_intake));

// Button X: Stop motor power
driverXbox.x()
    .onTrue(Commands.runOnce(() -> m_intake.stopIntake(), m_intake));

// Button Y: Return to zero
driverXbox.y()
    .onTrue(Commands.runOnce(() -> m_intake.goToDegree(0), m_intake));

driverXbox.povUp()
    .onTrue(Commands.run(() -> m_intake.setIntake(0.2), m_intake))
    .onFalse(Commands.run(() -> m_intake.setIntake(0), m_intake));

    /* 
    driverXbox
      .x()
      .onTrue(Commands.run(() -> Intake.setIndexer(0.2), Intake))
      .onFalse(Commands.run(() -> Intake.stopIndexer(),Intake));
    driverXbox
      .y()
      .onTrue(Commands.run(() -> Intake.setIntakePivot(0.2), Intake))
      .onFalse(Commands.run(() -> Intake.stopIntakePivot(),Intake));
    //shootah
    driverXbox
      .povUp()
      .onTrue(Commands.run(() -> Shooter.setTurret(0.2), Shooter))
      .onFalse(Commands.run(() -> Shooter.stopTurret(),Shooter));    
    driverXbox
      .povDown()
      .onTrue(Commands.run(() -> Shooter.setFlywheel(0.2), Shooter))
      .onFalse(Commands.run(() -> Shooter.stopFlyWheel(),Shooter));  
    driverXbox
      .povLeft()
      .onTrue(Commands.run(() -> Shooter.setCounterRoller(0.2), Shooter))
      .onFalse(Commands.run(() -> Shooter.stopCounterRoller(),Shooter));  
    driverXbox
      .povRight()
      .onTrue(Commands.run(() -> Shooter.setHood(0.2), Shooter))
      .onFalse(Commands.run(() -> Shooter.stopHood(),Shooter)); 

        //Intake.setIntake(0.1); // 

        //Hunter note
        //runonce: one rotation at set power, run: until controller is off
        //onTrue: doesn't
      
      //driver.a().onTrue(new ExampleCommand()); //#toggle
    //driverXbox.x().whileTrue(Commands.run(() -> intake.startIntake()));
      //intake.startIntake(1);  
    */

      }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //Intake.setIntake(0.2);
    
    //return Autos.exampleAuto(m_exampleSubsystem, Intake);
  //}
}
