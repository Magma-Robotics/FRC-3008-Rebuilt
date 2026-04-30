// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.Autos;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Hood;
import swervelib.SwerveInputStream;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.commands.Autos;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {

  private double slowMultiplier = 0.9;
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem      drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/main"));
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Hood hood = new Hood();

  private final CommandXboxController driverXbox =
      new CommandXboxController(0);
      
  private final CommandXboxController driverXbox2 =
      new CommandXboxController(1);

  public Command resetOdometry() {
    return Commands.run(() -> drivebase.resetOdometry(new Pose2d(new Translation2d(), new Rotation2d())));
  }

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> -driverXbox.getLeftY() * slowMultiplier,
                                                                () -> -driverXbox.getLeftX() * slowMultiplier)
                                                            .withControllerRotationAxis(() -> -driverXbox.getRightX())
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(1)
                                                            .allianceRelativeControl(true);    

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -driverXbox.getLeftY(),
                                                                   () -> -driverXbox.getLeftX())
                                                               .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);

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

  public RobotContainer() {
    configureAutoBindings();
    configureBindings();
  }

  private void configureAutoBindings() {
    NamedCommands.registerCommand("Shoot",  Commands.run(() -> shooter.autonomousShoot1()));
    NamedCommands.registerCommand("No Shoot",  Commands.run(() -> shooter.autoNoFly()));
    NamedCommands.registerCommand("Open the Hopper", Commands.run(() -> intake.autoPivot(0.4)));
    NamedCommands.registerCommand("Hopper Shrink", Commands.run(() -> intake.autoPivotRetract(0.4)));
    NamedCommands.registerCommand("Intake", Commands.run(() -> intake.autoIntake(-0.4)));
    NamedCommands.registerCommand("No Intake", Commands.run(() -> intake.autoIntake(0)));
    NamedCommands.registerCommand("No Intake", Commands.run(() -> intake.autoIntake(0)));
  }

  private void configureBindings() {
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

    driverXbox
      .a()
      .onTrue(Commands.runOnce(() -> drivebase.zeroGyro()));

    driverXbox
      .b()
      .onTrue(Commands.runOnce(() -> drivebase.autoAimMode()));

    /* driver
    Xbox2
      .leftBumper()
      .onTrue(Commands.run(() -> intake.setIntake(Math.abs(0.4)), intake))
      .onFalse(Commands.run(() -> intake.stopIntake(), intake));
      
    driverXbox2
      .rightBumper()
      .onTrue(Commands.run(() -> intake.setIntake(-Math.abs(0.4)), intake)) //this better work -0.2
      .onFalse(Commands.run(() -> intake.stopIntake(), intake)); */

    driverXbox2
      .leftBumper()
      .onTrue(Commands.run(() -> intake.smartIntake(0.4), intake))
      .onFalse(Commands.run(() -> intake.stopIntake(), intake));
    
    driverXbox2
      .leftTrigger()
      .onTrue(Commands.run(() -> intake.smartIntake(-0.7), intake)) //this better work -0.2
      .onFalse(Commands.run(() -> intake.stopIntake(), intake));

     driverXbox2
      .rightTrigger()
      .onTrue(Commands.run(() -> intake.setIndexer(0.75), intake))
      .onFalse(Commands.run(() -> intake.stopIndexing(), intake));

    /* driverXbox2
      .rightTrigger()
      .onTrue(Commands.run(() -> intake.setIndexer(-0.75), intake))
      .onFalse(Commands.run(() -> intake.stopIndexing(), intake)); */

    driverXbox2
      .y()
      .onTrue(Commands.run(() -> shooter.autoPowerShoot(), shooter))
      .onTrue(Commands.runOnce(() -> hood.hoodAutoEnable(), hood))
      .onFalse(Commands.run(() -> shooter.stopflyWheel(), shooter))
      .onFalse(Commands.runOnce(() -> hood.hoodAutoRetract(), hood));

    driverXbox2
      .a()
      .onTrue(Commands.run(() -> shooter.autoPowerShoot(), shooter))
      .onFalse(Commands.run(() -> shooter.stopflyWheel(), shooter));

    /* driverXbox2
      .povDown()
      .onTrue(Commands.run(() -> shooter.setflyWheelD(), shooter))
      .onFalse(Commands.run(() -> shooter.stopflyWheel(), shooter));
    
    driverXbox2
      .povRight()
      .onTrue(Commands.run(() -> shooter.setflyWheelR(), shooter))
      .onFalse(Commands.run(() -> shooter.stopflyWheel(), shooter));
    
    driverXbox2
      .povUp()
      .onTrue(Commands.run(() -> shooter.setflyWheelU(), shooter))
      .onFalse(Commands.run(() -> shooter.stopflyWheel(), shooter)); */
    
    driverXbox2
      .b()
      .onTrue(Commands.run(() -> shooter.setFeeder(Math.abs(0.5)), shooter))
      .onFalse(Commands.run(() -> shooter.stopFeeding(), shooter));

    /* driverXbox2
      .a()  
      .onTrue(Commands.run(() -> shooter.setFeederBack(-Math.abs(0.5)), shooter))
      .onFalse(Commands.run(() -> shooter.stopFeeding(), shooter)); */

    driverXbox2
      .rightBumper()  
      .onTrue(Commands.run(() -> intake.agitatePivot(), intake))
      .onFalse(Commands.run(() -> intake.setPivot(0), intake));

    driverXbox2
      .povUp().and(() -> (intake.getPivotPosition() < -0.45))
      .onTrue(Commands.run(() -> intake.setPivot(1), intake))
      .onFalse(Commands.run(() -> intake.stopPivot(), intake));

    driverXbox2
      .povDown().and(() -> (intake.getPivotPosition() > -74.5))
      .onTrue(Commands.run(() -> intake.setPivot(-1), intake))
      .onFalse(Commands.run(() -> intake.stopPivot(), intake));

    driverXbox2
      .leftStick().and(() -> (hood.getHoodPos() < 2.5))
      .onTrue(Commands.runOnce(() -> hood.hoodAutoDisable(), hood))
      .onTrue(Commands.run(() -> hood.setHoodSpeed(1), hood))
      .onFalse(Commands.runOnce(() -> hood.stopHood(), hood));

    driverXbox2
      .rightStick().and(() -> (hood.getHoodPos() > 0))
      .onTrue(Commands.runOnce(() -> hood.hoodAutoDisable(), hood))
      .onTrue(Commands.run(() -> hood.setHoodSpeed(-0.15), hood))
      .onFalse(Commands.runOnce(() -> hood.stopHood(), hood));

      //   driverXbox2
      //   .leftStick().and(() -> (hood.getHoodPos() < 2.5))
      //   .onTrue(Commands.run(() -> hood.hoodAutoDisable(), hood))
      //   .onTrue(Commands.run(() -> hood.setHoodSpeed(1), hood))
      //   .onFalse(Commands.run(() -> hood.stopHood(), hood));

      // driverXbox2
      //   .rightStick().and(() -> (hood.getHoodPos() > 0))
      //   .onTrue(Commands.run(() -> hood.hoodAutoDisable(), hood))
      //   .onTrue(Commands.run(() -> hood.setHoodSpeed(-0.15), hood))
      //   .onFalse(Commands.run(() -> hood.stopHood(), hood));
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
  
  public Command getAutonomousCommand() {
    return null;
  }

   public Command getPathPlannerAutonomous() {
    return drivebase.getAutonomousCommand("AB Left Trench");
  }

  public void initForTeleop(){
    drivebase.setAutoAimMode(false);
  }
}
