/*
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

 An example command that uses an example subsystem. 
public class Audos extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final ExampleSubsystem m_subsystem;
  private final Intake Intake;
  private final Shooter Shooter;

  
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   
  public Audos(ExampleSubsystem subsystem, Intake intake, Shooter shooter) {
    m_subsystem = subsystem;
    this.Intake = intake;
    this.Shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem, intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
   //Shooter.setFlywheel(0.5);
   //Shooter.setHood(0.5);
   //Shooter.setTurret(0.5);
   //SwerveSubsystem.setSwerveDrive(0.5, 0.5, 0.5);
   //add more commands here
   //use .andThen() to string commands together
   //use .withTimeout() to set a time limit on commands
   //use .until() to end a command when a condition is met
   

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Intake.setIntake(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake.setIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
*/