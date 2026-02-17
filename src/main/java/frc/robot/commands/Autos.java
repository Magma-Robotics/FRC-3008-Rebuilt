/*
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  private final Intake Intake;
  private final Shooter Shooter;
Example static factory for an autonomous command. 
  public static Command exampleAuto(ExampleSubsystem subsystem, Intake intake, Shooter shooter) {
    // Set intake during autonomous as needed; replace the lambda body with the actual intake call when available
    return Commands.sequence(
        Commands.runOnce(() -> {
          // intake.setSpeed(double) is not defined on Intake; implement Intake.setSpeed(double)
          // or replace this with the appropriate existing Intake method.
        }),
        subsystem.exampleMethodCommand(),
        Commands.none());
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
*/