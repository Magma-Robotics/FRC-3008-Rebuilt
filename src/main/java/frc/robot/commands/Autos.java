package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  private final Intake intake;
  public static Command exampleAuto(ExampleSubsystem subsystem, Intake intake) {
    // Build an actual command sequence (do not create commands inside a runOnce lambda)
    return Commands.sequence(
        // Turn turret to the right for 0.75s
        Commands.runOnce(() -> intake.setTurretL(0.05), intake),
        new WaitCommand(0.25),

        // Stop turret
        Commands.runOnce(() -> intake.setTurretL(0.0), intake),
        
        // Hood Up      
        Commands.runOnce(() -> intake.hoodUp(0.01), intake),
        new WaitCommand(0.25),

        // Stops Hood from Overshooting
        Commands.runOnce(() -> intake.hoodUp(0.0), intake),

        
        // Spin up flywheel and wait
        Commands.runOnce(() -> intake.setflyWheel33(), intake),
        new WaitCommand(2.5),

        // Start indexing/feeding
        Commands.runOnce(() -> {
          intake.setIndexer(0.5);
          intake.setFeeder(0.8);
        }, intake),

        // Wait while shooting
        new WaitCommand(13),

        Commands.runOnce(() -> intake.setFlywheelZero(0)),

        // Return turret to origin (runs until position reached), then stop turret
        Commands.run(() -> intake.setTurretToOrigin(0.1), intake)
            .until(() -> Math.abs(intake.getTurretPosition()) <= 0.01)
            .andThen(Commands.runOnce(intake::stopTurret, intake)),

        // Hood Down
        Commands.runOnce(() -> intake.hoodDown(0.01), intake),
        new WaitCommand(0.25),
        
        // Stops the hood to dig down too much to avoid motor burns
        Commands.runOnce(() -> intake.hoodDown(0.0), intake),

        // Ensure everything is off
        Commands.runOnce(() -> {
          intake.setIndexer(0);
          intake.setFeeder(0);
          intake.setFlywheelZero(0);
        }, intake),

        // keep the existing example method command if you want it included
        subsystem.exampleMethodCommand(),
        Commands.none()
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}