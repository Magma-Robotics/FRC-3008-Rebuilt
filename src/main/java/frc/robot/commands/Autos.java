package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  private final Intake intake;
  public static Command exampleAuto(ExampleSubsystem subsystem, Intake intake) {
    // Build an actual command sequence (do not create commands inside a runOnce lambda)
    return Commands.sequence(
        // Turn turret to the right for 0.75s
        Commands.runOnce(() -> intake.setTurretR(0.2), intake),
        new WaitCommand(0.75),

        // Stop turret
        Commands.runOnce(() -> intake.setTurretR(0.0), intake),

        // Spin up flywheel and wait
        Commands.runOnce(() -> intake.setflyWheel22(), intake),
        new WaitCommand(3),

        // Start indexing/feeding
        Commands.runOnce(() -> {
          intake.setIndexer(0.5);
          intake.setFeeder(0.7);
        }, intake),

        // Wait while shooting
        new WaitCommand(14),

        // Return turret to origin (runs until position reached), then stop turret
        Commands.run(() -> intake.setTurretToOrigin(0.1), intake)
            .until(() -> Math.abs(intake.getTurretPosition()) <= 0.01)
            .andThen(Commands.runOnce(intake::stopTurret, intake)),

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