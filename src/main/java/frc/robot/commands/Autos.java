package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  private final Intake intake;
  public static Command exampleAuto(ExampleSubsystem subsystem, Intake intake) {
    // Set intake during autonomous as needed; replace the lambda body with the actual intake call when available
    return Commands.sequence(
        Commands.runOnce(() -> {
          intake.setTurretR(0.2); //Turret Turning
          WaitCommand waita = new WaitCommand(0.75); // wait a - turret facing to the shooter manually
          Commands.waitUntil(() -> waita.isFinished());
          intake.setTurretR(0);

          intake.setflyWheel22();   // Flywheel Setup
          WaitCommand waitb = new WaitCommand(3);   // wait b - setting and reving up the motors to start to shoot
          Commands.waitUntil(() -> waitb.isFinished());    
          intake.setIndexer(0.5);  // Indexer misc settings
          intake.setFeeder(0.7);

          WaitCommand waitc = new WaitCommand(14);     // wait c - waiting for all of the balls to be finished shoorting
          Commands.waitUntil(() -> waitc.isFinished());

          // intake.setTurretL(0.2); //Turret Turning
          // WaitCommand waitd = new WaitCommand(0.75);    // wait d - turning the turret to its original face to consider for the init for the teleop
          // Commands.waitUntil(() -> waitd.isFinished());
          // intake.setTurretL(0);

          Commands.run(() -> intake.setTurretToOrigin(0.1), intake)
              .until(() -> Math.abs(intake.getTurretPosition()) <= 0.01)
              .andThen(Commands.runOnce(() -> intake.stopTurret(), intake));
          
          intake.setIndexer(0); // turn off everything for the teleop init to avoid system break
          intake.setFeeder(0);
          intake.setFlywheelZero(0);
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
