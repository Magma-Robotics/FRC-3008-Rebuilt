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
          intake.setflyWheel22();
          WaitCommand wait = new WaitCommand(3);
          Commands.waitUntil(() -> wait.isFinished());    
          intake.setIndexer(0.5);
          intake.setFeeder(0.7);
          WaitCommand wait2 = new WaitCommand(16);
          Commands.waitUntil(() -> wait2.isFinished());
          
          intake.setIndexer(0);
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
