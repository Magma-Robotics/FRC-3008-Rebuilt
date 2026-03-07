package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  public static Command exampleAuto(ExampleSubsystem subsystem, Intake intake) {
    // Build an actual command sequence (do not create commands inside a runOnce lambda)
    return Commands.sequence(
      
      new WaitCommand(1.3),// Time 0.3 elapsed

      // Run pivot and shooting at the same time:
      // - Pivot runs for 1.00s and then stops
      // - Shooting (flywheels + feeder/indexer) runs for 2.83s and then stops
      Commands.parallel(
        // Pivot sequence
        Commands.sequence(
          Commands.runOnce(() -> intake.setPivot(0.5), intake),
          new WaitCommand(0.83),
          Commands.runOnce(() -> intake.stopPivot(), intake)
        ),

        // Shooting sequence
        Commands.sequence(
          // start flywheels and feeding
          Commands.runOnce(() -> {
            intake.setflyWheel22(); // use the higher velocity configuration
            intake.setIndexer(0.5);
            intake.setFeeder(0.6);
          }, intake),
          new WaitCommand(2.83),
          // stop everything related to shooting
          Commands.runOnce(() -> {
            intake.stopIndexing();
            intake.stopFeeding();
            intake.stopflyWheel();
          }, intake)
        )
      ),

      Commands.runOnce(() -> intake.setIntake(0.6), intake),

      new WaitCommand(2.09), // Time 6.31 elapsed

      Commands.runOnce(() -> intake.setIntake(0), intake),

      new WaitCommand(1.16), //time 7.47 elapsed

      Commands.runOnce(() -> intake.shootingMovements(2.66), intake), // Time 10.13 elapsed

      new WaitCommand(1.96), // Time 12.09 elapsed

      Commands.runOnce(() -> intake.setIntake(0.6), intake),

      new WaitCommand(2.4), // Time 14.49 elapsed

      Commands.runOnce(() -> intake.setIntake(0), intake),








                  //       // Turn turret to the right for 0.75s


                  //                       // Commands.runOnce(() -> intake.setTurretL(0.1),intake),
                  //                       // new WaitCommand(0.367),

                  //                       // // Stop turret
                  //                       // Commands.runOnce(() -> intake.setTurretL(0.0), intake),
                                        
                  //       // Hood Up      
                  //       Commands.runOnce(() -> intake.hoodUp(0.05), intake),
                  //       new WaitCommand(0.25),

                  //       // Stops Hood from Overshooting
                  //       Commands.runOnce(() -> intake.hoodUp(0.0), intake),

                        
                  //       // Spin up flywheel and wait
                  //       Commands.runOnce(() -> intake.setflyWheel33(), intake),
                  //       new WaitCommand(1.5),

                  //       // Start indexing/feeding
                  //       Commands.runOnce(() -> {
                  //         intake.setIndexer(0.5);
                  //         intake.setFeeder(0.6);
                  //       }, intake),

                  //       // Wait while shooting
                  //       new WaitCommand(2),
                        
                  //       Commands.runOnce(() -> intake.shootingMovements(), intake),
                  //       Commands.runOnce(() -> intake.stopflyWheel()),
                        
                  //       new WaitCommand(2.5),


                  //       Commands.runOnce(() -> intake.setFlywheelZero(0)),
                        
                  //                         // //going back
                  //                         // Commands.runOnce(() -> intake.setTurretR(0.1)),
                  //                         // new WaitCommand(0.367),

                  //                         // Commands.runOnce(() -> intake.setTurretR(0)),






                  //       // Return turret to origin (runs until position reached), then stop turret)

                  //       // Commands.run(() -> intake.setTurretToOrigin(0.1), intake)
                  //       //     .until(() -> Math.abs(intake.getTurretPosition()) <= 0.01)
                  //       //     .andThen(Commands.runOnce(intake::stopTurret, intake)),

                  //       // Hood Down
                  //       Commands.runOnce(() -> intake.hoodDown(0.05), intake),
                  //       new WaitCommand(0.25),
                        
                  //       // Stops the hood to dig down too much to avoid motor burns
                  //       Commands.runOnce(() -> intake.hoodDown(0.0), intake),

                  //       // Ensure everything is off
                  //       Commands.runOnce(() -> {
                  //         intake.setIndexer(0);
                  //         intake.setFeeder(0);
                  //         intake.setFlywheelZero(0);
                  //       }, intake),

                  //       // keep the existing example method command if you want it included
      subsystem.exampleMethodCommand(),
      Commands.none()
    );
  }

                  // private Autos() {
                  //   throw new UnsupportedOperationException("This is a utility class!");
                  // }
}