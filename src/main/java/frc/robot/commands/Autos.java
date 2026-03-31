package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Modules;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public final class Autos {
  public static Command exampleAuto(ExampleSubsystem subsystem, Modules modules) {
    // Build an actual command sequence (do not create commands inside a runOnce lambda)
    return Commands.sequence(
      
      // new WaitCommand(1.3),// Time 0.3 elapsed

      // // Run pivot and shooting at the same time:
      // // - Pivot runs for 1.00s and then stops
      // // - Shooting (flywheels + feeder/indexer) runs for 2.83s and then stops
      // Commands.parallel(
      //   // Pivot sequence
      //   Commands.sequence(
      //     Commands.runOnce(() -> modules.setPivot(0.5), modules),
      //     new WaitCommand(0.83),
      //     Commands.runOnce(() -> modules.stopPivot(), modules)
      //   ),

      //   // Shooting sequence
      //   Commands.sequence(
      //     // start flywheels and feeding
      //     Commands.runOnce(() -> {
      //       modules.setflyWheel22(); // use the higher velocity configuration
      //       modules.setIndexer(0.5);
      //       modules.setFeeder(0.6);
      //     }, modules),
      //     new WaitCommand(2.83),
      //     // stop everything related to shooting
      //     Commands.runOnce(() -> {
      //       modules.stopIndexing();
      //       modules.stopFeeding();
      //       modules.stopflyWheel();
      //     }, modules)
      //   )
      // ),

      // Commands.runOnce(() -> modules.setIntake(0.6), modules),

      // new WaitCommand(2.09), // Time 6.31 elapsed
      // Commands.runOnce(() -> modules.setIntake(0), modules),

      // new WaitCommand(1.16), //time 7.47 elapsed

      // Commands.runOnce(() -> modules.setIntake(0.6), modules),

      // new WaitCommand(2.4), // Time 14.49 elapsed

      // Commands.runOnce(() -> modules.setIntake(0), modules),

      // Turn turret to the right for 0.75s


      // Commands.runOnce(() -> intake.setTurretL(0.1),intake),
      // new WaitCommand(0.367),

      // // Stop turret
      // Commands.runOnce(() -> intake.setTurretL(0.0), intake),
                                        
      Commands.runOnce(() -> modules.setflyWheelR(), modules),

      new WaitCommand(3),

      Commands.runOnce(() -> {
        modules.setFeeder(1);
        modules.setIndexer(0.75);
      }),

      new WaitCommand(10),
      
      Commands.runOnce(() -> {
        modules.setFeeder(0);
        modules.setIndexer(0);
        modules.setFlywheelZero(0);
      }),

                        // keep the existing example method command if you want it included
      subsystem.exampleMethodCommand(),
      Commands.none()
    );
  }

                  // private Autos() {
                  //   throw new UnsupportedOperationException("This is a utility class!");
                  // }
}