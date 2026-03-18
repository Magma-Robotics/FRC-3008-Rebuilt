// ...existing code...
package frc.robot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.SwerveDrive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private final Timer disabledTimer = new Timer();

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    // Measure how long the scheduler run takes to help diagnose loop overruns.
    long t0 = System.nanoTime();
    // Run the Scheduler. This is responsible for polling buttons, scheduling,
    // running, and ending commands.
    CommandScheduler.getInstance().run();
    long elapsedMs = (System.nanoTime() - t0) / 1_000_000;
    // Report if the periodic loop is taking longer than 15 ms (leaves some margin below 20 ms)
    if (elapsedMs > 15) {
      String msg = "robotPeriodic loop overrun: " + elapsedMs + " ms";
      System.out.println(msg);
      DriverStation.reportWarning(msg, false);
    }
  }

  @Override
  public void disabledInit() {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    // Get the autonomous command from the container and schedule it.
    // Guard with try/catch so a thrown exception in construction/scheduling
    // doesn't kill the robot process and cause a restart.
    try {
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();
      if (m_autonomousCommand != null) {
        m_autonomousCommand.schedule();
        System.out.println("Scheduled autonomous command: " + m_autonomousCommand.getName());
      }
    } catch (Exception e) {
      // Print stack trace to console and report to DriverStation so you can see the error
      e.printStackTrace();
      DriverStation.reportError("Failed to start autonomous: " + e.getMessage(), false);
      // Fallback to no-op command to ensure robot stays running
      m_autonomousCommand = edu.wpi.first.wpilibj2.command.Commands.none();
    }
  }

  @Override
  public void autonomousPeriodic() {
    // No explicit scheduler call here; robotPeriodic runs CommandScheduler
  }

  @Override
  public void teleopInit() {
    // If autonomous is still running when teleop starts, cancel it
    if (m_autonomousCommand != null && m_autonomousCommand.isScheduled()) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  //limelight helpers
  double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= SwerveDrive.kMaxAngularSpeed;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  @Override
  public void testInit() {
    // Cancel all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    //SparkMax(22, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless)

    // test for the sparkmax for the hood to be locked
          // SparkMax hood = new SparkMax(22, MotorType.kBrushless);
          // hood.set(0);
          // hood.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void testPeriodic() {}
}
// ...existing code...