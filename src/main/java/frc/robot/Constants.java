// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
   public static class cTalon {
    public static final double talon_1 = 0.25;
    public static final double talon_2=0.5;
    public static final double talon_3=0.75;
    public static final double talon_4=1;
  }
////////////////////////////// OLD CODE ////////////////////////////////////////////////////////////////
  public static final double ROBOT_MASS = (120) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.Inches.of(8).in(Meters)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.Feet.of(12.66).in(Meters);
  public static final double ALIGN_MAX_SPEED = Units.Meters.of(2).in(Meters);
  public static final double MAX_ACCEL  = Units.Meters.of(2).in(Meters);
  public static final double MAX_ANGV  = Units.Radians.of(5).in(Radians);
  public static final double MAX_ANGA = Units.Radians.of(4).in(Radians);

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
  
  public static class CANIds {
  }

  public static class Testing {
    //keep all as false unless testing positions
    public static final boolean testingArm = false;
    public static final boolean testingWrist = false;
    public static final boolean testingElevator = false;
    public static final boolean testingCoralIntake = false;
  }

  public static class Drive {
    public static final AngularVelocity TURN_SPEED = Units.DegreesPerSecond.of(360);
    public static final LinearVelocity OBSERVED_DRIVE_SPEED = Units.FeetPerSecond.of(12.66);
    public static class TELEOP_AUTO_ALIGN {
      public static final LinearVelocity DESIRED_AUTO_ALIGN_SPEED = Units.MetersPerSecond
          .of(MAX_SPEED / 4);

      public static final Distance MAX_AUTO_DRIVE_CORAL_STATION_DISTANCE = Units.Meters.of(10);
      public static final Distance MAX_AUTO_DRIVE_REEF_DISTANCE = Units.Meters.of(1);
      public static final Distance MAX_AUTO_DRIVE_PROCESSOR_DISTANCE = Units.Meters.of(3);

      public static final PIDController TRANS_CONTROLLER = new PIDController(
          1,
          0,
          0);
      public static final Distance AT_POINT_TOLERANCE = Units.Inches.of(0.5);

      public static final ProfiledPIDController ROTATION_CONTROLLER = new ProfiledPIDController(
          0.01, 0, 0, new TrapezoidProfile.Constraints(TURN_SPEED.in(Units.DegreesPerSecond),
              Math.pow(TURN_SPEED.in(Units.DegreesPerSecond), 2)));
      public static final Angle AT_ROTATION_TOLERANCE = Units.Degrees.of(1);

      public static final Distance AUTO_ALIGNMENT_TOLERANCE = Units.Inches.of(1);

      static {
        TRANS_CONTROLLER.setTolerance(AT_POINT_TOLERANCE.in(Units.Meters));

        ROTATION_CONTROLLER.enableContinuousInput(0, 360);
        ROTATION_CONTROLLER.setTolerance(AT_ROTATION_TOLERANCE.in(Units.Degrees));
      }

      public static HolonomicDriveController TELEOP_AUTO_ALIGN_CONTROLLER = new HolonomicDriveController(
          TRANS_CONTROLLER,
          TRANS_CONTROLLER,
          ROTATION_CONTROLLER);
    }
  }
}
