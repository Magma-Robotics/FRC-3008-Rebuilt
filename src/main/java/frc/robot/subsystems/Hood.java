package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Hood extends SubsystemBase{
    //private final SparkAbsoluteEncoder m_hoodEncoder;


    private RelativeEncoder hoodEncoder;

    private SparkMax hood = new SparkMax(15, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless); //Type in the CanID

    private SparkMaxConfig hoodConfig = new SparkMaxConfig();
    
    public Hood() {

        hoodEncoder = hood.getEncoder();
        hoodConfig
            .smartCurrentLimit(20)
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .encoder
            .positionConversionFactor(1);

        hood.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getHoodAngle() {
        // Use the new name here
        return hoodEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // Now you publish using the renamed variable's getter
        SmartDashboard.putNumber("Hood Angle", getHoodAngle());
        
        // Optional: Also publish velocity to see if it's moving smoothly
        SmartDashboard.putNumber("Hood Velocity", hoodEncoder.getVelocity());

        double swerveX = SmartDashboard.getNumber("Swerve X", 0);
        double swerveY = SmartDashboard.getNumber("Swerve Y", 0);
    }

    public void setHood(double speed) {
        speed = -speed;
        hood.set(speed);
    }

    public void setHoodSpeed(double speed) {
        hood.set(-speed);
    }

    public double getHoodPos() {
        return hoodEncoder.getPosition();
    }

    public void stopHood() {
        hood.set(0);
    }
}

