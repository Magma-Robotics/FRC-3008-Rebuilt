package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
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
    private SparkClosedLoopController hoodController = hood.getClosedLoopController();
    private double autoHoodPos = 0.0;
    private double manualHoodSpeed = 0.0;

    private double MIN_HOOD_POS = 0.1;

    private double MAX_HOOD_POS = 2.214;
    private double MIN_DISTANCE_TO_GOAL = 1.62;
    private double MAX_DISTANCE_TO_GOAL = 4.11;

    private Boolean autoHoodMode = false;
    private Boolean autoHoodRestMode = false;
    
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
        SmartDashboard.putNumber("Hood Position", getHoodAngle());
        
        // Optional: Also publish velocity to see if it's moving smoothly
        SmartDashboard.putNumber("Hood Velocity", hoodEncoder.getVelocity());

        double distToGoal = SmartDashboard.getNumber("DistanceToGoal", 0.0);

        if (autoHoodMode) {
            if(!autoHoodRestMode){
                autoHoodPos = MIN_HOOD_POS + ((distToGoal - MIN_DISTANCE_TO_GOAL)/(MAX_DISTANCE_TO_GOAL - MIN_DISTANCE_TO_GOAL)) * MAX_HOOD_POS;
                autoHoodPos = Math.max(MIN_HOOD_POS, Math.min(MAX_HOOD_POS, autoHoodPos));
            }
            double currentPos = getHoodPos();
            if(currentPos - autoHoodPos > 0.2) {
                setHood(-0.1);
            }
            else if (currentPos - autoHoodPos < -0.2){
                setHood(0.7);
            }
            else{
                setHood(0);
            }
            SmartDashboard.putNumber("HoodAutoTarget", autoHoodPos);
        }
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

    public void hoodAutoEnable(){
        autoHoodRestMode = false;
        autoHoodMode = true;
    }

    public void hoodAutoDisable(){
        autoHoodMode = false;
    }

    public void hoodAutoRetract(){
        autoHoodPos = MIN_HOOD_POS;
        autoHoodRestMode = true;
    }

    public void requestHoodSpeed(double speed){
        manualHoodSpeed = speed;
    }
}

