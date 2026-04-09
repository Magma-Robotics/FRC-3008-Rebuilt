package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase{
    
    private SparkMax hood = new SparkMax(15, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless); //Type in the CanID

    private SparkMaxConfig hoodConfig = new SparkMaxConfig();;
    
    public Hood() {

        hoodConfig
            .smartCurrentLimit(20)
            .inverted(false)
            .idleMode(IdleMode.kBrake);

        hood.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setHood(double speed) {
        hood.set(speed);
    }

    public void stopHood() {
        hood.set(0);
    }
}

