/*
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private SparkMax turret, counterRoller, hood;
    private SparkMaxConfig turretConfig, counterRollerConfig, hoodConfig;

    private final TalonFX flywheel1;
    private final TalonFX flywheel2;

    private TalonFXConfiguration flywheel1Config;
    private TalonFXConfiguration flywheel2Config;

    private RelativeEncoder hoodEncoder;
    private RelativeEncoder turretEncoder;

    public Shooter() {
        //change CAN IDs
        turret = new SparkMax(5, MotorType.kBrushless);
        counterRoller = new SparkMax(5, MotorType.kBrushless);
        hood = new SparkMax(5, MotorType.kBrushless);
        flywheel1 = new TalonFX(5);
        flywheel2 = new TalonFX(5);

        turretConfig = new SparkMaxConfig();
        counterRollerConfig = new SparkMaxConfig();
        hoodConfig = new SparkMaxConfig();
        flywheel1Config = new TalonFXConfiguration();
        flywheel2Config = new TalonFXConfiguration();

        turretConfig
            .smartCurrentLimit(20)
            .inverted(false)
            .idleMode(IdleMode.kCoast);

        counterRollerConfig
            .smartCurrentLimit(20)
            .inverted(false)
            .idleMode(IdleMode.kCoast);

        hoodConfig
            .smartCurrentLimit(20)
            .inverted(false)
            .idleMode(IdleMode.kBrake); //assuming it's brake -- Hunter

        flywheel1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        flywheel1Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        flywheel2Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        flywheel2Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;


        hoodEncoder = hood.getEncoder();
        turretEncoder = turret.getEncoder();

        turret.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        counterRoller.configure(counterRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        hood.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flywheel1.getConfigurator().apply(flywheel1Config);
        flywheel2.getConfigurator().apply(flywheel2Config);
    }
    //stop
    public void stopHood() {
        hood.set(0);
    }
    public void stopCounterRoller() {
        counterRoller.set(0);
    }

    public void stopFlyWheel() {
        flywheel1.set(0);
        flywheel2.set(0);
    }

    public void stopTurret() {
        turret.set(0);
    }
    //set
    public void setTurret(double speed) {
        turret.set(speed);
    }

    public void setHood(double speed) {
        hood.set(speed);
    }

    public void setCounterRoller(double speed) {
        counterRoller.set(speed);
    }

    public void setFlywheel(double speed) {
        flywheel1.set(speed);
        flywheel2.set(speed);
    }

    public double getHoodEncoderPosition() {
        return hoodEncoder.getPosition();
    }

    public double getturretEncoderPosition() {
        return turretEncoder.getPosition();
    }

} 
    */
