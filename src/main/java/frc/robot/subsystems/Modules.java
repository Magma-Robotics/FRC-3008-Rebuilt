package frc.robot.subsystems;

import org.opencv.core.Mat;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.ColorSensorV3.MainControl;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;

public class Modules extends SubsystemBase{
    private SparkMax counterRoller, counterRoller2;
    // private SparkFlex intakePivot, lift;

    // private SparkFlexConfig intakePivotConfig, liftConfig;
    private SparkMaxConfig counterRollerConfig, counterRoller2Config;

    private final TalonFX intake, flyWheel1, flyWheel2, feeder, indexer;
    private final VelocityVoltage velocityRequest2, velocityRequest1, velocityRequest3;

    private TalonFXConfiguration intakeConfig, flyWheel1Config, flyWheel2Config, turretConfig, feederConfig, indexerConfig;

    public Modules() {
        //Declaration of CanIDs
        
        //SparkFlex
        //intakePivot = new SparkFlex(16, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        //lift = new SparkFlex(17, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

        //TalonFX
        intake = new TalonFX(24);
        flyWheel1 = new TalonFX(1);
        flyWheel2 = new TalonFX(0);
        feeder = new TalonFX(21);
        indexer = new TalonFX(5);

        //SparkMax - Minion
        counterRoller = new SparkMax(23, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        counterRoller2 = new SparkMax(22, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless); 
        

        //Configurator

        //TalonFX
        intakeConfig = new TalonFXConfiguration();
        flyWheel2Config = new TalonFXConfiguration();
        flyWheel1Config = new TalonFXConfiguration();
        turretConfig = new TalonFXConfiguration();
        feederConfig = new TalonFXConfiguration();   
        indexerConfig = new TalonFXConfiguration(); 
        
        //SparkMax
        counterRollerConfig = new SparkMaxConfig(); 
        counterRoller2Config = new SparkMaxConfig(); 

        //SparkFlex
        //intakePivotConfig = new SparkFlexConfig();
        //liftConfig = new SparkFlexConfig();
        
        VelocityVoltage velocityRequest = new VelocityVoltage(50);
        velocityRequest2 = new VelocityVoltage(80);
        velocityRequest1 = new VelocityVoltage(50);
        velocityRequest3 = new VelocityVoltage(67);

        //PID vals for the shooter
        
        var slot0 = flyWheel1Config.Slot0;
        slot0.kP = 0.12;  // Proportional gain
        slot0.kI = 0.0;   // Integral gainl
        slot0.kD = 0.01;  // Derivative gain
        slot0.kV = 0.11;  // Feedforward (Crucial for velocity)

        var slot1 = flyWheel2Config.Slot0;
        slot1.kP = 0.12;  // Proportional gain
        slot1.kI = 0.0;   // Integral gain
        slot1.kD = 0.01;  // Derivative gain
        slot1.kV = 0.11;  // Feedforward (Crucial for velocity)

        //Configurator Definitions;

        counterRollerConfig
            .smartCurrentLimit(20)
            .inverted(false)
            .idleMode(IdleMode.kCoast);

        counterRoller2Config
            .smartCurrentLimit(20)
            .inverted(false)
            .idleMode(IdleMode.kCoast);

        // intakePivotConfig //sparkflex(idk if this works)
        //     .smartCurrentLimit(20)
        //     .inverted(false)
        //     .idleMode(IdleMode.kBrake);
        

        indexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        flyWheel1Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        flyWheel1Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        flyWheel2Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        flyWheel2Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        counterRoller.configure(counterRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        counterRoller2.configure(counterRoller2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        feeder.getConfigurator().apply(feederConfig);
        intake.getConfigurator().apply(intakeConfig);
        flyWheel1.getConfigurator().apply(flyWheel1Config);
        flyWheel2.getConfigurator().apply(flyWheel2Config);
    }


    // Controls Declaration

    //Shooter Module

    public void setflyWheel11() {
        flyWheel1.setControl(velocityRequest1);
        flyWheel2.setControl(velocityRequest1);
    }

    public void setflyWheel22() {
        flyWheel1.setControl(velocityRequest2);
        flyWheel2.setControl(velocityRequest2);
    }

    public void setflyWheel33() {
        flyWheel1.setControl(velocityRequest3);
        flyWheel2.setControl(velocityRequest3);
    }

    public void setFeeder(double speed) {
        feeder.set(speed);
        counterRoller.set(Math.abs(speed));
        counterRoller2.set(-Math.abs(speed));
    }

    public void setFeederBack(double speed) {
        feeder.set(speed);
        counterRoller.set(-Math.abs(speed));
        counterRoller2.set(Math.abs(speed));
    }

    public void stopFeeding() {
        feeder.set(0);
        counterRoller.set(0);
        counterRoller2.set(0);
    }

    public void stopflyWheel() {
        flyWheel1.set(0);
        flyWheel2.set(0);
    }

    public void setFlywheelZero(double speed) {
        flyWheel1.set(Math.abs(speed));
        flyWheel2.set(Math.abs(speed));
    }





    //Intake Modules

    public void setIntake(double speed) {
        intake.set(speed);
    }

    public void stopIntake() {
        intake.set(0);
    }
    
    public void setIndexer(double speed) {
        indexer.set(speed);
    }

    public void stopIndexing() {
        indexer.set(0);
    }

    // public void setPivot(double speed) {
    //     intakePivot.set(speed);
    // }

    // public void stopPivot() {
    //     intakePivot.set(0);
    // }

    // public void setLift(double speed) {
    //     lift.set(speed);
    // }

    // public void stopLift() {
    //     lift.set(0);
    // }
    
}