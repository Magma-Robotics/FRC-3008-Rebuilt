package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private SparkMax feeder, counterRoller, indexer;
    private SparkFlex intakePivot;

    private SparkFlexConfig intakePivotConfig;
    private SparkMaxConfig feederConfig, counterRollerConfig, indexerConfig;

    private final TalonFX intake, flyWheel1, flyWheel2, turret;
    private final VelocityVoltage velocityRequest2, velocityRequest1;

    private TalonFXConfiguration intakeConfig, flyWheel1Config, flyWheel2Config, turretConfig;

    //private RelativeEncoder intakePivotEncoder;

    public Intake() {
        //change CAN IDs
        intakePivot = new SparkFlex(16, MotorType.kBrushless);
        //indexer = new SparkMax(5, MotorType.kBrushless);
        intake = new TalonFX(24);
        flyWheel1 = new TalonFX(1);
        flyWheel2 = new TalonFX(0);
        turret = new TalonFX(26); //change me tmr
        //Minions
        counterRoller = new SparkMax(23, MotorType.kBrushless); //22 = hood
        feeder = new SparkMax(21, MotorType.kBrushless);
        indexer = new SparkMax(5, MotorType.kBrushless); //21 = feeder

        intakeConfig = new TalonFXConfiguration();
        flyWheel2Config = new TalonFXConfiguration();
        flyWheel1Config = new TalonFXConfiguration();
        turretConfig = new TalonFXConfiguration();
        feederConfig = new SparkMaxConfig();    
        counterRollerConfig = new SparkMaxConfig();  
        indexerConfig = new SparkMaxConfig(); 

        intakePivotConfig = new SparkFlexConfig();


        
        VelocityVoltage velocityRequest = new VelocityVoltage(50);
        velocityRequest2 = new VelocityVoltage(100);
        velocityRequest1 = new VelocityVoltage(50);

        var slot0 = flyWheel1Config.Slot0;
        slot0.kP = 0.12;  // Proportional gain
        slot0.kI = 0.0;   // Integral gain
        slot0.kD = 0.01;  // Derivative gain
        slot0.kV = 0.11;  // Feedforward (Crucial for velocity)

        var slot1 = flyWheel2Config.Slot0;
        slot1.kP = 0.12;  // Proportional gain
        slot1.kI = 0.0;   // Integral gain
        slot1.kD = 0.01;  // Derivative gain
        slot1.kV = 0.11;  // Feedforward (Crucial for velocity)
        //intakePivotConfig
         //   .smartCurrentLimit(20)
         //   .inverted(false)
         //   .idleMode(IdleMode.kCoast);

        //indexerConfig
         //   .smartCurrentLimit(20)
         //   .inverted(false)
         //   .idleMode(IdleMode.kCoast);

        feederConfig
            .smartCurrentLimit(20)
            .inverted(true)
            .idleMode(IdleMode.kCoast);
        counterRollerConfig
            .smartCurrentLimit(20)
            .inverted(false)
            .idleMode(IdleMode.kCoast);
        counterRollerConfig
            .smartCurrentLimit(20)
            .inverted(false)
            .idleMode(IdleMode.kCoast);
        intakePivotConfig //sparkflex(idk if this works)
            .smartCurrentLimit(20)
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        indexerConfig
            .smartCurrentLimit(20)
            .inverted(false)
            .idleMode(IdleMode.kBrake);

        intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

         flyWheel1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        flyWheel1Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        flyWheel2Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        flyWheel2Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;



        //intakePivotEncoder = intakePivot.getEncoder();

        //intakePivot.configure(intakePivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       // indexer.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        counterRoller.configure(counterRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        feeder.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intake.getConfigurator().apply(intakeConfig);
        flyWheel1.getConfigurator().apply(flyWheel1Config);
        flyWheel2.getConfigurator().apply(flyWheel2Config);
        turret.getConfigurator().apply(turretConfig);
        //indexer.getConfigurator().apply(indexerConfig);
    }
    //functions??
    //turn-on
    public void setTurret(double speed) {
        turret.set(speed);
    }
    
    public void setIntake(double speed) {
        intake.set(speed);
    }

    public void setflyWheel11() {
        flyWheel1.setControl(velocityRequest1);
        flyWheel2.setControl(velocityRequest1);
    }

        public void setflyWheel22() {
        flyWheel1.setControl(velocityRequest2);
        flyWheel2.setControl(velocityRequest2);
    }
     public void setFeeder(double speed) {
        feeder.set(speed);
        counterRoller.set(speed);

    }
     public void setIndexer(double speed) {
     indexer.set(speed);
    }

    public void setPivot(double speed) {
        intakePivot.set(speed);
    }
    
    //stop functions
    public void stopTurret() {
        turret.set(0);
    }
    public void stopIntake() {
        intake.set(0);
    }
    public void stopflyWheel() {
        flyWheel1.set(0);
        flyWheel2.set(0);
    }
    public void stopFeeding() {
        feeder.set(0);
        counterRoller.set(0);
    }
    public void stopIndexing() {
        indexer.set(0);
    }
    public void stopPivot() {
        intakePivot.set(0);
    }
    //public double getIntakePivotEncoderPosition() {
    //    return intakePivotEncoder.getPosition();
    //}

}
