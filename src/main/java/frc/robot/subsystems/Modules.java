package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage; //Keep this line commented out
//import com.ctre.phoenix6.controls.velocity_duty_cycle.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;

public class Modules extends SubsystemBase{
    private SparkMax counterRoller;
    private SparkFlex intakePivot; //, lift;

    private SparkFlexConfig intakePivotConfig; //, liftConfig;
    private SparkMaxConfig counterRollerConfig;

    private final TalonFX intake, flyWheel1, flyWheel2, feeder, indexer;
    private final VelocityVoltage velocityRequestl, velocityRequestd, velocityRequestr, velocityRequestu;

    private TalonFXConfiguration intakeConfig, flyWheel1Config, flyWheel2Config, feederConfig, indexerConfig;

    // encoder for the intake pivot SparkFlex
    // private RelativeEncoder intakePivotEncoder;
    // Shuffleboard entry for pivot position
    // private GenericEntry pivotPositionEntry;
    
    // Duplicate TalonFX fields removed - use the canonical fields declared above:
    // intake, flyWheel1, flyWheel2, feeder, indexer

    public Modules() {
        //Declaration of CanIDs
        
        //SparkFlex
        intakePivot = new SparkFlex(16, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        //lift = new SparkFlex(17, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        //TalonFX
        intake = new TalonFX(24);
        flyWheel1 = new TalonFX(1);
        flyWheel2 = new TalonFX(0);
        feeder = new TalonFX(21);
        indexer = new TalonFX(5);

        //SparkMax - Minion
        counterRoller = new SparkMax(23, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        //counterRoller2 = new SparkMax(22, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless); 
        

        //Configurator

        //TalonFX
        intakeConfig = new TalonFXConfiguration();
        flyWheel2Config = new TalonFXConfiguration();
        flyWheel1Config = new TalonFXConfiguration();
        feederConfig = new TalonFXConfiguration();   
        indexerConfig = new TalonFXConfiguration(); 
        
        //SparkMax
        counterRollerConfig = new SparkMaxConfig(); 
        //counterRoller2Config = new SparkMaxConfig(); 

        //SparkFlex
        intakePivotConfig = new SparkFlexConfig();
        //liftConfig = new SparkFlexConfig();
        
        velocityRequestl = new VelocityVoltage(79);
        velocityRequestd = new VelocityVoltage(81);
        velocityRequestr = new VelocityVoltage(82);
        velocityRequestu = new VelocityVoltage(83);

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
            .smartCurrentLimit(20, 20, 4000)
            .inverted(false)
            .idleMode(IdleMode.kCoast);

        // l

        intakePivotConfig //sparkflex(idk if this works)
            .smartCurrentLimit(20)
            .inverted(false)
            .idleMode(IdleMode.kBrake);

        // // initialize intake pivot encoder (defensive)
        // try {
        //     intakePivotEncoder = intakePivot.getEncoder();
        // } catch (Exception e) {
        //     intakePivotEncoder = null;
        //     System.out.println("Warning: intakePivot encoder not available: " + e.getMessage());
        // }

        // // Shuffleboard entry
        // try {
        //     pivotPositionEntry = Shuffleboard.getTab("Diagnostics").add("Intake Pivot Position", Double.NaN).getEntry();
        // } catch (Exception e) {
        //     pivotPositionEntry = null;
        //     System.out.println("Warning: could not create Shuffleboard entry: " + e.getMessage());
        // }

        var currentConfigs1 = new CurrentLimitsConfigs();
        currentConfigs1.StatorCurrentLimit = 40; // Limits torque/acceleration
        currentConfigs1.StatorCurrentLimitEnable = true;
        currentConfigs1.SupplyCurrentLimit = 20; // Protects battery/breaker
        currentConfigs1.SupplyCurrentLimitEnable = true;

        var currentConfigs = new CurrentLimitsConfigs();
        currentConfigs.StatorCurrentLimit = 60; // Limits torque/acceleration
        currentConfigs.StatorCurrentLimitEnable = true;
        currentConfigs.SupplyCurrentLimit = 40; // Protects battery/breaker
        currentConfigs.SupplyCurrentLimitEnable = true;

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

        counterRoller.configure(counterRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakePivot.configure(intakePivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        feeder.getConfigurator().apply(feederConfig);
        intake.getConfigurator().apply(intakeConfig);
        flyWheel1.getConfigurator().apply(flyWheel1Config);
        flyWheel2.getConfigurator().apply(flyWheel2Config);

        intake.getConfigurator().apply(currentConfigs);
        // flyWheel1.getConfigurator().apply(currentConfigs);
        // flyWheel2.getConfigurator().apply(currentConfigs);
        feeder.getConfigurator().apply(currentConfigs);
        indexer.getConfigurator().apply(currentConfigs1);
    }


    // Controls Declaration

    //Shooter Module

    public void setflyWheelL() {
        flyWheel1.setControl(velocityRequestl);
        flyWheel2.setControl(velocityRequestl);
    }

    public void setflyWheelD() {
        flyWheel1.setControl(velocityRequestd);
        flyWheel2.setControl(velocityRequestd);
    }

    public void setflyWheelR() {
        flyWheel1.setControl(velocityRequestr);
        flyWheel2.setControl(velocityRequestr);
    }

    public void setflyWheelU() {
        flyWheel1.setControl(velocityRequestu);
        flyWheel2.setControl(velocityRequestu);
    }

    public void setFeeder(double speed) {
        feeder.set(speed);
        counterRoller.set(Math.abs(speed));
    }

    public void setFeederBack(double speed) {
        feeder.set(speed);
        counterRoller.set(-Math.abs(speed));
    }

    public void stopFeeding() {
        feeder.set(0);
        counterRoller.set(0);
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

    public void setPivot(double speed) {
        // double pos = Double.NaN;
        // if (intakePivotEncoder != null) {
        //     try {
        //         pos = intakePivotEncoder.getPosition();
        //     } catch (Exception e) {
        //         pos = Double.NaN;
        //     }
        // }

        // if (!Double.isNaN(pos)) {
        //     if (pos >= 3.999 && speed > 0) {
        //         intakePivot.set(0);
        //         return;
        //     }
        //     if (pos <= -4.585 && speed < 0) {
        //         intakePivot.set(0);
        //         return;
        //     }
        // }

        intakePivot.set(speed);
    }

    public void stopPivot() {
        intakePivot.set(0);
    }

    // /**
    //  * Return the intake pivot position from the SparkFlex encoder.
    //  * Returns Double.NaN if the encoder is unavailable or an error occurs.
    //  */
    // public double getPivotPosition() {
    //     if (intakePivotEncoder == null) {
    //         return Double.NaN;
    //     }
    //     try {
    //         return intakePivotEncoder.getPosition();
    //     } catch (Exception e) {
    //         return Double.NaN;
    //     }
    // }

    // @Override
    // public void periodic() {
    //     // Publish pivot position to Shuffleboard (if created)
    //     if (pivotPositionEntry != null) {
    //         double pos = getPivotPosition();
    //         try {
    //             pivotPositionEntry.setDouble(pos);
    //         } catch (Exception e) {
    //             // Ignore failures to write telemetry
    //         }
    //     }
    // }

    // public void setLift(double speed) {
    //     lift.set(speed);
    // }

    // public void stopLift() {
    //     lift.set(0);
    // }
    
}