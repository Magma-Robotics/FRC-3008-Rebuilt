package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;

public class Intake extends SubsystemBase{

    private SparkFlex intakePivot;

    private SparkFlexConfig intakePivotConfig;

    private final TalonFX intake, indexer;

    private TalonFXConfiguration intakeConfig, indexerConfig;

    private RelativeEncoder intakePivotEncoder;
    private GenericEntry pivotPositionEntry;

    public Intake() {

        intakePivot = new SparkFlex(16, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        intake = new TalonFX(24);
        indexer = new TalonFX(5);

        intakeConfig = new TalonFXConfiguration();
        indexerConfig = new TalonFXConfiguration(); 


        intakePivotConfig = new SparkFlexConfig();

        try {
            intakePivotEncoder = intakePivot.getEncoder();
        } catch (Exception e) {
            intakePivotEncoder = null;
            System.out.println("Warning: intakePivot encoder not available: " + e.getMessage());
        }

        try {
            pivotPositionEntry = Shuffleboard.getTab("Diagnostics").add("Intake Pivot Position", Double.NaN).getEntry();
        } catch (Exception e) {
            pivotPositionEntry = null;
            System.out.println("Warning: could not create Shuffleboard entry: " + e.getMessage());
        }

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

        intakePivot.configure(intakePivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        intake.getConfigurator().apply(intakeConfig);
        intake.getConfigurator().apply(currentConfigs);
        indexer.getConfigurator().apply(currentConfigs1);

    }

    /* public void setIntake(double speed) {
        intake.set(speed);
    } */

    public void stopIntake() {
        intake.set(0);
    }
    
    public void smartIntakeIN(double speed) {
        intake.set(Math.abs(speed));
        //indexer.set(Math.abs(speed*1.35));
    }

    public void smartIntakeOUT(double speed) {
        intake.set(Math.abs(speed));
        //indexer.set(Math.abs(speed*1.35));
    }

    public void autoIntake(double speed) {
        intake.set(speed);
        indexer.set(speed);
        new WaitCommand(5)
            .andThen(() -> stopIntake())
            .andThen(() -> stopIndexing());
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

    public void autoPivot(double speed) {
        intakePivot.set(speed);
        new WaitCommand(1);
        intakePivot.set(0);
    }

    public void stopPivot() {
        intakePivot.set(0);
    }

    /**
     * Return the intake pivot position from the SparkFlex encoder.
     * Returns Double.NaN if the encoder is unavailable or an error occurs.
     */

    public double getPivotPosition() {
        if (intakePivotEncoder == null) {
            return Double.NaN;
        }
        try {
            return intakePivotEncoder.getPosition();
        } catch (Exception e) {
            return Double.NaN;
        }
    }

    @Override
    public void periodic() {
        // Publish pivot position to Shuffleboard (if created)
        if (pivotPositionEntry != null) {
            double pos = getPivotPosition();
            try {
                pivotPositionEntry.setDouble(pos);
            } catch (Exception e) {
                // Ignore failures to write telemetry
            }
        }
    }
}