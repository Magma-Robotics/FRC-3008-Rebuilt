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

public class Intake extends SubsystemBase{
    private SparkMax intakePivot, indexer;
    private SparkMaxConfig intakePivotConfig, indexerConfig;

    private final TalonFX intake;

    private TalonFXConfiguration intakeConfig;

    private RelativeEncoder intakePivotEncoder;

    public Intake() {
        intakePivot = new SparkMax(0, MotorType.kBrushless);
        indexer = new SparkMax(0, MotorType.kBrushless);

        intake = new TalonFX(0);

        intakePivotConfig = new SparkMaxConfig();
        indexerConfig = new SparkMaxConfig();
        intakeConfig = new TalonFXConfiguration();

        intakePivotConfig
            .smartCurrentLimit(20)
            .inverted(false)
            .idleMode(IdleMode.kBrake);

        indexerConfig
            .smartCurrentLimit(20)
            .inverted(false)
            .idleMode(IdleMode.kBrake);

        intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        intakePivotEncoder = intakePivot.getEncoder();

        intakePivot.configure(intakePivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        indexer.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intake.getConfigurator().apply(intakeConfig);
    }

    public void stopIndexer() {
        indexer.set(0);
    }

    public void stopIntakePivot() {
        intakePivot.set(0);
    }

    public void stopIntake() {
        intake.set(0);
    }

    public void setIndexer(double speed) {
        indexer.set(speed);
    }

    public void setIntakePivot(double speed) {
        intakePivot.set(speed);
    }

    public void setIntake(double speed) {
        intake.set(speed);
    }

    public double getIntakePivotEncoderPosition() {
        return intakePivotEncoder.getPosition();
    }

}
