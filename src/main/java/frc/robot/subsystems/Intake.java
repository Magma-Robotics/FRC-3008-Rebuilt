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
    //private SparkMax intakePivot, indexer, feeder;
    //private SparkMaxConfig intakePivotConfig, indexerConfig, feederConfig;

    private final TalonFX intake;

    private TalonFXConfiguration intakeConfig;

    //private RelativeEncoder intakePivotEncoder;

    public Intake() {
        //change CAN IDs
        intake = new TalonFX(5);

        //intakePivotConfig = new SparkMaxConfig();
        //indexerConfig = new SparkMaxConfig();
        //feederConfig = new SparkMaxConfig();
        intakeConfig = new TalonFXConfiguration();

        
        intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        intake.getConfigurator().apply(intakeConfig);
    }
    //functions??
    //stop
    public void stopIntake() {
        intake.set(0);
    }
    public void setIntake(double speed) {
        intake.set(speed);
    }

    //public double getIntakePivotEncoderPosition() {
     //   return intakePivotEncoder.getPosition();
    //}

}