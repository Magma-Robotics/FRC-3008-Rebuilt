package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Shooter extends SubsystemBase{
    private SparkMax counterRoller;

    private SparkMaxConfig counterRollerConfig = new SparkMaxConfig();

    private final TalonFX flyWheel1, flyWheel2, feeder;
    private final VelocityVoltage velocityRequestl, velocityRequestd, velocityRequestr, velocityRequestu, velocityRequesta;

    private TalonFXConfiguration flyWheel1Config, flyWheel2Config, feederConfig;


    public Shooter() {
        
        flyWheel1 = new TalonFX(1);
        flyWheel2 = new TalonFX(0);
        feeder = new TalonFX(21);

        counterRoller = new SparkMax(23, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

        flyWheel2Config = new TalonFXConfiguration();
        flyWheel1Config = new TalonFXConfiguration();
        feederConfig = new TalonFXConfiguration();   

        /*All of the request is named after the Dpad
         *     U
         *   L   R         The D pad accordingly
               D                                    */

        velocityRequesta = new VelocityVoltage(83);
        velocityRequestl = new VelocityVoltage(79);
        velocityRequestd = new VelocityVoltage(81);
        velocityRequestr = new VelocityVoltage(82);
        velocityRequestu = new VelocityVoltage(83);

        var slot0 = flyWheel1Config.Slot0;
        slot0.kP = 0.12;
        slot0.kI = 0.0;  
        slot0.kD = 0.01; 
        slot0.kV = 0.11;  

        var slot1 = flyWheel2Config.Slot0;
        slot1.kP = 0.12; 
        slot1.kI = 0.0;  
        slot1.kD = 0.01; 
        slot1.kV = 0.11; 

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

        feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        flyWheel1Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        flyWheel1Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        flyWheel2Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        flyWheel2Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        counterRoller.configure(counterRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        feeder.getConfigurator().apply(feederConfig);
        feeder.getConfigurator().apply(currentConfigs);

        flyWheel1.getConfigurator().apply(flyWheel1Config);
        flyWheel2.getConfigurator().apply(flyWheel2Config);
    }

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

    public void autonomousShoot() {
        flyWheel1.setControl(velocityRequesta);
        flyWheel2.setControl(velocityRequesta);
        new WaitCommand(2)
            .andThen(() -> setFeeder(1));
            //.andThen(() -> hood positioning);
        new WaitCommand(5)
            .andThen(() -> stopFeeding())
            .andThen(() -> stopflyWheel());
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
}