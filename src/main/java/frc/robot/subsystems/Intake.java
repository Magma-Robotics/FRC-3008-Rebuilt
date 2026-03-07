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

public class Intake extends SubsystemBase{
    private SparkMax counterRoller, hood;
    private SparkFlex intakePivot, lift;

    private SparkFlexConfig intakePivotConfig, liftConfig;
    private SparkMaxConfig counterRollerConfig, hoodConfig;

    private final TalonFX intake, flyWheel1, flyWheel2, turret, feeder, indexer;
    private final VelocityVoltage velocityRequest2, velocityRequest1, velocityRequest3;

    private TalonFXConfiguration intakeConfig, flyWheel1Config, flyWheel2Config, turretConfig, feederConfig, indexerConfig;

    //private RelativeEncoder intakePivotEncoder;

    // keep a periodically-updated cached turret position so commands using setTurretToOrigin/readers see fresh values
    private volatile double turretPosition = 0.0;

    ////Mr. Reid's Code///////
    private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

    public Intake() {
        //change CAN IDs
        intakePivot = new SparkFlex(16, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless); //intake pivot
        lift = new SparkFlex(17, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless); //lift
        //indexer = new SparkMax(5, M,otorType.kBrushless);
        intake = new TalonFX(24);
        flyWheel1 = new TalonFX(1);
        flyWheel2 = new TalonFX(0);
        turret = new TalonFX(20); //change me tmr
        
        //Minions
        counterRoller = new SparkMax(23, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless); //22 = hood
        hood = new SparkMax(22, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless); //22 = hood
        feeder = new TalonFX(21);
        indexer = new TalonFX(5); //21 = feeder

        intakeConfig = new TalonFXConfiguration();
        flyWheel2Config = new TalonFXConfiguration();
        flyWheel1Config = new TalonFXConfiguration();
        turretConfig = new TalonFXConfiguration();
        //Mr. Reid's Code
        // turretConfig.Slot0.kP = 50;
        // turretConfig.Slot0.kI = 0.0;
        // turretConfig.Slot0.kD = 0.01;
        // turret.setPosition(0);
        ///
        
        feederConfig = new TalonFXConfiguration();   
        counterRollerConfig = new SparkMaxConfig(); 
        hoodConfig = new SparkMaxConfig();
        indexerConfig = new TalonFXConfiguration(); 

        intakePivotConfig = new SparkFlexConfig();
        liftConfig = new SparkFlexConfig();

        
        VelocityVoltage velocityRequest = new VelocityVoltage(50);
        velocityRequest2 = new VelocityVoltage(80);
        velocityRequest1 = new VelocityVoltage(50);
        velocityRequest3 = new VelocityVoltage(75);

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

        indexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

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
        feeder.getConfigurator().apply(feederConfig);
        intake.getConfigurator().apply(intakeConfig);
        flyWheel1.getConfigurator().apply(flyWheel1Config);
        flyWheel2.getConfigurator().apply(flyWheel2Config);
        turret.getConfigurator().apply(turretConfig);
        //indexer.getConfigurator().apply(indexerConfig);
    }

    // // update cached sensors every scheduler tick so commands see fresh values
    // @Override
    // public void periodic() {
    //     try {
    //         turretPosition = turret.getPosition().getValueAsDouble();
    //     } catch (Exception e) {
    //         // keep last known value on read failure
    //     }
    // }

    //functions??
    //turn-on

    // /////Mr. Reid's Code//////
    // public Command autoTurret1(){
    //     return runOnce(
    //         () -> {
    //             turret.setControl(m_positionVoltage.withPosition(-5.0)); // Adjust the position as needed);
    //         }
    //     );
    // }


    public void setTurretR(double speed) {
        // use cached turretPosition and maintain soft limits
        // clamp speed to [-1,1]
        double p = Math.max(-1.0, Math.min(1.0, speed));

        if (p > 0) {
            if (turretPosition >= 3.999) {
                turret.set(0);
                return;
            }
        } else if (p < 0) {
            if (turretPosition <= -4.585) {
                turret.set(0);
                return;
            }
        }

        turret.set(p);
    }
    
    public void setTurretL(double speed) {
        // same behavior as setTurretR (keeps original calling signatures)
        setTurretR(speed);
    }

    public void hoodUp(double speed) {
        hood.set(Math.abs(speed));
    }

    public void hoodDown(double speed) {
        hood.set(-Math.abs(speed));
    }
    
    // public void setTurretToOrigin(double speed) {
    //     // non-blocking: call repeatedly until getTurretPosition() within tolerance, then stop.
    //     final double TOLERANCE = 0.01; // adjust for your encoder units
    //     double pos = turretPosition;

    //     if (Double.isNaN(pos)) {
    //         // defensive: if we don't have a valid reading, don't drive
    //         turret.set(0);
    //         return;
    //     }

    //     if (Math.abs(pos) <= TOLERANCE) {
    //         turret.set(0);
    //         return;
    //     }

    //     // drive toward zero using sign of position
    //     double direction = -Math.signum(pos); // +1 or -1 toward zero
    //     double p = Math.min(Math.max(Math.abs(speed), 0.0), 1.0); // positive magnitude
    //     turret.set(direction * p);
    // }

    public double getTurretPosition() {
        // return cached, frequently-updated value
        return turretPosition;
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

    public void setflyWheel33() {
        flyWheel1.setControl(velocityRequest3);
        flyWheel2.setControl(velocityRequest3);
    }

     public void setFeeder(double speed) {
        feeder.set(speed);
        counterRoller.set(-speed);

    }
    
     public void setIndexer(double speed) {
        indexer.set(speed);
    }

    public void setPivot(double speed) {
        intakePivot.set(speed);
    }

    public void setLift(double speed) {
        lift.set(speed);
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

    public void stopLift() {
        lift.set(0);
    }

    public void setFlywheelZero(double speed) {
        flyWheel1.set(Math.abs(speed));
        flyWheel2.set(Math.abs(speed));
    }
    
    public void shootingMovements(double time) {
        // example: reverse feeder and counter-roller to clear jams
        feeder.set(0.6); //keeps intaking either way, but reverses the counter-roller to clear jams
        counterRoller.set(1);
        new WaitCommand(time);
        feeder.set(0);
        counterRoller.set(0);
    }

}