/*
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase {
    private final TalonFX intake;
    private final PositionVoltage m_Request = new PositionVoltage(0);
    
    // ADJUST THIS: If your gearbox is 10:1, set this to 10.0. If no gearbox, use 1.0.
    private final double GEAR_RATIO = 1.0; 

    public Intake() {
        intake = new TalonFX(5);
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

        // PID Gains
        var slot0Configs=new Slot0Configs();
        intakeConfig.Slot0.kP = 12.0; 
        intakeConfig.Slot0.kI = 0.0; // Set to 0 for now to test safely
        intakeConfig.Slot0.kD = 0.1; 

        intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        intake.getConfigurator().apply(new TalonFXConfiguration().withSlot0(slot0Configs));
        
        // Zero the sensor on boot so "0 degrees" is where the robot starts
        intake.setPosition(0);
    }
    public double getPosition(){
        return intake.getPosition().getValueAsDouble();
    }

     public void goToDegree(double degrees) {
        // formula: (Degrees / 360) * Gear Ratio
        double targetRotations = (degrees / 360.0) * GEAR_RATIO;
        intake.setControl(positionRequest.withPosition(targetRotations));
    }
 public void goToPosition() {
        // formula: (Degrees / 360) * Gear Ratio
        double targetPosition=5.5;
        intake.setControl(m_Request.withPosition(targetPosition);
    public void stopIntake() {
        // Use duty cycle 0 to stop and let PID release
        intake.set(0); 
    }
    public void setIntake(double speed) {
        intake.set(0.2);
    }
    @Override
    public void periodic() {
        // Check this in Shuffleboard to see if the number changes when you move it by hand
        SmartDashboard.putNumber("Intake Deg", intake.getPosition().getValueAsDouble() * 360 / GEAR_RATIO);
    }
}
*/