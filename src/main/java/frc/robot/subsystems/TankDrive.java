package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDrive extends SubsystemBase {
    private SparkMax frontLeft, frontRight, backLeft, backRight;
    private SparkMaxConfig frontLeftConfig, frontRightConfig, backLeftConfig, backRightConfig;

    private RelativeEncoder leftEncoder, rightEncoder;
    private DifferentialDrive diffDrive;

    public TankDrive() {
        frontLeftConfig = new SparkMaxConfig();
        frontRightConfig = new SparkMaxConfig();
        backLeftConfig = new SparkMaxConfig();
        backRightConfig = new SparkMaxConfig();

        //frontLeftConfig
        //    .inverted(false);
        //backLeftConfig
            //.follow(3);
        backRightConfig
            .inverted(true)
            .follow(17);

        frontRightConfig
            .inverted(true);

        //frontLeft = new SparkMax(3, SparkMax.MotorType.kBrushless);
        frontRight = new SparkMax(17, SparkMax.MotorType.kBrushless);
        backLeft = new SparkMax(1, SparkMax.MotorType.kBrushless);
        backRight = new SparkMax(4, SparkMax.MotorType.kBrushless);

        //frontLeft.configure(frontLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // backLeft.configure(backLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        frontRight.configure(frontRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        backRight.configure(backRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftEncoder = backLeft.getEncoder();
        rightEncoder = frontRight.getEncoder();

        diffDrive = new DifferentialDrive(backLeft, frontRight);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        diffDrive.tankDrive(leftSpeed, rightSpeed);
    }

    // public Command tankDriveJoysticks(double leftJoystick, double rightJoystick) {
    //     return Commands.run(() -> diffDrive.tankDrive(leftJoystick, rightJoystick), this);
    // }
    
    public void stop() {
        diffDrive.stopMotor();
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public double getLeftEncoderPosition() {
        return leftEncoder.getPosition();
    }

    public double getRightEncoderPosition() {
        return rightEncoder.getPosition();
    }
}