package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.TankDrive;

public class DriveTrainCommand extends Command {
    private final TankDrive tankDrive;
    private final CommandXboxController driveController;

    public DriveTrainCommand(TankDrive tankDrive, CommandXboxController driveController)  {
        this.tankDrive = tankDrive;
        this.driveController = driveController;
        addRequirements(tankDrive);
    }

    @Override
    public void execute() {
        tankDrive.tankDrive(-driveController.getLeftY(), -driveController.getRightY());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
