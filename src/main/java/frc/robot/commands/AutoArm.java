package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Arms;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoArm extends Command { // While not currently used, this might be useful for autonomous code
    private Arms a_Arms;
    private PIDController a_PIDController;

    public AutoArm(Arms a_Arms, double setPoint) {
        this.a_Arms = a_Arms;
        addRequirements(a_Arms);

        this.a_PIDController = new PIDController(Constants.Arms.armKP, Constants.Arms.armKI, Constants.Arms.armKD);
        a_PIDController.setSetpoint(setPoint);
    }

    @Override
    public void initialize() {
        if (Math.abs(a_Arms.getLeftArmPosition() - a_Arms.getRightArmPosition()) <= Constants.Arms.armsMaxErrorTolerance) { // Checks if the motors are synchronized
            a_PIDController.reset();
        } else {
            System.out.println("WARNING: Arms need calibration!");
        }
    }

    @Override
    public void execute() {
        double newLeftArmSpeed = a_PIDController.calculate(a_Arms.getAverageArmPosition());
        a_Arms.setLeftArmMotorSpeed(newLeftArmSpeed * Constants.Arms.percentAutomaticArmOutput);
        double newRightArmSpeed = a_PIDController.calculate(a_Arms.getAverageArmPosition());
        a_Arms.setRightArmMotorSpeed(newRightArmSpeed * Constants.Arms.percentAutomaticArmOutput);
    }

    @Override
    public void end(boolean interrupted) {
        a_Arms.brakeArmMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
