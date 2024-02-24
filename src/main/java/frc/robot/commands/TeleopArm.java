package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Arms;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj2.command.Command;

public class TeleopArm extends Command {
    private Arms a_Arms;
    private DoubleSupplier speedSup;
    private BooleanSupplier slowModeSup;

    public TeleopArm(Arms a_Arms, DoubleSupplier speedSup, BooleanSupplier slowModeSup) {
        this.a_Arms = a_Arms;
        addRequirements(a_Arms);

        this.speedSup = speedSup;
        this.slowModeSup = slowModeSup;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double a_speed = MathUtil.applyDeadband(speedSup.getAsDouble(), Constants.Drive.armStickDeadband);
        boolean a_slowMode = slowModeSup.getAsBoolean();

        if (Constants.Arms.armCalibrationMode) { // For calibrating the arm motors
            /* Left Arm Motor */
            if (a_Arms.getLeftArmPosition() <= Constants.Arms.armUpperBoundTheta && a_Arms.getLeftArmPosition() >= Constants.Arms.armLowerBoundTheta) {
                if (a_slowMode) {
                    a_Arms.setLeftArmMotorSpeed(a_speed * Constants.Drive.percentMaxBasePercentArmOutput);
                } else {
                    a_Arms.setLeftArmMotorSpeed(a_speed);
                }
            } else if (a_Arms.getLeftArmPosition() > Constants.Arms.armUpperBoundTheta) {
                if (a_speed < 0) {
                    if (a_slowMode) {
                        a_Arms.setLeftArmMotorSpeed(a_speed * Constants.Drive.percentMaxBasePercentArmOutput);
                    } else {
                        a_Arms.setLeftArmMotorSpeed(a_speed);
                    }
                } else {
                    a_Arms.brakeLeftArmMotor();
                }
            } else {
                if (a_speed > 0) {
                    if (a_slowMode) {
                        a_Arms.setLeftArmMotorSpeed(a_speed * Constants.Drive.percentMaxBasePercentArmOutput);
                    } else {
                        a_Arms.setLeftArmMotorSpeed(a_speed);
                    }
                } else {
                    a_Arms.brakeLeftArmMotor();
                }
            }

            /* Right Arm Motor */
            if (a_Arms.getRightArmPosition() <= Constants.Arms.armUpperBoundTheta && a_Arms.getRightArmPosition() >= Constants.Arms.armLowerBoundTheta) {
                if (a_slowMode) {
                    a_Arms.setRightArmMotorSpeed(a_speed * Constants.Drive.percentMaxBasePercentArmOutput);
                } else {
                    a_Arms.setRightArmMotorSpeed(a_speed);
                }
            } else if (a_Arms.getRightArmPosition() > Constants.Arms.armUpperBoundTheta) {
                if (a_speed < 0) {
                    if (a_slowMode) {
                        a_Arms.setRightArmMotorSpeed(a_speed * Constants.Drive.percentMaxBasePercentArmOutput);
                    } else {
                        a_Arms.setRightArmMotorSpeed(a_speed);
                    }
                } else {
                    a_Arms.brakeRightArmMotor();
                }
            } else {
                if (a_speed > 0) {
                    if (a_slowMode) {
                        a_Arms.setRightArmMotorSpeed(a_speed * Constants.Drive.percentMaxBasePercentArmOutput);
                    } else {
                        a_Arms.setRightArmMotorSpeed(a_speed);
                    }
                } else {
                    a_Arms.brakeRightArmMotor();
                }
            }
        } else { // Both Left & Right Motors
            if (Math.abs(a_Arms.getLeftArmPosition() - a_Arms.getRightArmPosition()) <= Constants.Arms.armsMaxErrorTolerance) { // Checks if the motors are synchronized
                if ((a_Arms.getLeftArmPosition() <= Constants.Arms.armUpperBoundTheta && a_Arms.getLeftArmPosition() >= Constants.Arms.armLowerBoundTheta) && (a_Arms.getRightArmPosition() <= Constants.Arms.armUpperBoundTheta && a_Arms.getRightArmPosition() >= Constants.Arms.armLowerBoundTheta)) {
                    if (a_slowMode) {
                        a_Arms.setArmMotorSpeeds(a_speed * Constants.Drive.percentMaxBasePercentArmOutput);
                    } else {
                        a_Arms.setArmMotorSpeeds(a_speed);
                    }
                } else if ((a_Arms.getLeftArmPosition() > Constants.Arms.armUpperBoundTheta) && (a_Arms.getRightArmPosition() > Constants.Arms.armUpperBoundTheta)) {
                    if (a_speed < 0) {
                        if (a_slowMode) {
                            a_Arms.setArmMotorSpeeds(a_speed * Constants.Drive.percentMaxBasePercentArmOutput);
                        } else {
                            a_Arms.setArmMotorSpeeds(a_speed);
                        }
                    } else {
                        a_Arms.brakeArmMotors();
                    }
                } else if ((a_Arms.getLeftArmPosition() < Constants.Arms.armLowerBoundTheta) && (a_Arms.getRightArmPosition() < Constants.Arms.armLowerBoundTheta)) {
                    if (a_speed > 0) {
                        if (a_slowMode) {
                            a_Arms.setArmMotorSpeeds(a_speed * Constants.Drive.percentMaxBasePercentArmOutput);
                        } else {
                            a_Arms.setArmMotorSpeeds(a_speed);
                        }
                    } else {
                        a_Arms.brakeArmMotors();
                    }
                } else {
                    a_Arms.brakeArmMotors();
                    System.out.println("ERROR: Potential arm code error! Check your if-else statements!");
                }
            } else {
                a_Arms.brakeArmMotors();
                System.out.println("WARNING: Arms need calibration!");
            }
            
        }

        /* Out-of-Bounds Error Adjustment */
        // a_Arms.correctArmMotorPositions(); // Possible issue involving conflicts with the PID instant command
        
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
