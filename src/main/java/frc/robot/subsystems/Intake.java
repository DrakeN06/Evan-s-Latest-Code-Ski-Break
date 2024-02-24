package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

public class Intake extends SubsystemBase {
    private CANSparkMax intakeMotor;

    public Intake() {
        intakeMotor = new CANSparkMax(Constants.Intake.intakeMotorID, MotorType.kBrushless);
        intakeMotor.setInverted(Constants.Intake.IntakeMotorInverted);
    }

    public void runIntake(double speed){
        intakeMotor.set(speed);
    }
    
    public void coastIntake(){
        intakeMotor.setIdleMode(IdleMode.kCoast);
    }

    public void brakeIntake(){
        intakeMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        double intakeSpeed = intakeMotor.get();
        SmartDashboard.putNumber("Intake Speed: ", intakeSpeed);
    }
}
