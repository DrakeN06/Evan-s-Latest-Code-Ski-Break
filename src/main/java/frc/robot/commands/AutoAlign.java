package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Vision;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlign extends Command {
    private Vision m_vision;
    private BooleanSupplier autoAlign;

    public AutoAlign(Vision m_vision, BooleanSupplier autoAlign) {
        this.m_vision = m_vision;
        addRequirements(m_vision);

        this.autoAlign = autoAlign;
    }

    @Override
    public void execute() {
        double KP = Constants.Vision.visionKP;
        double min_Movement = Constants.Vision.visionMin_Movement;
        boolean m_autoAlign = autoAlign.getAsBoolean();

        // std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
        double tx = m_vision.getTX();

        if (m_autoAlign) { // "joystick->GetRawButton(9)"
            double heading_error = -tx;
            double steering_adjust = 0.0;

            if (Math.abs(heading_error) > 1.0) {

                if (heading_error < 0) {
                    steering_adjust = KP * heading_error + min_Movement;
                } else {
                    steering_adjust = KP * heading_error - min_Movement;
                }
            } 
            
            // left_command += steering_adjust;
            // right_command -= steering_adjust;
        }
    }
}
