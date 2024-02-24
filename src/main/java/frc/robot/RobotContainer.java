package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(Constants.Drive.driveController);
    private final Joystick weapons = new Joystick(Constants.Drive.weaponController);

    /* Drive Controls */
    private final int d_translationAxis = XboxController.Axis.kLeftY.value;
    private final int d_strafeAxis = XboxController.Axis.kLeftX.value;
    private final int d_rotationAxis = XboxController.Axis.kRightX.value;

    /* Weapon Controls */
    private final int w_rotationAxis = XboxController.Axis.kLeftY.value;
    private final int w_loadIntake = XboxController.Axis.kLeftTrigger.value;
    private final int w_emptyIntake = XboxController.Axis.kRightTrigger.value;
    private final int w_shooterAxis = XboxController.Axis.kRightY.value; 

    /* Driver Buttons */
    private final JoystickButton d_zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton d_robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton d_slowMode = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    /* Weapons Buttons */
    private final JoystickButton w_resetArm = new JoystickButton(weapons, XboxController.Button.kY.value);
    private final JoystickButton w_trapAutoArm = new JoystickButton(weapons, XboxController.Button.kA.value);
    private final JoystickButton w_ampAutoArm = new JoystickButton(weapons, XboxController.Button.kB.value);
    private final JoystickButton w_speakerAutoArm = new JoystickButton(weapons, XboxController.Button.kX.value);
    private final JoystickButton w_slowMode = new JoystickButton(weapons, XboxController.Button.kRightBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Arms a_Arms = new Arms();
    private final Intake i_Intake = new Intake();
    private final Shooter s_Shooter = new Shooter();
    private final Vision m_Vision = new Vision();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(d_translationAxis), 
                () -> -driver.getRawAxis(d_strafeAxis), 
                () -> -driver.getRawAxis(d_rotationAxis), 
                () -> d_robotCentric.getAsBoolean(),
                () -> d_slowMode.getAsBoolean()
            )
        );

        a_Arms.setDefaultCommand(
            new TeleopArm(
                a_Arms,
                () -> -weapons.getRawAxis(w_rotationAxis),
                () -> w_slowMode.getAsBoolean()
            )
        );

        i_Intake.setDefaultCommand(
            new TeleopIntake(
                i_Intake,
                () -> -weapons.getRawAxis(w_loadIntake),
                () -> -weapons.getRawAxis(w_emptyIntake)
            )
        );

        s_Shooter.setDefaultCommand(
            new TeleopShooter(
                s_Shooter,
                () -> -weapons.getRawAxis(w_shooterAxis))
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        d_zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        /* Weapon Buttons */
        /*w_resetArm.onTrue(new InstantCommand(() -> a_Arms.autoSetLeftArmPosition(Constants.Arms.calculatedArmThetaAtDefault)));
        w_resetArm.onTrue(new InstantCommand(() -> a_Arms.autoSetRightArmPosition(Constants.Arms.calculatedArmThetaAtDefault)));
        w_trapAutoArm.onTrue(new InstantCommand(() -> a_Arms.autoSetLeftArmPosition(Constants.Arms.calculatedArmThetaAtTrap)));
        w_trapAutoArm.onTrue(new InstantCommand(() -> a_Arms.autoSetRightArmPosition(Constants.Arms.calculatedArmThetaAtTrap)));
        w_ampAutoArm.onTrue(new InstantCommand(() -> a_Arms.autoSetLeftArmPosition(Constants.Arms.calculatedArmThetaAtAmp)));
        w_ampAutoArm.onTrue(new InstantCommand(() -> a_Arms.autoSetRightArmPosition(Constants.Arms.calculatedArmThetaAtAmp)));
        w_speakerAutoArm.onTrue(new InstantCommand(() -> a_Arms.autoSetLeftArmPosition(Constants.Arms.calculatedArmThetaAtSpeaker)));
        w_speakerAutoArm.onTrue(new InstantCommand(() -> a_Arms.autoSetRightArmPosition(Constants.Arms.calculatedArmThetaAtSpeaker)));*/

        /* Weapons with Motion Magic */
        w_resetArm.onTrue(new InstantCommand(() -> a_Arms.SetMotionMagicSpeeds(Constants.Arms.calculatedArmThetaAtDefault)));
        w_trapAutoArm.onTrue(new InstantCommand(() -> a_Arms.SetMotionMagicSpeeds(Constants.Arms.calculatedArmThetaAtTrap)));
        w_ampAutoArm.onTrue(new InstantCommand(() -> a_Arms.SetMotionMagicSpeeds(Constants.Arms.calculatedArmThetaAtAmp)));
        w_speakerAutoArm.onTrue(new InstantCommand(() -> a_Arms.autoSetLeftArmPosition(Constants.Arms.calculatedArmThetaAtSpeaker)));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // The "BASIC Autonomous Path" will run
        return new PathPlannerAuto("BASIC Autonomous Path");
    }
}