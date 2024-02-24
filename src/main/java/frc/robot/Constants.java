package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public final class Constants {


    public static final class Drive {
        /* Driver Constants */
        public static final double driveStickDeadband = 0.1;
        public static final double armStickDeadband = 0.1;
        public static final double basePercentDriveOutput = 0.5; // The percent motor output for the swerve modules
        public static final double percentBasePercentDriveOutput = 0.2; // The percent amount of basePercentDriveOutput; used in d_slowMode
        public static final double maxBasePercentArmOutput = 0.4; // The percent motor output for the arm motors
        public static final double percentMaxBasePercentArmOutput = 0.5; // The percent amount of maxBasePercentArmOutput; used in w_slowMode
        public static final int slowModeButtonBinding = 2; // The button binding for slowMode; 1 = A, 2 = B, 3 = X, 4 = Y (Do NOT use ID = 4!)

        /* Controller Constants */
        public static final int driveController = 0;
        public static final int weaponController = 1;
    }

    public static final class Display { // By default, these should be set to true
        /* Swerve Display */
        public static final boolean showSwerveData = true;

        /* Arm Display */
        public static final boolean showArmTheta = true;

        /* Vision Display */
        public static final boolean showHorizontalVisionError = true;
        public static final boolean showVerticalVisionError = true;
        public static final boolean showDistanceVisionError = true;
    }

    public static final class Arms {
        /* Arm Motor & Gearbox Constants */
        public static final int leftArmMotorID = 9;
        public static final int rightArmMotorID = 10;
        public static final boolean leftArmMotorInverted = false;
        public static final boolean rightArmMotorInverted = true;
        public static final double armMotorGearRatio = 100; // Essentially, the AMOUNT of motor rotations without a gearbox needed to visibly rotate the shaft ONCE
        public static final double armsMaxVoltage = 12;

        /* Arm Bounds & Tolerance Constants */
        public static final double armsMinimumRotation = 0 + 1; // Needs to be in degrees of VISIBLE rotation
        public static final double armsMaximumRotation = 115 - 1; // Needs to be in degrees of VISIBLE rotation
        public static final double armsMaxAngleDiscrepancy = 1; // Needs to be in degrees of VISIBLE rotation

        public static final double armLowerBoundTheta = armsMinimumRotation * armMotorGearRatio;
        public static final double armUpperBoundTheta = armsMaximumRotation * armMotorGearRatio;
        public static final double armsMaxErrorTolerance = armsMaxAngleDiscrepancy * armMotorGearRatio;

        /* Arm Offset Constants */
        public static final double leftArmThetaOffset = 0; // BUGGED: Do NOT change unless you know what you are doing!
        public static final double rightArmThetaOffset = 0; // BUGGED: Do NOT change unless you know what you are doing!

        public static final double calculatedLeftArmThetaOffset = leftArmThetaOffset * armMotorGearRatio;
        public static final double calculatedRightArmThetaOffset = rightArmThetaOffset * armMotorGearRatio;

        /* Automatic Arm  Constants */
        public static final double armKS = 0.25;
        public static final double armKV = 0.12;
        public static final double armKA = 0.01;
        public static final double armKP = 0.025;
        public static final double armKI = 0.005;
        public static final double armKD = 0.0025;
        public static final double percentAutomaticArmOutput = 0.1;
        public static final double maxPIDArmThetaOffset = 0.25;
        public static final double maxPIDArmIntegrationZone = 5;
        public static final double armThetaAtDefault = 3.5;
        public static final double armThetaAtTrap = 45; // TODO: Needs to be changed!
        public static final double armThetaAtAmp = 97;
        public static final double armThetaAtSpeaker = 29; // TODO: Needs to be changed!

        public static final double calculatedMaxPIDArmThetaOffset = maxPIDArmThetaOffset * armMotorGearRatio;
        public static final double calculatedMaxPIDArmIntegrationZone = maxPIDArmIntegrationZone * armMotorGearRatio;
        public static final double calculatedArmThetaAtDefault = armThetaAtDefault * armMotorGearRatio;
        public static final double calculatedArmThetaAtTrap = armThetaAtTrap * armMotorGearRatio;
        public static final double calculatedArmThetaAtAmp = armThetaAtAmp * armMotorGearRatio;
        public static final double calculatedArmThetaAtSpeaker = armThetaAtSpeaker * armMotorGearRatio;

        /* Arm Maintenance Constants */
        public static final boolean armCalibrationMode = true;
        
        /* Motion Magic Profiling Constants*/
        public static final double motionMagicCruiseVelocity = 15;
        public static final double motionMagicAcceleration = 80;
        public static final double motionMagicMotionJerk = 800;

    }

    public static final class Intake {
        public static final int intakeMotorID = 0; //TODO: Needs to be changed!
        public static final boolean IntakeMotorInverted = false; //TODO: Needs to be changed!
    }

    public static final class Shooter {
        public static final int topShooterID = 1; //TODO: Needs to be changed!
        public static final int bottomShooterID = 2; //TODO: Needs to be changed!
        public static final boolean topShooterInverted = false; //TODO: Needs to be changed!
        public static final boolean bottomShooterInverted = true; //TODO: Needs to be changed!
    }

    public static final class Vision {
        /* Vision Constants */
        public static final double visionKP = -0.1;
        public static final double visionMin_Movement = 0.05;
    }

    public static final class Swerve {
        public static final int pigeonID = 1;

        public static final COTSTalonFXSwerveConstants chosenModule = 
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double robotSideLength = Units.inchesToMeters(28);
        public static final double trackWidth = Units.inchesToMeters(22.75); 
        public static final double wheelBase = Units.inchesToMeters(22.875);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* PathPlanner Constants */
        public static final double maxModuleSpeed = 4.5; // Max module speed, in m/s
        public static final double driveBaseRadius = (robotSideLength / 2) * Math.sqrt(2);
        public static final HolonomicPathFollowerConfig pathPlannerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            maxModuleSpeed, 
            driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ); 

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; // Originally 0.12
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; // Maybe check later
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 5.0; 
        /** Radians per Second */
        public static final double maxAngularVelocity = 2 * 2 * Math.PI; // Originally 10

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(254.355);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(213.75);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(129.902);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(348.047);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }

    public static final class AutoConstants { // Ignore for now
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}