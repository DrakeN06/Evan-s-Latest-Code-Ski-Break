// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private CANSparkMax topShooter;
  private CANSparkMax bottomShooter;

  public Shooter() {
    topShooter = new CANSparkMax(Constants.Shooter.topShooterID, MotorType.kBrushless);
    bottomShooter = new CANSparkMax(Constants.Shooter.bottomShooterID, MotorType.kBrushless);

    topShooter.setInverted(Constants.Shooter.topShooterInverted);
    bottomShooter.setInverted(Constants.Shooter.bottomShooterInverted);
  }

  public void runShooter(double speed) {
    topShooter.set(speed);
    bottomShooter.set(speed);
  }

  public void coastShooter() {
    topShooter.setIdleMode(IdleMode.kCoast);
    bottomShooter.setIdleMode(IdleMode.kCoast);
  }

  public void brakeShooter() {
    topShooter.setIdleMode(IdleMode.kBrake);
    bottomShooter.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    
  }
}
