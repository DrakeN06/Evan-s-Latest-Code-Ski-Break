// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

public class TeleopShooter extends Command {
  private Shooter s_Shooter;
  private DoubleSupplier speedSup;
  
  public TeleopShooter(Shooter s_Shooter, DoubleSupplier speedSup) {
    this.s_Shooter = s_Shooter;
    addRequirements(s_Shooter);
    
    this.speedSup = speedSup;
  }


  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double s_Speed = speedSup.getAsDouble();
    s_Shooter.runShooter(s_Speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Shooter.brakeShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
