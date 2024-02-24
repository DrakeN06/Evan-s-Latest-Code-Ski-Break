// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

public class TeleopIntake extends Command {
  private Intake i_Intake;
  private DoubleSupplier speedInSup;
  private DoubleSupplier speedOutSup;

  public TeleopIntake(Intake i_Intake, DoubleSupplier speedInSup, DoubleSupplier speedOutSup) {
    this.i_Intake = i_Intake;
    addRequirements(i_Intake);
    
    this.speedInSup = speedInSup;
    this.speedOutSup = speedOutSup;
  }

  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double i_speedIn = speedInSup.getAsDouble();
    double i_speedOut = speedOutSup.getAsDouble();
    i_Intake.runIntake(i_speedIn - i_speedOut);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    i_Intake.brakeIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
