// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;


public class TurretPIDCommand extends Command {

  private PIDController ArmPID = new PIDController(
      Constants.TurretConstants.kP,
      Constants.TurretConstants.kI,
      Constants.TurretConstants.kD);

  private TurretSubsystem t_TurretSubsystem;

  /** Creates a new ArmPIDCommand. */
  public TurretPIDCommand(TurretSubsystem t_TurretSubsystem, double setpoint) {
    this.t_TurretSubsystem = t_TurretSubsystem;
    ArmPID.setSetpoint(setpoint);
    ArmPID.setTolerance(Constants.TurretConstants.PIDTolerance);
    addRequirements(t_TurretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double value = ArmPID.calculate(t_TurretSubsystem.getAngle());
    t_TurretSubsystem.setSpeed(value);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    t_TurretSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ArmPID.atSetpoint();
  }
}
