// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private TalonFX IntakeMotor = new TalonFX(Constants.IntakeConstants.IntakeMotorID);
  public final double MaxIntakeSpeed = (Constants.IntakeConstants.MaxIntakeSpeed);
   
  
    public IntakeSubsystem() {
      IntakeMotor.getConfigurator().apply(Robot.CTRE_CONFIGS.intakeConfigs);
    }
    public void setSpeed(double speed) {
      IntakeMotor.set(speed);
    }
   

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
