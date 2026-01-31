// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;



public class TurretSubsystem extends SubsystemBase {
  private TalonFX Motor = new TalonFX(Constants.TurretConstants.TurretMotorID);
  //private SparkMax Motor = new SparkMax(0, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushed);
   private static double kDt = 0.02;

  
  
  // Note: These gains are fake, and will have to be tuned for your robot.
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 1.5);

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  private final TrapezoidProfile m_profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75, 0.75));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  
  
  // private Spark LeftMotor = new Spark(Constants.CoralIntakeConstants.LeftMotorID);
  // private Spark RightMotor = new Spark(Constants.CoralIntakeConstants.RightMotorID);
 // private DigitalInput sensor = new DigitalInput(Constants.LIMIT_SWITCH_INTAKE);
  /** Creates a new ClimbSubsystem. */
   public TurretSubsystem() { 
    //m_encoderFR.setSimDevice(SimDevice.create("encoder"));
    TalonFXConfiguration turretConfig = new TalonFXConfiguration();
      turretConfig.CurrentLimits.StatorCurrentLimit = Constants.TurretConstants.CURRENT_LIMIT;
            turretConfig.CurrentLimits.SupplyCurrentLimit = Constants.TurretConstants.CURRENT_LIMIT;
            turretConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.TurretConstants.ENABLE_CURRENT_LIMIT;
            turretConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.TurretConstants.ENABLE_CURRENT_LIMIT;
            //turretConfig.
            

            turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = Constants.TurretConstants.LimitEnable;
            turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = -Constants.TurretConstants.forwardSoftLimit;
            turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = Constants.TurretConstants.LimitEnable;
            turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -Constants.TurretConstants.reverseSoftLimit;

            
    
    

      Motor.getConfigurator().apply(turretConfig);
    
  // /** Creates a new ReleaseSubsystem. */
 }

  @Override
  public void periodic() {
    Motor.setPosition(getAngle());
    SmartDashboard.putNumber("turretangle", getAngle());
   
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    Motor.set(speed);
    SmartDashboard.putBoolean("hiiiiiii", true);
    
    
  }

  public Command run(DoubleSupplier input){
    
    return this.runEnd(() -> this.setSpeed(MathUtil.applyDeadband(input.getAsDouble(), 0.1)), () -> this.setSpeed(0.0));
  }

  public double getAngle() {
    //m_encoderFR.isConnected();
   
    
    return -(m_encoderFR.getDistance()/28000) * 360;
    
    
  }
  
}
