// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;



public class TurretSubsystem extends SubsystemBase {
  private TalonFX Motor = new TalonFX(Constants.TurretConstants.TurretMotorID);
  private Rotation2d turretOffsetField = new Rotation2d();
  private Rotation2d turretPastValue = new Rotation2d(Constants.TurretConstants.initialAngle);
  //private SparkMax Motor = new SparkMax(0, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushed);
   

  
  
  // Note: These gains are fake, and will have to be tuned for your robot.
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0.25, .15, 0.01);
  private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  private final TrapezoidProfile m_profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(.175, .075));
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

            var s_slot0Configs = turretConfig.Slot0;
              s_slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
              s_slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
              s_slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
              s_slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
              s_slot0Configs.kI = 0; // no output for integrated error
              s_slot0Configs.kD = 0; // no output for error derivative
    
    

      Motor.getConfigurator().apply(turretConfig);
    
  // /** Creates a new ReleaseSubsystem. */
 }

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("turretangle", getAngleAsDouble());
    
    SmartDashboard.putNumber("limelightrobotyaw", getLimelightYaw());
   
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    Motor.set(speed);
    SmartDashboard.putBoolean("hiiiiiii", true);
    
    
  }

  public Command run(DoubleSupplier input){
    
    return this.runEnd(() -> this.setSpeed(input.getAsDouble()), () -> this.setSpeed(0.0));
  }

  public Rotation2d getAngle() {
    return new Rotation2d(getAngleAsDouble());
    
    
  }

  public double getAngleAsDouble() {
    return Motor.getPosition().getValueAsDouble();
  }

 

  public void setAngle(double degrees){
    Motor.setPosition(degrees);
  }

  public double getVel(){
    return Motor.getVelocity().getValueAsDouble();
  }
  public void setVoltage(double Voltage){
    Motor.setVoltage(Voltage);
  }

  
  public double getLimelightYaw(){
    double limelightMeasurement = LimelightHelpers.getIMUData("limelight").robotYaw;
    
    return limelightMeasurement;
  }

  public void setpoint(TrapezoidProfile.State m_setpoint){
     
     

    double toOutputVelocity = m_setpoint.velocity;
    SmartDashboard.putNumber("sovel", toOutputVelocity);

            // toOutputVelocity = closedLoopLimiter.calculate(toOutputVelocity);

            driveVelocity.Velocity = toOutputVelocity/32 ;
            driveVelocity.FeedForward = m_feedforward.calculate(m_setpoint.velocity)/12;

          Motor.setControl(driveVelocity);
  }
}
