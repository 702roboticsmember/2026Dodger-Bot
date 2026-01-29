// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;



public class TurretSubsystem extends SubsystemBase {
  private TalonFX Motor = new TalonFX(Constants.TurretConstants.TurretMotorID);
  private Rotation2d turretOffsetField = new Rotation2d();
  private Rotation2d turretPastValue = new Rotation2d(Constants.TurretConstants.initialAngle);
  //private SparkMax Motor = new SparkMax(0, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushed);
 // private Encoder m_encoderFR = new Encoder(0, 1);
  
  
  // private Spark LeftMotor = new Spark(Constants.CoralIntakeConstants.LeftMotorID);
  // private Spark RightMotor = new Spark(Constants.CoralIntakeConstants.RightMotorID);
 // private DigitalInput sensor = new DigitalInput(Constants.LIMIT_SWITCH_INTAKE);
  /** Creates a new ClimbSubsystem. */
   public TurretSubsystem() { 
    //m_encoderFR.setSimDevice(SimDevice.create("encoder"));
      
    

      Motor.getConfigurator().apply(Robot.CTRE_CONFIGS.turretConfig);
      setAngle(Constants.TurretConstants.initialAngle);
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


  
  public double getLimelightYaw(){
    double limelightMeasurement = LimelightHelpers.getIMUData("limelight").robotYaw;
    
    return limelightMeasurement;
  }
}
