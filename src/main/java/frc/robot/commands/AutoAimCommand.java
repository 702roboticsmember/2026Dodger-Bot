// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAimCommand extends Command {
  private String limelightName = Constants.limelightConstants.limelightName;
  private double txOffset;
 
  private TurretSubsystem t_TurretSubsystem ;
  private Swerve s_Swerve;
  //Pose2d Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName).pose;

  private double g = Constants.PhysicsConstants.gravity;
  private double h = Constants.PhysicsConstants.HubHeight;
  private double i = Constants.PhysicsConstants.BallIntialHeight;

  private Translation2d poi;

   private PIDController ArmPID = new PIDController(
      0.008,
      0.0,
      0.0001);


 // private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(null, null, null, null);
  

  
  /** Creates a new AutoAimCommand. */
  public AutoAimCommand(Translation2d poi, TurretSubsystem t_TurretSubsystem) {
    this.t_TurretSubsystem = t_TurretSubsystem;
    addRequirements(t_TurretSubsystem);
    this.poi = poi;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArmPID.setTolerance(0.01);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     Field2d fieldActive = new Field2d();
     
    Pose2d pose2 = Constants.TurretConstants.turretPose2d;
    double degrees = getTurretAngleToHub(pose2).getDegrees();
    if (LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName).pose != null){
    Pose2d pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName).pose;
    //Pose2d angle = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName).angle;
    SmartDashboard.putNumber("getDistance", getDistance(pose));
    SmartDashboard.putNumber("getAngle", getAngleToHub(pose).getDegrees());
    SmartDashboard.putNumber("getturretAngletohub", getTurretAngleToHub(pose).getDegrees());

    }
    SmartDashboard.putNumber("getDistanceS", getDistance(pose2));
    SmartDashboard.putNumber("getAngleS", getAngleToHub(pose2).getDegrees());
    SmartDashboard.putNumber("getturretAngletohubS", degrees);
     SmartDashboard.putNumber("getturretAngle", getTurretAngle(pose2).getDegrees());
      fieldActive.setRobotPose(pose2);
      SmartDashboard.putData("Fieldtest", fieldActive);
      ArmPID.setSetpoint(0);
       double value = ArmPID.calculate(degrees);
       if(value > 0){
        value += 0.017;
       }else if(value < 0){
        value += -0.017;
       }
       SmartDashboard.putNumber("hubpid", value);
    t_TurretSubsystem.setSpeed(MathUtil.clamp(value, -0.6, 0.6));
      

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double getDistance(Pose2d Pose) {
    return Pose.getTranslation().getDistance(poi);
  }

  private Rotation2d getAngleToHub(Pose2d Pose){
    return poi.minus(Pose.getTranslation()).getAngle();
  }

  public Rotation2d getTurretAngleToHub(Pose2d pose){
    return getAngleToHub(pose).minus(pose.getRotation());
  }


  public double CalculateOffset(double Vrz, double Dx, double t){
    return Math.tanh((Vrz* t)/Dx);
  }

  public double CalculateVy(double Dx){
    return 0.322959*Dx + 19.09439;
  }

  public double timeTillTarget(double Vy){
    double a = -g * 0.5;
    double b = Vy;
    double c = -(h-i);
    return (-b) - Math.sqrt((b * b) - (4* a * c));

  }


  public double CalculateVx(double distance, double Vy){
    double a = -g * distance * distance * 0.5;
    double b = Vy*distance;
    double c = -(h- i);
    return (2*(a)) / ((-b) - Math.sqrt((b * b) - 4*(a * c)));
  }

  public double CalculateVs(double Vx, double Vy, double Vrx){
    return Math.sqrt(((Vx - Vrx) * (Vx - Vrx)) + (Vy * Vy));
  }

  public double CalculateShootAngle(double Vx, double Vy, double Vrx){
    return Math.tanh(Vy/(Vx - Vrx));
  }

  public double getVrz(Pose2d pose, double Vz){
    return getTurretAngleToHub(pose).getCos() * Vz;
  }

  public Rotation2d getTurretAngle(Pose2d pose){
    return pose.getRotation();
  }

  public double getIMUYaw(){
    return LimelightHelpers.getIMUData("limelight").gyroY;
  }

 
}
