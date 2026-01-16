// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAimCommand extends Command {
  private String limelightName = Constants.limelightConstants.limelightName;
  private double txOffset;
  private DoubleSupplier tx = () -> LimelightHelpers.getTX(limelightName);
  private DoubleSupplier ty = () -> LimelightHelpers.getTY(limelightName);
  private DoubleSupplier ta = () -> LimelightHelpers.getTA(limelightName);
  private BooleanSupplier tv = () -> LimelightHelpers.getTV(limelightName);

  private double g = 32;
  private double h = 0;
  private double i = Constants.PhysicsConstants.BallIntialHeight;
  

  private double getDistance() {
    double[] Pose = LimelightHelpers.getTargetPose_RobotSpace(limelightName);
    Translation2d translation2d = new Translation2d(Pose[0], Pose[1]);
    return translation2d.getDistance(new Translation2d());  
  }
  /** Creates a new AutoAimCommand. */
  public AutoAimCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double CalculateOffset(double Vrz, double Dx, double t){
    return Math.tanh((Vrz* t)/Dx);
  }

  public double CalculateVy(double Dx){
    return 0.322959*Dx + 19.09439;
  }

  public double CalculateVx(double distance, double Vy){
    double a = -g * distance * distance * 0.5;
    double b = Vy*distance;
    double c = -(h- i);
    return (2*(a)) / ((-b) - Math.sqrt((b * b) - 4*(a * c)));
  }

  public double CalculateVs(){
    return 0;
  }
}
