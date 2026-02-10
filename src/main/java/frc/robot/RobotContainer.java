package frc.robot;



import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final XboxController driver = new XboxController(0);
    private final XboxController codriver = new XboxController(1);
    //private final JoystickButton Shoot = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton Intake = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton smartAim = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton homeGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton fastMode = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton slowMode = new JoystickButton(driver, XboxController.Button.kA.value);
    //private final JoystickButton smartShoot = new JoystickButton(codriver, XboxController.Button.kA.value);
    private final JoystickButton Shoot = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
        //private final JoystickButton Flywheel = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    public static double power = 1;
    public static boolean robotCentric = false;
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Command> teamChooser;
    

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final ShooterSubsystem s_ShooterSubsystem = new ShooterSubsystem();
    private final IntakeSubsystem i_IntakeSubsystem = new IntakeSubsystem();
     private final TurretSubsystem t_TurretSubsystem = new TurretSubsystem();
     private final HoodSubsystem h_HoodSubsystem = new HoodSubsystem();
    //private final ReleaseSubsystem r_ReleaseSubsystem = new ReleaseSubsystem();

    public Command shoot(double velocity, ShooterSubsystem s_ShooterSubsystem){
        
        return new SequentialCommandGroup(
            new InstantCommand(()->s_ShooterSubsystem.setVelocity(-velocity)),
            new WaitCommand(1),
            new InstantCommand(()->s_ShooterSubsystem.setVelocity(0.0))
        );
    }

    

    
    


    public RobotContainer() {
        Field2d field = new Field2d();
        Field2d fieldActive = new Field2d();
        SmartDashboard.putData("Field", field);
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            field.setRobotPose(pose);
        });
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            field.getObject("target pose").setPose(pose);
        });
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("path").setPoses(poses);
        });
        //fieldActive.setRobotPose(s_Swerve.getPose());
         //SmartDashboard.putData("FieldActive", fieldActive);

        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, 
        ()-> -driver.getRawAxis(1) * power, 
        ()-> -driver.getRawAxis(0) * power,
        ()-> -driver.getRawAxis(4) * power, 
        ()->robotCentric));



        t_TurretSubsystem.setDefaultCommand(t_TurretSubsystem.run(()-> codriver.getRawAxis(0) * 0.3));
        //r_ReleaseSubsystem.setDefaultCommand(r_ReleaseSubsystem.run(()-> codriver.getRawAxis(3) * 0.3));
        
        s_ShooterSubsystem.setDefaultCommand(s_ShooterSubsystem.spin(()-> codriver.getRawAxis(2) * 0.45));
        h_HoodSubsystem.setDefaultCommand(h_HoodSubsystem.spin(()->codriver.getRawAxis(5) * 0.05));

        configureButtonBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        teamChooser = new SendableChooser<>();
        
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("Team Chooser", teamChooser);
        SmartDashboard.putNumber("Input Distance", 0);
    }

    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new ParallelCommandGroup(new InstantCommand(() -> s_Swerve.zeroHeading()), new InstantCommand(()->s_Swerve.gyro.reset())));
        homeGyro.onTrue(new InstantCommand(()->{s_Swerve.setposetoField();}));
        slowMode.onTrue(new InstantCommand(() -> RobotContainer.power = .333));
        fastMode.onTrue(new InstantCommand(() -> RobotContainer.power = 1));  
        // Shoot.whileTrue(new ParallelCommandGroup(
        //     Commands.runEnd(()->s_ShooterSubsystem.setVelocity(60), ()-> s_ShooterSubsystem.setVelocity(0), s_ShooterSubsystem),
        //     Commands.runEnd(()->h_HoodSubsystem.goToAngle(50), ()-> h_HoodSubsystem.goToAngle(76), h_HoodSubsystem)));
        //Shoot.whileTrue(s_ShooterSubsystem.spin(()-> 0.1));
        //Shoot.onTrue(new InstantCommand(()->s_ShooterSubsystem.setVelocity(10)));
        //Shoot.onFalse(new InstantCommand(()->s_ShooterSubsystem.setVelocity(0)));
        // Intake.whileTrue(new InstantCommand(()-> i_IntakeSubsystem.setSpeed(1)));
        // Intake.whileFalse(new InstantCommand(()-> i_IntakeSubsystem.setSpeed(0)));
        //Intake.whileTrue(t_TurretSubsystem.run(()-> 0.019));
        //Intake.whileFalse(new InstantCommand(()-> t_TurretSubsystem.setSpeed(0)));
        //smartAim.whileTrue(new AutoAimCommand(new Translation2d(11.91, 4.02), t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem));
        //smartAim.whileTrue(new TurretPIDCommand(t_TurretSubsystem, 0.5));
        smartAim.whileTrue(new AutoAimCommand(new Translation2d(11.91, 4.02), t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem));
    }
    
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup((new InstantCommand(() -> {
             s_Swerve.gyro.reset();
            // s_Swerve.zeroHeading();
        })), autoChooser.getSelected());
    }
}
