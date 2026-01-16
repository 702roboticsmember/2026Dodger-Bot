package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX ShooterMotor = new TalonFX(Constants.ShootSubsystem.ShootMotorID);

    final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

    public ShooterSubsystem(){
        
        ShooterMotor.getConfigurator().apply(Robot.CTRE_CONFIGS.shooterConfigs);
    }
    public void set(double value) {
        ShooterMotor.setVoltage(value * 12);
    }
    public void setMode(NeutralModeValue mode) {
        ShooterMotor.setNeutralMode(mode);
    }

    public void setVelocity(double value){
        ShooterMotor.setControl(m_request.withVelocity(value));
    }

    public Command runCmd(DoubleSupplier value) {
        return this.runEnd(() -> this.set(value.getAsDouble()), () -> this.set(0));
    }
}
