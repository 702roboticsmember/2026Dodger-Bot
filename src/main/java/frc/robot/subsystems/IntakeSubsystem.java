package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class IntakeSubsystem extends SubsystemBase {
    public TalonFX IntakeMotor = new TalonFX(0);

    public IntakeSubsystem(){
        
        IntakeMotor.getConfigurator().apply(Robot.CTRE_CONFIGS.intakeConfigs);
    }
    public void set(double value) {
        IntakeMotor.setVoltage(value * 12);
    }
    public void setMode(NeutralModeValue mode) {
        IntakeMotor.setNeutralMode(mode);
    }

    public Command runCmd(double value) {
        return this.run(() -> this.set(value));
    }
}
