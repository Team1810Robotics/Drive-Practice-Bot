package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private SparkMax intakeMotor;

    public IntakeSubsystem() {
        //TODO: Figure out this port
        intakeMotor = new SparkMax(0, MotorType.kBrushless);
    }



    public void intakeIn() {
        intakeMotor.set(1);
    }

    public void intakeOut() {
        intakeMotor.set(-1);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }
}
