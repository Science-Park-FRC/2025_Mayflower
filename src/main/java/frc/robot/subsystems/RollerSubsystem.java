package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase{
    private final SparkMax m_rollerSpark;

    public RollerSubsystem(){
        m_rollerSpark= new SparkMax(21, MotorType.kBrushed);

    }
    public void runMotor(double speed) {
        m_rollerSpark.set(speed);
    }

}
