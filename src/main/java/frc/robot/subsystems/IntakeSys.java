package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.CANDevices;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSys extends SubsystemBase {
    
    public static SparkMax m_rightIntakeMtr = new SparkMax(CANDevices.rightIntakeOutMtrId, MotorType.kBrushed);
    public static SparkMax m_leftIntakeMtr = new SparkMax(CANDevices.leftIntakeOutMtrId, MotorType.kBrushed);

    public static RelativeEncoder m_rightIntakeEnc = m_rightIntakeMtr.getEncoder();
    public static RelativeEncoder m_leftIntakeEnc = m_leftIntakeMtr.getEncoder();

    private boolean Rintakeout = false;
    private boolean Lintakeout = false;
    private boolean Lintakein = false;
    private boolean Rintakein = false;

    public boolean Rintakeout() {
        return Rintakeout = true;
    }

    public boolean Lintakeout() {
        return Lintakeout = true;
    }

    public boolean Rintakein() {
        return Rintakein = true;
    }

    public boolean Lintakein() {
        return Rintakein = true;
    }

    public IntakeSys() {

        m_rightIntakeEnc.setPosition(0);
        m_leftIntakeEnc.setPosition(0);
        
    }

    @Override
    public void periodic() {
        if(
            Rintakeout == true
        ) {
            m_rightIntakeEnc.setPosition(40);
            Rintakeout = false;
        }
        else if(
            Lintakeout == true
        ) {
            m_leftIntakeEnc.setPosition(40);
            Lintakeout = false;
        }
        else if (Rintakein == true) {
            m_rightIntakeEnc.setPosition(0);
        }
        else if (Lintakein == true) {
            m_rightIntakeEnc.setPosition(0);
        }
    }

}
