package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.CANDevices;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeSys {
    
    public static SparkMax m_rightIntakeOutMtr = new SparkMax(CANDevices.rightIntakeOutMtrId, MotorType.kBrushed);
    public static SparkMax m_leftIntakeOutMtr = new SparkMax(CANDevices.leftIntakeOutMtrId, MotorType.kBrushed);

    public static RelativeEncoder m_rightIntakeOutEnc = m_rightIntakeOutMtr.getEncoder();
    public static RelativeEncoder m_leftIntakeOutEnc = m_leftIntakeOutMtr.getEncoder();

    private boolean Rintakeout = false;
    private boolean Lintakeout = false;

    public boolean Rintakeout() {
        return Rintakeout = true;
    }

    public boolean Lintakeout() {
        return Lintakeout = true;
    }

    public IntakeSys() {

        m_rightIntakeOutEnc.setPosition(0);
        m_leftIntakeOutEnc.setPosition(0);
        
        if(
            Rintakeout = true
        ) {
            m_rightIntakeOutEnc.setPosition(40);
        }
        else if(
            Lintakeout = true
        ) {
            m_leftIntakeOutEnc.setPosition(40);
        }
        else{
            m_rightIntakeOutEnc.setPosition(0);
        }
    }

}
