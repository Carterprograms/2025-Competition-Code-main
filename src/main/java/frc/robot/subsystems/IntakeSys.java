package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.CANDevices;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeSys {
    
    public static SparkMax m_intakeOutMtr = new SparkMax(CANDevices.intakeOutMtrId, MotorType.kBrushed);

    public static RelativeEncoder m_intakeOutEnc = m_intakeOutMtr.getEncoder();

    private boolean intakeout = false;

    public boolean intakeout() {
        return intakeout = true;
    }

    public IntakeSys() {

        m_intakeOutEnc.setPosition(0);
        
        if(
            intakeout = true
        ) {
            m_intakeOutEnc.setPosition(40);
        }
        else{
            m_intakeOutEnc.setPosition(0);
        }
    }

}
