package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.RobotContainer;

import frc.robot.Constants.CANDevices;

public class LiftSys {
    
    SparkMax m_leftLiftMtr = new SparkMax(CANDevices.m_leftLiftMtrId, MotorType.kBrushless);
    SparkMax m_rightLiftMtr = new SparkMax(CANDevices.m_rightLiftMtrId, MotorType.kBrushless);

    public LiftSys() {

        double deadzone = 0.5;

        if(
            RobotContainer.operatorController.getRightTriggerAxis() > deadzone
        ){
            m_leftLiftMtr.set(RobotContainer.operatorController.getRightTriggerAxis());
            m_rightLiftMtr.set(RobotContainer.operatorController.getRightTriggerAxis());
        }
        else if (
            RobotContainer.operatorController.getLeftTriggerAxis() > deadzone
        ){
            m_leftLiftMtr.set(-RobotContainer.operatorController.getLeftTriggerAxis());
            m_rightLiftMtr.set(-RobotContainer.operatorController.getLeftTriggerAxis());
        }
        else{
            m_leftLiftMtr.set(0);
            m_rightLiftMtr.set(0);
        }
        
    }

}
