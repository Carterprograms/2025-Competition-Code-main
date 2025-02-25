package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Constants.CANDevices;

public class LiftSys extends SubsystemBase {
    
    public static SparkMax m_leftLiftMtr = new SparkMax(CANDevices.m_leftLiftMtrId, MotorType.kBrushless);
    public static SparkMax m_rightLiftMtr = new SparkMax(CANDevices.m_rightLiftMtrId, MotorType.kBrushless);

    RelativeEncoder m_leftliftEnc = m_leftLiftMtr.getEncoder();
    RelativeEncoder m_rightliftEnc = m_rightLiftMtr.getEncoder();

    private boolean islvl4Called = false;
    private boolean islvl3Called = false;
    private boolean islvl2Called = false;
    private boolean islvl1Called = false;
    private boolean islvl0Called = false;

    public boolean islvl4Called() {
        return islvl4Called = true;
    }

    public boolean islvl3Called() {
        return islvl3Called = true;
    }

    public boolean islvl2Called() {
        return islvl2Called = true;
    }

    public boolean islvl1Called() {
        return islvl1Called = true;
    }

    public boolean islvl0Called() {
        return islvl0Called = true;
    }

    public void lvl4() {
        islvl4Called = true;
    }

    public void lvl3() {
        islvl3Called = true;
    }

    public void lvl2() {
        islvl2Called = true;
    }

    public void lvl1() {
        islvl1Called = true;
    }

    public void lvl0() {
        islvl0Called = true;
    }
    
    public LiftSys() {

        m_leftliftEnc.setPosition(0);
        m_rightliftEnc.setPosition(0);

    }

    @Override
    public void periodic() {
        if(
            islvl4Called == true
        ){
            m_leftliftEnc.setPosition(400);
            m_rightliftEnc.setPosition(400);
            islvl4Called = false;
            islvl0Called = true;
            System.out.println("lvl4");
        }
        else if (
            islvl3Called == true
        ){
            m_leftliftEnc.setPosition(300);
            m_rightliftEnc.setPosition(300);
            islvl3Called = false;
            islvl0Called = true;
            System.out.println("lvl3");
        }
        else if (
            islvl2Called == true
        ) {
            m_leftliftEnc.setPosition(200);
            m_rightliftEnc.setPosition(200);
            islvl2Called = false;
            islvl0Called = true;
            System.out.println("lvl2");
        }
        else if (
            islvl1Called == true
        ) {
            m_leftliftEnc.setPosition(100);
            m_rightliftEnc.setPosition(100);
            islvl1Called = false;
            islvl0Called = true;
            System.out.println("lvl1");
        }
        else if(
            islvl0Called == true
        ) {
            m_leftliftEnc.setPosition(0);
            m_rightliftEnc.setPosition(0);
            System.out.println("lvl0");
        }
    }

}
