package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.CANDevices;

public class LiftSys extends SubsystemBase {
    
    public static SparkMax m_liftMtr = new SparkMax(CANDevices.m_leftLiftMtrId, MotorType.kBrushless);

    private final PIDController liftController = new PIDController(0, 0, 0);

    RelativeEncoder m_liftEnc = m_liftMtr.getEncoder();

    private boolean islvl4Called = false;
    private boolean islvl3Called = false;
    private boolean islvl2Called = false;
    private boolean islvl1Called = false;
    private boolean islvl0Called = false;

    private double lvl0Pose = 0;
    private double lvl1Pose = 5;
    private double lvl2Pose = 10;
    private double lvl3Pose = 15;
    private double lvl4Pose = 20;

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

        // Set the starting position of the lift to 0.
        m_liftEnc.setPosition(0);

    }

    @Override
    public void periodic() {
        if(
            islvl4Called == true
        ){
            m_liftMtr.set(liftController.calculate(m_liftEnc.getPosition(), lvl4Pose));
            islvl4Called = false;
            System.out.println("lvl4");
        }
        else if (
            islvl3Called == true
        ){
            m_liftMtr.set(liftController.calculate(m_liftEnc.getPosition(), lvl3Pose));
            islvl3Called = false;
            System.out.println("lvl3");
        }
        else if (
            islvl2Called == true
        ) {
            m_liftMtr.set(liftController.calculate(m_liftEnc.getPosition(), lvl2Pose));
            islvl2Called = false;
            System.out.println("lvl2");
        }
        else if (
            islvl1Called == true
        ) {
            m_liftMtr.set(liftController.calculate(m_liftEnc.getPosition(), lvl1Pose));
            islvl1Called = false;
            System.out.println("lvl1");
        }
        else if(
            islvl0Called == true && m_liftEnc.getPosition() < 0.3
        ){
            m_liftMtr.set(0);
        }
        else if(
            islvl0Called == true
        ) {
            m_liftMtr.set(liftController.calculate(m_liftEnc.getPosition(), lvl0Pose));
            System.out.println("lvl0");
        }
        else {
            m_liftMtr.set(liftController.calculate(m_liftEnc.getPosition(), lvl0Pose));
        }
    }

}
