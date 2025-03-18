package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CANDevices;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SmartMotionConfig;
import com.revrobotics.RelativeEncoder;

public class IntakeSys extends SubsystemBase {
    private SparkMax motor;
    private RelativeEncoder encoder;
    private PIDController intakeController;
    public final static CommandXboxController operatorController = new CommandXboxController(3);

    public static SparkMax m_rightIntakeMtr = new SparkMax(31, MotorType.kBrushed);
    public static SparkMax m_leftIntakeMtr = new SparkMax(30, MotorType.kBrushed);

    Encoder m_rightIntakeEnc;
    Encoder m_leftIntakeEnc;

    double rollerSpeed = -0.3;
    double reverseRollerSpeed = 0.3;

    private boolean Rintakeout = false;
    private boolean Rintakeoutrun = false;
    private boolean Lintakeout = false;
    private boolean Lintakeoutrun = false;
    private boolean Lintakein = false;
    private boolean Rintakein = false;
    private boolean intakein = false;

    public boolean Rintakeout() {
        return Rintakeout = true;
    }

    public boolean Rintakeoutrun() {
        return Rintakeoutrun = true;
    }

    public boolean Lintakeout() {
        return Lintakeout = true;
    }

    public boolean Lintakeoutrun() {
        return Lintakeoutrun = true;
    }

    public boolean Rintakein() {
        return Rintakein = true;
    }

    public boolean Lintakein() {
        return Lintakein = true;
    }

    public boolean intakein() {
        return intakein = true;
    }

    public IntakeSys() {

        m_rightIntakeEnc = new Encoder(3, 2, false);
        m_leftIntakeEnc = new Encoder(1, 0, false);

        m_leftIntakeEnc.reset();
        m_rightIntakeEnc.reset();

        m_leftIntakeEnc.setDistancePerPulse(3);
        
    }

    @Override
    public void periodic() {
        if(DriverStation.isTeleopEnabled() == true){
        if(
            operatorController.getRawAxis(0) < -0.95 && operatorController.getRawAxis(1) < 0.95 && operatorController.getRawAxis(1) > -0.95
        ) {
            Lintakeout();
        }
        else if(
            operatorController.getRawAxis(0) > 0.95 && operatorController.getRawAxis(1) > -0.95 && operatorController.getRawAxis(1) < 0.95
        ) {
            Rintakeout();
        }
        else if(
            operatorController.getRawAxis(1) < -0.95 && operatorController.getRawAxis(0) > 0.95
        ) {
            System.out.println("Right Intake out and moving fwd");
        }
        else if(
            operatorController.getRawAxis(1) < 0.95 && operatorController.getRawAxis(1) > -0.95 && operatorController.getRawAxis(0) < 0.95 && operatorController.getRawAxis(0) > -0.95
        ) {
            intakein = true;
        }
        else if(
            operatorController.getRawAxis(1) > 0.95 && operatorController.getRawAxis(0) < -0.95
        ) {
            System.out.println("Left Intake Out and Moving Back");
        }
        else if(operatorController.getRawAxis(1) < -0.95 && operatorController.getRawAxis(0) < -0.95) {
            System.out.println("Left Intake Out and Moving Fwd");
        }
        else if(operatorController.getRawAxis(1) > 0.95 && operatorController.getRawAxis(0) > 0.95){
            System.out.println("Right Intake Out and Moving Bwd");
        }
        }
        if(Rintakeout == true && m_rightIntakeEnc.getDistance() > 32.87){
            m_rightIntakeMtr.set(0.3);
            Rintakeout = false;
        }
        else if(
            Rintakeout == true && m_rightIntakeEnc.getDistance() < 32.87
        ) {
            m_rightIntakeMtr.set(-0.3);
            Rintakeout = false;
        }
        if(
            Lintakeout == true && m_leftIntakeEnc.getDistance() > 32.87
        ){
            m_rightIntakeMtr.set(0.3);
            Lintakeout = false;
        }
        else if(
            Lintakeout == true && m_leftIntakeEnc.getDistance() < 32.87
        ) {
            m_leftIntakeMtr.set(-0.3);
            Lintakeout = false;
        }
        if (
           intakein == true && m_rightIntakeEnc.getDistance() < 0
        ){
            m_rightIntakeMtr.set(0.3);
        }
        else if (
            intakein == true && m_rightIntakeEnc.getDistance() > 0
        ){
            m_rightIntakeMtr.set(-0.3);
        }
        else if (intakein == true && m_leftIntakeEnc.getDistance() < 0){
            m_leftIntakeMtr.set(0.3);
        }
        else if (
            intakein == true && m_leftIntakeEnc.getDistance() > 0
        ) {
            m_leftIntakeMtr.set(-0.3);
        }
    }

}