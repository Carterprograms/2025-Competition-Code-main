package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.RobotContainer;

public class ConveyorSys extends SubsystemBase {

    public static SparkMax m_conveyorMtr = new SparkMax(CANDevices.m_conveyorMtrId, MotorType.kBrushless);

    private boolean ConveyorFwd = false;
    private boolean ConveyorBwd = false;
    private boolean ConveyorStp = false;

    public boolean ConveyorFwd() {
        return ConveyorFwd = true;
    }

    @Override
    public void periodic() {

        if(RobotContainer.ButtonPanel.getRawButton(11) == false){
            ConveyorBwd = true;
        }

        if (ConveyorFwd == true
        ) {
            m_conveyorMtr.set(1);
            ConveyorFwd = false;
            ConveyorStp = true;
        }
        else if (ConveyorBwd == true && ConveyorFwd == true
        ) {
            m_conveyorMtr.set(-1);
            ConveyorBwd = false;
            ConveyorStp = true;
        }
        else if (ConveyorStp == true
        ) {
            m_conveyorMtr.set(0);
        }
    }

    public ConveyorSys() {

    }

}
