package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/**
 * Class to represent and handle a swerve module
 * A module's state is measured by a CANCoder for the absolute position, integrated CANEncoder for relative position
 * for both steeration and linear movement
 */
public class SwerveModule extends SubsystemBase {

    // Instaciate Motors
    private final SparkFlex driveMtr;
    private final SparkFlex steerMtr;

    // Instaciate Encoders
    private final RelativeEncoder driveEnc;
    private final RelativeEncoder steerEnc;
    private final CANcoder canCoder;

    // Instaciate PID Controllers
    private final SparkClosedLoopController steerController;
    private final SparkClosedLoopController driveController;

    private final SparkFlexSim steeringFlexSim;
    private final SparkFlexSim drivingFlexSim;

    private final DCMotorSim steeringSim;
    private final DCMotorSim drivingSim;

    /**
     * Constructs a new SwerveModule.
     * 
     * <p>SwerveModule represents and handles a swerve module.
     * 
     * @param driveMtrId CAN ID of the NEO Vortex drive motor.
     * @param steerMtrId CAN ID of the NEO Vortex steer motor.
     * @param canCoderId CAN ID of the CANCoder.
     * @param measuredOffsetRadians Offset of CANCoder reading from forward.
     */
    public SwerveModule(int driveMtrId, int steerMtrId, int canCoderId, Rotation2d offset) {

        // Create Motors
        driveMtr = new SparkFlex(driveMtrId, MotorType.kBrushless);
        steerMtr = new SparkFlex(steerMtrId, MotorType.kBrushless);

        // Create Encoders
        driveEnc = driveMtr.getEncoder();
        steerEnc = steerMtr.getEncoder();
        canCoder = new CANcoder(canCoderId);

        // Create PID Controllers
        steerController = steerMtr.getClosedLoopController();
        driveController = driveMtr.getClosedLoopController();

        // Drive Spark Flex Configs
        SparkFlexConfig driveConfig = new SparkFlexConfig();
        driveConfig 
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        driveConfig.encoder
            .positionConversionFactor(DriveConstants.driveMetersPerEncRev)
            .velocityConversionFactor(DriveConstants.driveMetersPerSecPerMtrRPM);
        driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(DriveConstants.drivekP, 0, DriveConstants.drivekD);

        // Steering Spark Flex Configs
        SparkFlexConfig steerConfig = new SparkFlexConfig();
        steerConfig 
            .inverted(false)
            .idleMode(IdleMode.kCoast);
        steerConfig.encoder
            .positionConversionFactor(DriveConstants.steerRadiansPerEncRev);
        steerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(DriveConstants.steerkP, 0, DriveConstants.steerkD);

        // Apply Configs
        driveMtr.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        steerMtr.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Can-Encoder Config
        CANcoderConfiguration ccConfig = new CANcoderConfiguration();
        ccConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        ccConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        canCoder.getConfigurator().apply(ccConfig);

        // Initializes the steer encoder position to the CANCoder position, accounting for offset.
        steerEnc.setPosition(getCanCoderAngle().getRadians() /*- offset.getRadians()*/);

        steeringFlexSim = new SparkFlexSim(steerMtr, DCMotor.getNeoVortex(1));
        drivingFlexSim = new SparkFlexSim(driveMtr, DCMotor.getNeoVortex(1));

        DCMotor gearbox = DCMotor.getNeoVortex(1); // One motor for a drive or steering sim
        double momentOfInertia = 0.00032; // Moment of inertia in kg·m²
        LinearSystem<N2, N1, N2> dcMotorPlantDrive = 
            LinearSystemId.createDCMotorSystem(
            DriveConstants.kvVoltSecsPerMeter,
            DriveConstants.kaVoltSecsPerMeterSq);

        LinearSystem<N2, N1, N2> dcMotorPlantSteer = LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1),
        0.004, 
        1/DriveConstants.steerMtrGearReduction);
        
        steeringSim = new DCMotorSim(dcMotorPlantSteer, gearbox);
        drivingSim = new DCMotorSim(dcMotorPlantDrive, gearbox);

    }

    
    @Override
    public void periodic() {
        // Set Inputs
        steeringSim.setInputVoltage(steeringFlexSim.getAppliedOutput() * 12.0);
        drivingSim.setInputVoltage(drivingFlexSim.getAppliedOutput() * 12.0);
        // Update Sims
        steeringSim.update(0.02);
        drivingSim.update(0.02);

        steeringFlexSim.iterate(
            Units.radiansPerSecondToRotationsPerMinute(steeringSim.getAngularVelocityRadPerSec()),
            12, 0.02
        );
        
        drivingFlexSim.iterate(
            Units.radiansPerSecondToRotationsPerMinute(drivingSim.getAngularVelocityRadPerSec()),
            12,
            0.02
        );

        canCoder.getSimState().setRawPosition(steeringSim.getAngularPositionRotations());

        SmartDashboard.putNumber("driveMotor/id" + driveMtr.getDeviceId() + "/simPos", drivingSim.getAngularPositionRad());

        /*steeringSim.setAngle(Units.radiansToDegrees(steeringSim.getAngularPositionRad()));
        drivingSim.setAngle(Units.radiansToDegrees(drivingSim.getAngularPositionRad()));*/

    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {

    return new SwerveModulePosition(
        driveEnc.getPosition(), getSteerEncAngle());
    }

    /**
     * Resets the distance traveled by the module to zero.
     */
    public void resetDriveDistance() {
        driveEnc.setPosition(0.0);
    }

    /**
     * Returns the current drive distance of the module.
     * 
     * @return The current drive distance of the module.
     */
    public double getDriveDistanceMeters() {

        return driveEnc.getPosition();

    }
    
    /**
     * Returns the current absolute angle of the module from the CANCoder.
     * This measurement does not account for offset.
     * It is preferred to use the method getSteerEncAngle().
     * 
     * @return The value of the CANCoder.
     */
    public Rotation2d getCanCoderAngle() {
        return new Rotation2d(canCoder.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI);
    }

    /**
     * Returns the current absolute angle of the module from the steer motor encoder.
     * This measurement accounts for offset.
     * 
     * @return The current absolute angle of the module.
     */
    public Rotation2d getSteerEncAngle() {
        return new Rotation2d(steerEnc.getPosition());
    }

    /**
     * Returns the current velocity of the module from the drive motor encoder.
     * 
     * @return The current velocity of the module in meters per second.
     */
    public double getVelocityMetersPerSec() {
        return driveEnc.getVelocity();
    }

    /**
     * Calculates the angle motor setpoint based on the desired angle and the current angle measurement.
     * 
     * @param targetAngle The desired angle to set the module in radians.
     * @param currentAngle The current angle of the module in radians.
     * 
     * @return The adjusted target angle for the module in radians.
     */
    public double calculateAdjustedAngle(double targetAngle, double currentAngle) {
        double modAngle = currentAngle % (2.0 * Math.PI);

        if (modAngle < 0.0) modAngle += 2.0 * Math.PI;
        
        double newTarget = targetAngle + currentAngle - modAngle;

        if (targetAngle - modAngle > Math.PI) newTarget -= 2.0 * Math.PI;
        else if (targetAngle - modAngle < -Math.PI) newTarget += 2.0 * Math.PI;

        return newTarget;
    }

    /**
     * Sets the desired state of the swerve module and optimizes it.
     * <p>If closed-loop, uses PID and a feedforward to control the speed.
     * If open-loop, sets the speed to a percentage. Open-loop control should
     * only be used if running an autonomour trajectory.
     *
     * @param desiredState Object that holds a desired linear and steerational setpoint.
     * @param isClosedLoop True if the velocity control is closed-loop.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isClosedLoop) {
        
        // Optimizes speed and angle to minimize change in heading
        // (e.g. module turns 1 degree and reverses drive direction to get from 90 degrees to -89 degrees)
        desiredState = SwerveModuleState.optimize(desiredState, getSteerEncAngle());

        // Scale velocity based on turn error to help prevent skew.
        double angleErrorRad = desiredState.angle.getRadians() - getSteerEncAngle().getRadians();
        desiredState.speedMetersPerSecond *= Math.cos(angleErrorRad);

        steerController.setReference(
            calculateAdjustedAngle(
                desiredState.angle.getRadians(),
                getSteerEncAngle().getRadians()),
            ControlType.kPosition
        );

        if(!isClosedLoop) {
            driveMtr.set(desiredState.speedMetersPerSecond / DriveConstants.freeMetersPerSecond);
        }
        else {
            /*driveController.setReference(
                desiredState.speedMetersPerSecond,
                ControlType.kVelocity,
                0, 
                DriveConstants.driveFF.calculate(desiredState.speedMetersPerSecond)
            );*/
            driveController.setReference(0.0,ControlType.kPosition);
        }
    }

    public void runCharacterization(double volts) {
        steerController.setReference(
            0.0,
            ControlType.kPosition
        );

        driveMtr.setVoltage(volts);
    }

    public double getDriveVoltage() {
        return driveMtr.get() * 12.0;
    }
}