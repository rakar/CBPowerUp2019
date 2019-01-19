package frc.robot.powerup;

//#region imports
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import org.montclairrobotics.cyborg.Cyborg;
import org.montclairrobotics.cyborg.core.assemblies.CBDriveModule;
import org.montclairrobotics.cyborg.core.assemblies.CBSimpleSpeedControllerArray;
import org.montclairrobotics.cyborg.core.behaviors.CBStdDriveBehavior;
import org.montclairrobotics.cyborg.core.controllers.CBDifferentialDriveController;
import org.montclairrobotics.cyborg.core.controllers.CBLiftController;
import org.montclairrobotics.cyborg.core.mappers.CBArcadeDriveMapper;
import org.montclairrobotics.cyborg.core.mappers.CBMotorMonitorMapper;
import org.montclairrobotics.cyborg.core.utils.CB2DVector;
import org.montclairrobotics.cyborg.core.utils.CBEnums;
import org.montclairrobotics.cyborg.core.utils.CBPIDErrorCorrection;
import org.montclairrobotics.cyborg.devices.CBAxis;
import org.montclairrobotics.cyborg.devices.CBButton;
import org.montclairrobotics.cyborg.devices.CBDashboardChooser;
import org.montclairrobotics.cyborg.devices.CBDeviceID;
import org.montclairrobotics.cyborg.devices.CBDigitalInput;
import org.montclairrobotics.cyborg.devices.CBEncoder;
import org.montclairrobotics.cyborg.devices.CBHardwareAdapter;
import org.montclairrobotics.cyborg.devices.CBNavX;
import org.montclairrobotics.cyborg.devices.CBPDB;
import org.montclairrobotics.cyborg.devices.CBSpeedControllerFaultCriteria;
import org.montclairrobotics.cyborg.devices.CBTalonSRX;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.powerup.behaviors.AutoExample1;
import frc.robot.powerup.behaviors.IntakeLiftBehavior;
import frc.robot.powerup.behaviors.MainLiftBehavior;
import frc.robot.powerup.behaviors.OperatorBehavior;
import frc.robot.powerup.data.ControlData;
import frc.robot.powerup.data.RequestData;
import frc.robot.powerup.mappers.OperatorMapper;
import frc.robot.powerup.mappers.SensorMapper;
//#endregion

public class RobotCB extends Cyborg {
    // constants
    // joystick ports
    private final int driveStickID = 0;
    private final int operatorStickID = 1;

    //
    // the devices have been changed to "public static" from "private" to allow
    // for direct access in cases of NON-reusable mappers/behaviors
    // that will always be robot specific anyway in order to avoid lengthy init code. 
    // Reusable code, should of course use setter functions to
    // attach to devices.
    //
    //#region Device List...
    public static CBDeviceID 
    // modules
    pdbId, navxId,

    // driver controls
    driveRotAxisId, driveFwdAxisId, gyroLockButtonId,

    // operator controls
    operRotAxisId, operFwdAxisId, shootCubeButtonId, intakeLiftUpButtonId, intakeLiftDownButtonId, mainLiftUpButtonId, mainLiftDownButtonId,

    // drivetrain Motors
    dtFrontLeftMotorId, dtFrontRightMotorId, dtBackLeftMotorId, dtBackRightMotorId,

    // dt Encoders
    dtLeftEncoderId, dtRightEncoderId,

    //lift Motors
    mainLiftMotorFrontId, mainLiftMotorBackId, intakeLiftMotorId,

    // lift encoders
    mainLiftEncoderId, intakeLiftEncoderId,

    // lift limit switches
    mainLiftLimitId,

    // intake motors
    intakeLeftMotorId, intakeRightMotorId,

    // dashboard choosers
    fieldPositionId, autoSelectionId
    ;
    //#endregion

    public static RequestData requestData;
    public static ControlData controlData;

    public RobotCB() {
    }

    @Override
    public void cyborgInit() {

        // data init
        requestData = new RequestData();
        controlData = new ControlData();

        defineDevices();
        defineMappers();
        defineControllers();
        defineBehaviors();
    }

    @Override
    public void cyborgDisabledInit() {

    }

    @Override
    public void cyborgAutonomousInit() {
        switch(requestData.autoSelection) {
            case "auto1":
                this.addAutonomous(new AutoExample1(this));
                break;
            default:
                // just say no
                break;
        }
    }

    @Override
    public void cyborgTeleopInit() {

    }

    @Override
    public void cyborgTestInit() {

    }

    private void defineDevices() {
        // Configure Hardware Adapter and Devices
        hardwareAdapter = new CBHardwareAdapter(this);
        //ha = hardwareAdapter;

        pdbId = hardwareAdapter.add(
                new CBPDB()
        );

        navxId = hardwareAdapter.add(
                new CBNavX(SPI.Port.kMXP)
        );

        // setup drivetrain
        dtFrontLeftMotorId = hardwareAdapter.add(
                new CBTalonSRX(1)
                        .setDeviceName("DriveTrain", "FrontLeft")
                        .setPowerSource(pdbId, 0)
                        .setSpeedControllerFaultCriteria(
                                new CBSpeedControllerFaultCriteria()
                                        .setBasic(.1, 1, 30)
                        )
        );
        dtFrontRightMotorId = hardwareAdapter.add(
                new CBTalonSRX(7)
                        .setDeviceName("DriveTrain", "FrontRight")
                        .setPowerSource(pdbId, 1)
                        .setSpeedControllerFaultCriteria(
                                new CBSpeedControllerFaultCriteria()
                                        .setBasic(.1, 1, 30)
                        )
        );
        dtBackLeftMotorId = hardwareAdapter.add(
                new CBTalonSRX(3)
                        .setDeviceName("DriveTrain", "BackLeft")
                        .setPowerSource(pdbId, 2)
                        .setSpeedControllerFaultCriteria(
                                new CBSpeedControllerFaultCriteria()
                                        .setBasic(.1, 1, 30)
                        )
        );
        dtBackRightMotorId = hardwareAdapter.add(
                new CBTalonSRX(8)
                        .setDeviceName("DriveTrain", "BackRight")
                        .setPowerSource(pdbId, 3)
                        .setSpeedControllerFaultCriteria(
                                new CBSpeedControllerFaultCriteria()
                                        .setBasic(.1, 1, 30)
                        )
        );
        final double inchesPerTick = 96 / 4499;
        dtLeftEncoderId = hardwareAdapter.add(
                new CBEncoder(1, 0, CounterBase.EncodingType.k4X, false, inchesPerTick)
        );
        dtRightEncoderId = hardwareAdapter.add(
                new CBEncoder(3, 2, CounterBase.EncodingType.k4X, false, inchesPerTick)
        );


        // setup main lift
        mainLiftMotorFrontId = hardwareAdapter.add(
                new CBTalonSRX(4)
                        .setDeviceName("MainLift", "Front")
                        .setPowerSource(pdbId, 4)
                        .setSpeedControllerFaultCriteria(
                                new CBSpeedControllerFaultCriteria()
                                        .setBasic(.1, 1, 20)
                        )
        );
        mainLiftMotorBackId = hardwareAdapter.add(
                new CBTalonSRX(2)
                        .setDeviceName("MainLift", "Back")
                        .setPowerSource(pdbId, 5)
                        .setSpeedControllerFaultCriteria(
                                new CBSpeedControllerFaultCriteria()
                                        .setBasic(.1, 1, 20)
                        )
        );
        mainLiftEncoderId = hardwareAdapter.add(
                new CBEncoder(4, 5, CounterBase.EncodingType.k4X, false, 1)
        );
        mainLiftLimitId = hardwareAdapter.add(
                new CBDigitalInput(9)
        );


        // setup intake lift
        intakeLiftMotorId = hardwareAdapter.add(
                new CBTalonSRX(9)
                        .setDeviceName("Intake", "LiftMotor")
                        .setPowerSource(pdbId, 6)
                        .setSpeedControllerFaultCriteria(
                                new CBSpeedControllerFaultCriteria()
                                        .setBasic(.1, 1, 20)
                        )
        );
        intakeLiftEncoderId = hardwareAdapter.add(
                new CBEncoder(intakeLiftMotorId, FeedbackDevice.QuadEncoder, false, 1)
        );


        // setup intake motors
        intakeLeftMotorId = hardwareAdapter.add(
                new CBTalonSRX(10)
                        .setDeviceName( "Intake", "LeftMotor")
                        .setPowerSource(pdbId, 8)
                        .setSpeedControllerFaultCriteria(
                                new CBSpeedControllerFaultCriteria()
                                        .setBasic(.1, 1, 30)
                        )
        );
        intakeRightMotorId = hardwareAdapter.add(
                new CBTalonSRX(5)
                        .setDeviceName("Intake", "RightMotor")
                        .setPowerSource(pdbId, 9)
                        .setSpeedControllerFaultCriteria(
                                new CBSpeedControllerFaultCriteria()
                                        .setBasic(.1, 1, 30)
                        )
        );

        // driver controls
        driveRotAxisId = hardwareAdapter.add(
                new CBAxis(driveStickID, 1)
                        .setDeadzone(0.1)
                        //this is the new default scale of rotation for CBArcadeDriveMapper
                        //.setScale(-1.0)  
        );
        driveFwdAxisId = hardwareAdapter.add(
                new CBAxis(driveStickID, 0)
                        .setDeadzone(0.1)
                        //this is the new default scale of fwd for CBArcadeDriveMapper
                        //.setScale(-1.0)
        );
        gyroLockButtonId = hardwareAdapter.add(
                new CBButton(driveStickID, 1)
        );

        // operator controls
        operRotAxisId = hardwareAdapter.add(
                new CBAxis(operatorStickID, 1)
                        .setDeadzone(0.1)
                        //this is the new default scale of rotation for CBArcadeDriveMapper
                        //.setScale(-1.0)
        );

        operFwdAxisId = hardwareAdapter.add(
                new CBAxis(operatorStickID, 0)
                        .setDeadzone(0.1)
                        //this is the new default scale of fwd for CBArcadeDriveMapper
                        //.setScale(-1.0)
        );

        shootCubeButtonId = hardwareAdapter.add(
                new CBButton(operatorStickID, 1)
        );
        intakeLiftUpButtonId = hardwareAdapter.add(
                new CBButton(operatorStickID, 3)
        );
        intakeLiftDownButtonId = hardwareAdapter.add(
                new CBButton(operatorStickID, 2)
        );
        mainLiftUpButtonId = hardwareAdapter.add(
                new CBButton(operatorStickID, 4)
        );
        mainLiftDownButtonId = hardwareAdapter.add(
                new CBButton(operatorStickID, 5)
        );

        // dashboard elements
        fieldPositionId = hardwareAdapter.add(
                new CBDashboardChooser<Character>("Field Position")
                        .addChoice("left",'L')
                        .addChoice("center", 'C')
                        .addChoice("right",'R')
        );
        autoSelectionId = hardwareAdapter.add(
                new CBDashboardChooser<String>("Autonomous")
                        .addDefault("NONE!!!", "none")
                        .addChoice("Auto1", "auto1")

        );
    }

    private void defineMappers() {
        // setup teleop mappers
        this.addTeleOpMapper(
                new CBArcadeDriveMapper(this, requestData.drivetrain)
                        .setAxes(driveFwdAxisId, null, driveRotAxisId)
                        .setGyroLockButton(gyroLockButtonId)
        );

        // Here is a hack:
        // create a second "drivetrain" to operate the intake
        // because they work the same way...
        this.addTeleOpMapper(
                new CBArcadeDriveMapper(this, requestData.intake)
                        .setAxes(operFwdAxisId, null, operRotAxisId)
        );

        this.addTeleOpMapper(
                new OperatorMapper(this)
        );

        // setup sensor mapper(s)
        this.addSensorMapper(
                new SensorMapper(this)
        );

        this.addSensorMapper(
                new CBMotorMonitorMapper(this)
                        .add(dtFrontLeftMotorId)
                        .add(dtFrontRightMotorId)
                        .add(dtBackLeftMotorId)
                        .add(dtBackRightMotorId)
                        .add(mainLiftMotorBackId)
                        .add(mainLiftMotorFrontId)
                        .add(intakeLiftMotorId)
        );
    }

    private void defineControllers() {
        // setup robot controllers
        this.addRobotController(
                new CBDifferentialDriveController(this, controlData.drivetrain)
                        .addLeftDriveModule(
                                new CBDriveModule(new CB2DVector(-1, 0), 0)                                        
                                        .addSpeedControllerArray(
                                                new CBSimpleSpeedControllerArray()
                                                        .setDriveMode(CBEnums.CBDriveMode.Power)
                                                        .addSpeedController(dtFrontLeftMotorId)
                                                        .addSpeedController(dtBackLeftMotorId)
                                                        .setEncoder(dtLeftEncoderId)
                                                        .setErrorCorrection(
                                                                new CBPIDErrorCorrection()
                                                                        .setConstants(new double[]{1.5, 0, 0.0015}
                                                                        )
                                                        )
                                        )
                        )
                        .addRightDriveModule(
                                new CBDriveModule(new CB2DVector(1, 0), 180)
                                        .addSpeedControllerArray(
                                                new CBSimpleSpeedControllerArray()
                                                        .setDriveMode(CBEnums.CBDriveMode.Power)
                                                        .addSpeedController(dtFrontRightMotorId)
                                                        .addSpeedController(dtBackRightMotorId)
                                                        .setEncoder(dtRightEncoderId)
                                                        .setErrorCorrection(
                                                                new CBPIDErrorCorrection()
                                                                        .setConstants(new double[]{1.5, 0, 0.0015}
                                                                        )
                                                        )
                                        )
                        )
        );

        // yup again with the second drivetrain for the intake
        this.addRobotController(
                new CBDifferentialDriveController(this, controlData.intake)
                        .addLeftDriveModule(
                                new CBDriveModule(new CB2DVector(-6, 0), 0)
                                        .addSpeedControllerArray(
                                                new CBSimpleSpeedControllerArray()
                                                        .setDriveMode(CBEnums.CBDriveMode.Power)
                                                        .addSpeedController(intakeLeftMotorId)
                                        )
                        )
                        .addRightDriveModule(
                                new CBDriveModule(new CB2DVector(6, 0), -180)
                                        .addSpeedControllerArray(
                                                new CBSimpleSpeedControllerArray()
                                                        .setDriveMode(CBEnums.CBDriveMode.Power)
                                                        .addSpeedController(intakeRightMotorId)
                                        )
                        )
        );

        // main lift controller definition
        this.addRobotController(
                // hardware configurations are done here.
                // there are other "soft" configurations done in the behavior
                // that include margins (which trigger slow motion)
                // and in this case a encoder based top limit
                new CBLiftController(this, 
                        controlData.mainLift, 
                        new CBSimpleSpeedControllerArray()
                                .addSpeedController(mainLiftMotorFrontId)
                                .setDriveMode(CBEnums.CBDriveMode.Power)
                        )
                        // set a lower limit switch. this is a hard limit
                        .setBottomLimit(mainLiftLimitId)
                        // set the encoder for the lift
                        .setEncoder(mainLiftEncoderId)
        );

        // intake lift controller definition
        this.addRobotController(
                // hardware configurations are done here.
                // there are other "soft" configurations done in the behavior
                // that include margins (which trigger slow motion)
                // and in this case an encoder based top limit
                new CBLiftController(this,
                        controlData.intakeLift,
                        new CBSimpleSpeedControllerArray()
                                .addSpeedController(intakeLiftMotorId)
                                .setDriveMode(CBEnums.CBDriveMode.Power)
                        )
                        // set a lower limit switch this is a hard limit
                        //.setBottomLimit(mainLiftLimit)
                        // set the encoder for the lift
                        .setEncoder(intakeLiftEncoderId)
        );
    }

    private void defineBehaviors() {
        // setup behaviors
        this.addBehavior(
                new CBStdDriveBehavior(this, 
                        requestData.drivetrain, 
                        controlData.drivetrain
                        )
        );

        this.addBehavior(new MainLiftBehavior(this));
        this.addBehavior(new IntakeLiftBehavior(this));

        // while this looks like a drivetrain, its an intake.
        this.addBehavior(
                new CBStdDriveBehavior(this, 
                        requestData.intake, 
                        controlData.intake
                        )
        );

        this.addBehavior(new OperatorBehavior(this));
    }
}