package frc.robot.powerup.mappers;

import static org.montclairrobotics.cyborg.Cyborg.hardwareAdapter;
import org.montclairrobotics.cyborg.Cyborg;
import org.montclairrobotics.cyborg.core.mappers.CBSensorMapper;
import org.montclairrobotics.cyborg.core.utils.CBGameMode;
import org.montclairrobotics.cyborg.core.utils.CBTimingController;
import org.montclairrobotics.cyborg.devices.CBDashboardChooser;
import org.montclairrobotics.cyborg.devices.CBDigitalInput;
import org.montclairrobotics.cyborg.devices.CBEncoder;
import org.montclairrobotics.cyborg.devices.CBNavX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.powerup.RobotCB.*;

public class SensorMapper extends CBSensorMapper {

    //private RequestData rd = RobotCB.requestData; // (RequestData)Cyborg.requestData;

    // create local copies of the devices one-time
    // for use in update method
    private CBEncoder mainLiftEncoder = hardwareAdapter.getEncoder(mainLiftEncoderId);
    private CBDigitalInput mainLiftLimit = hardwareAdapter.getDigitalInput(mainLiftLimitId);
    private CBEncoder intakeLiftEncoder = hardwareAdapter.getEncoder(intakeLiftEncoderId);
    private CBEncoder drivetrainLeftEncoder = hardwareAdapter.getEncoder(dtLeftEncoderId);
    private CBEncoder drivetrainRightEncoder = hardwareAdapter.getEncoder(dtRightEncoderId);
    @SuppressWarnings("unchecked")
    private CBDashboardChooser<Character> fieldPosition = (CBDashboardChooser<Character>)hardwareAdapter.getDevice(fieldPositionId);
    @SuppressWarnings("unchecked")
    private CBDashboardChooser<String> autoSelection = (CBDashboardChooser<String>)hardwareAdapter.getDevice(autoSelectionId);
    private CBNavX navx = hardwareAdapter.getNavX(navxId);
    private CBTimingController dashboardTimer= new CBTimingController().setTiming(CBGameMode.anyPeriodic, 10);

    public SensorMapper(Cyborg robot) {
        super(robot);
    }

    @Override
    public void init() {

    }

    @Override
    public void update() {
        
        // The main work is done here transferring values from 
        // the devices to RequestData
        requestData.mainLiftEncoderValue = mainLiftEncoder.getDistance();
        requestData.mainLiftLimitValue = mainLiftLimit.get();
        requestData.drivetrainLeftEncoderValue = drivetrainLeftEncoder.getDistance();
        requestData.drivetrainRightEncoderValue = drivetrainRightEncoder.getDistance();
        requestData.drivetrainAverageEncoderValue = (requestData.drivetrainLeftEncoderValue+requestData.drivetrainRightEncoderValue)/2.0;
        requestData.robotAngle = navx.getYaw();

        // FMS Data, Driver Station (Just to make things interesting lets try this pre-game only)
        if (Cyborg.isGameMode(CBGameMode.preGame)) {
            requestData.gameSpecificMessage = DriverStation.getInstance().getGameSpecificMessage();
            requestData.fieldPosition = fieldPosition.getSelected();
            requestData.autoSelection = autoSelection.getSelected();
            requestData.nearSwitchSide = requestData.gameSpecificMessage.charAt(0);
        }

        if(dashboardTimer.update().getState()) {
            SmartDashboard.putBoolean("mainLiftLimit", requestData.mainLiftLimitValue);
            SmartDashboard.putNumber("mainLiftEncoder", requestData.mainLiftEncoderValue);
            SmartDashboard.putNumber("intakeLiftEncoder", intakeLiftEncoder.getDistance());
            SmartDashboard.putNumber("drivetrainLeftEncoder", requestData.drivetrainLeftEncoderValue);
            SmartDashboard.putNumber("drivetrainRightEncoder", requestData.drivetrainRightEncoderValue);
            SmartDashboard.putString("AutoSelectedEcho", requestData.autoSelection);
            SmartDashboard.putNumber("FieldPositionEcho", requestData.fieldPosition);
        }
    }
}
