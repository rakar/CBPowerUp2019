package frc.robot.powerup.mappers;

import static org.montclairrobotics.cyborg.Cyborg.hardwareAdapter;
import org.montclairrobotics.cyborg.Cyborg;
import org.montclairrobotics.cyborg.core.mappers.CBTeleOpMapper;
import org.montclairrobotics.cyborg.devices.CBButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.powerup.RobotCB.*;

import frc.robot.powerup.RobotCB;
import frc.robot.powerup.data.RequestData;

public class OperatorMapper extends CBTeleOpMapper {

    //private RequestData rd = RobotCB.requestData; // (RequestData) Cyborg.requestData;

    private CBButton shootCubeButton = hardwareAdapter.getButton(shootCubeButtonId);
    private CBButton intakeLiftUpButton = hardwareAdapter.getButton(intakeLiftUpButtonId);
    private CBButton intakeLiftDownButton = hardwareAdapter.getButton(intakeLiftDownButtonId);
    private CBButton mainLiftUpButton = hardwareAdapter.getButton(mainLiftUpButtonId);
    private CBButton mainLiftDownButton = hardwareAdapter.getButton(mainLiftDownButtonId);
    
    public OperatorMapper(Cyborg robot) {
        super(robot);
    }

    @Override
    public void init() {

    }

    @Override
    public void update() {
        requestData.shootCube      = shootCubeButton.getState();
        requestData.intakeLiftUp   = intakeLiftUpButton.getState();
        requestData.intakeLiftDown = intakeLiftDownButton.getState();
        requestData.mainLiftUp     = mainLiftUpButton.getState();
        requestData.mainLIftDown   = mainLiftDownButton.getState();

        SmartDashboard.putBoolean("intakeLiftUpButton", requestData.intakeLiftUp);
        SmartDashboard.putBoolean("intakeLiftDownButton", requestData.intakeLiftDown);
    }
}
