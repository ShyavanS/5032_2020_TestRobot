package frc.robot;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import lombok.Getter;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

@Getter
public class CameraSwitcher {

    private String name;
    private int selected = 0;

    private List<UsbCamera> cameras = new ArrayList<>();
    private MjpegServer switcher;

    private Joystick joystick;
    private int buttonID;

    public CameraSwitcher(String name, Joystick joystick, int buttonID, int... ids) {
        this.name = name;
        this.joystick = joystick;
        this.buttonID = buttonID;

        switcher = CameraServer.getInstance().addSwitchedCamera(name);
        switcher.setFPS(60);
        switcher.setResolution(320, 240);

        cameras.addAll(Arrays.stream(ids).mapToObj(this::getUSBCamera).collect(Collectors.toList()));
    }

    public void tick() {
        boolean status = joystick.getRawButtonPressed(buttonID);
        if (status) {
            selected++;
            if (selected >= cameras.size()) selected = 0;
            switcher.setSource(cameras.get(selected));
        }
    }

    private UsbCamera getUSBCamera(int id) {
        var camera = CameraServer.getInstance().startAutomaticCapture(String.format("%sSwitcher@USB%d", name, id), id);
        camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
        camera.setFPS(60);
        camera.setResolution(320, 240);

        return camera;
    }


}
