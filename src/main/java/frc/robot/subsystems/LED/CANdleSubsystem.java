package frc.robot.subsystems.LED;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.DoubleSupplier;

public class CANdleSubsystem extends SubsystemBase {
    private CANdle candle;
    private final int totalLEDs = 45;//+ 8 for the candle //SRIHAAS!!! CHANGE THIS TO THE NUMBER OF LEDS YOU HAVE! (Made it a multiple of 4)
    private final int DisabledSectionLength = totalLEDs / 4; //Chris said 4 sections is fine

    // Pull drivetrain speeds
    // ChassisSpeeds speeds = m_drivetrain.getState().Speeds;
    // double vx = speed.vxMetersPerSecond;
    // double vy = speed.vyMetersPerSecond;
    // double omega = speed.omegaRadiansPerSecond;

    public enum LEDmode {
        DISABLED_SWEEP,
        AUTO,
        TELEOP,
        FAULT_OVERRIDE,
    }

    private boolean canBusHealthy = true;
    private boolean batteryHealthy = true;
    private boolean autoReady = false;
    private final boolean[] cameraHasTargets = new boolean[]{false, false};
    private final DoubleSupplier speedSupplier;

    public CANdleSubsystem(int canID, DoubleSupplier speedSupplier) {
        candle = new CANdle(0);
        this.speedSupplier = speedSupplier;

        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.BrightnessScalar = 0.7; 
        candle.getConfigurator().apply(config);
    }

    public void setTeleopZone(int ZoneIndex, int r, int g, int b) {
        if (ZoneIndex < 0 || ZoneIndex >= 2) return;

        int teleopSectionLength = totalLEDs / 2;
        int startIndex = 8 + ZoneIndex * teleopSectionLength;
        int endIndex = startIndex + teleopSectionLength - 1;

        candle.setControl(new SolidColor(startIndex, endIndex).withColor(new RGBWColor(r, g, b, 0)));
    }

    public void setAutoZone(int AutoZoneIndex, int r, int g, int b) {
        if (AutoZoneIndex < 0 || AutoZoneIndex >= 2) return;

        int AutoSectionLength = totalLEDs / 2;
        int startIndex = 8 + AutoZoneIndex * AutoSectionLength;
        int endIndex = startIndex + AutoSectionLength - 1;

        candle.setControl(new SolidColor(startIndex, endIndex).withColor(new RGBWColor(r, g, b, 0)));
    }

    public void setDisabledZone(int DisabledZoneIndex, int r, int g, int b) {
        
        if (DisabledZoneIndex < 0 || DisabledZoneIndex >= 4) return;

        int startIndex = 8 + DisabledZoneIndex * DisabledSectionLength;
        int endIndex = startIndex + DisabledSectionLength - 1;
        candle.setControl(new SolidColor(startIndex, endIndex).withColor(new RGBWColor(r, g, b, 0)));
    }

    public void setSolidColor(int r, int g, int b) {
        candle.setControl(
            new SolidColor(8, 8 + totalLEDs - 1)                
            .withColor(new RGBWColor(r, g, b, 0))
        );
    }

    public void clearLEDs() {
        setSolidColor(0, 0, 0);
    }
    
    @Override
    public void periodic() {
        batteryHealthy = RobotController.getBatteryVoltage() >= 10.0;

        if (DriverStation.isDisabled()) {
            runDisabledLEDs();
        } else if (DriverStation.isAutonomous()) {
            runAutoLEDs();
        } else {
            runTeleopLEDs();
        }
    }

    //Disabled functions
    private void runDisabledLEDs() {
        // Section 1: Amber (Robot / CAN Health)
        if (canBusHealthy) {
            setDisabledZone(0, 255, 191, 0); // Amber
        } else {
            setDisabledZone(0, 255, 0, 0);   // Red fault
        }

        // Section 2: Cyan (Battery Voltage)
        if (batteryHealthy) {
            setDisabledZone(1, 0, 255, 255); // Cyan
        } else {
            setDisabledZone(1, 255, 0, 0);   // Red fault
        }

        // Section 3: Blue (Auto readiness)
        if (autoReady) {
            setDisabledZone(2, 0, 0, 255);   // Blue
        } else {
            setDisabledZone(2, 0, 0, 100);   // Dim blue waiting
        }

        // Section 4: Purple (Vision Cameras)
        if (cameraHasTargets[0] || cameraHasTargets[1]) {
            setDisabledZone(3, 128, 0, 128); // Purple
        } else {
            setDisabledZone(3, 30, 0, 30);   // Dim purple waiting for tags
        }
    }

    //Autonomous functions
    private void runAutoLEDs() {
        //Fault overide: Red, if CAN bus or battery is not healthy, override all other LED modes
        if (!canBusHealthy || !batteryHealthy) {
            setSolidColor(255, 0, 0); // Red fault
            return;
        }

        //Zone 0 out of 2
        double currentSpeed = speedSupplier.getAsDouble();

        //Map 0-5 MPS to a RGB value
        double zoneOneGreenColor = 50.0 + (currentSpeed / 5.0) * (255.0 - 50.0);
        //Get the final color
        int green = (int) Math.round(
            Math.max(0.0, Math.min(255.0, zoneOneGreenColor))
        );
        
        setAutoZone(0, 0, green,0 );

        //Zone 1 out of 2
        int activeCameras = 0;

        //Check how many cameras have targets
        for (boolean hasTarget : cameraHasTargets) {
            if (hasTarget) activeCameras++;
        }

        if (activeCameras >= 2) {
            setAutoZone(1, 232, 0, 232); // Bright Purple, 2 tags
        } else if (activeCameras == 1) {
            setAutoZone(1, 128, 0, 128); // Purple, 1 tag
        }
        else {
            setAutoZone(1, 30, 0, 30);   // Dim purple waiting for tags
        }
    }

    //Teleop functions
    private void runTeleopLEDs() {
        //Fault overide: Red, if CAN bus or battery is not healthy, override all other LED modes
        if (!canBusHealthy || !batteryHealthy) {
            setSolidColor(255, 0, 0); // Red fault
            return;
        }

        //Zone 0 out of 2
        double currentSpeed = speedSupplier.getAsDouble();

        //Map 0-5 MPS to a RGB value
        double zoneOneGreenColor = 50.0 + (currentSpeed / 5.0) * (255.0 - 50.0);
        //Get the final color
        int green = (int) Math.round(
            Math.max(0.0, Math.min(255.0, zoneOneGreenColor))
        );

        setTeleopZone(0, 0, green,0 );

        //Zone 1 out of 2
        int activeCameras = 0;

        //Check how many cameras have targets
        for (boolean hasTarget : cameraHasTargets) {
            if (hasTarget) activeCameras++;
        }

        if (activeCameras >= 2) {
            setTeleopZone(1, 232, 0, 232); // Bright Purple, 2 tags
        } else if (activeCameras == 1) {
            setTeleopZone(1, 128, 0, 128); // Purple, 1 tag
        }
        else {
            setTeleopZone(1, 30, 0, 30);   // Dim purple waiting for tags
        }
    }

    public void setCanBusHealthy(boolean healthy) { this.canBusHealthy = healthy; }
    public void setAutoReady(boolean ready) { this.autoReady = ready; }
    public void setCameraTarget(int cameraIdx, boolean hasTarget) {
        if (cameraIdx >= 0 && cameraIdx < cameraHasTargets.length) {
            this.cameraHasTargets[cameraIdx] = hasTarget;
        }
    }
}