package frc.robot.subsystems.LED;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.SolidColor;

public class CANdleSubsystem extends SubsystemBase {
    private final CANdle candle;
    private int totalLEDs = 70; //SRIHAAS!!! CHANGE THIS TO THE NUMBER OF LEDS YOU HAVE!
    private final int sectionLength = totalLEDs / 4; //Chris said 4 sections is fine

    public enum LEDmode {
        DISABLED_SWEEP,
        AUTO,
        TELEOP,
        FAULT_OVERRIDE,
    }

    private boolean canBusHealthy = true;
    private boolean batteryHealthy = true;
    private boolean autoReady = false;
    private boolean[] cameraHasTargets = new boolean[]{false, false, false, false};

    public CANdleSubsystem(int canID) {
        candle = new CANdle(canID);

        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.BrightnessScalar = 0.7; 
        candle.getConfigurator().apply(config);
    }
    public void setSectionColor(int sectionIndex, int r, int g, int b) {
        if (sectionIndex < 0 || sectionIndex >= 4) return;
        int startIndex = sectionIndex * sectionLength;
        int endIndex = startIndex + sectionLength - 1;
        candle.setControl(new SolidColor(startIndex, endIndex).withColor(new RGBWColor(r, g, b, 0)));
    }

    public void setSolidColor(int r, int g, int b) {
        candle.setControl(new SolidColor(0, totalLEDs - 1).withColor(new RGBWColor(r, g, b, 0)));
    }

    public void clearLEDs() {
        candle.setControl(new SolidColor(0, totalLEDs - 1).withColor(new RGBWColor(0, 0, 0, 0)));
    }
    
    @Override
    public void periodic() {
        batteryHealthy = RobotController.getBatteryVoltage() >= 12.0;

        if (DriverStation.isDisabled()) {
            runPreMatchSweep();
        } else if (DriverStation.isAutonomous()) {
            runAutoLEDs();
        } else {
            runTeleopLEDs();
        }
    }
    private void runPreMatchSweep() {
        // Section 1: Amber (Robot / CAN Health)
        if (canBusHealthy) {
            setSectionColor(0, 255, 191, 0); // Amber
        } else {
            setSectionColor(0, 255, 0, 0);   // Red fault
        }

        // Section 2: Cyan (Battery Voltage)
        if (batteryHealthy) {
            setSectionColor(1, 0, 255, 255); // Cyan
        } else {
            setSectionColor(1, 255, 0, 0);   // Red fault
        }

        // Section 3: Blue (Auto readiness)
        if (autoReady) {
            setSectionColor(2, 0, 0, 255);   // Blue
        } else {
            setSectionColor(2, 0, 0, 100);   // Dim blue waiting
        }

        // Section 4: Purple (Vision Cameras)
        if (cameraHasTargets[0] || cameraHasTargets[1]) {
            setSectionColor(3, 128, 0, 128); // Purple
        } else {
            setSectionColor(3, 30, 0, 30);   // Dim purple waiting for tags
        }
    }

    private void runAutoLEDs() {
        // Auto logic: Tag visibility across 4 sections
    }

    private void runTeleopLEDs() {
        // Teleop logic: Game piece status / ready to score
    }

    public void setCanBusHealthy(boolean healthy) { this.canBusHealthy = healthy; }
    public void setAutoReady(boolean ready) { this.autoReady = ready; }
    public void setCameraTarget(int cameraIdx, boolean hasTarget) {
        if (cameraIdx >= 0 && cameraIdx < 4) {
            this.cameraHasTargets[cameraIdx] = hasTarget;
        }
    }
}