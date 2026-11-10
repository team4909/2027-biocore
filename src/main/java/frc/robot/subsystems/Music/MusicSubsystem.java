package frc.robot.subsystems.Music;


import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Random;

import java.util.List;

public class MusicSubsystem extends SubsystemBase {
    private final Random randomSongSelector = new Random();
    private final Orchestra orchestra;
    private final List<TalonFX> motors;

    public MusicSubsystem(List<TalonFX> motors) {
        this.motors = motors;
        this.orchestra = new Orchestra();
        for (TalonFX motor : motors) {
            orchestra.addInstrument(motor);
        }
    }
    public void playRandomSong() {
        String[] songs = {"dieconunsmile.chrp", "espresso.chrp", "mario.chrp", "rickroll.chrp", "staychrp.chrp"}; // Add your song file names here
        int randomIndex = randomSongSelector.nextInt(songs.length); // Generate a random index
        String selectedSong = songs[randomIndex]; // Select a random song from the array
        playMusic(selectedSong);
    }

    public void playMusic(String filePath) {
        orchestra.loadMusic(filePath);
        orchestra.play();

    }

    public void stopMusic() {
        orchestra.stop();
    }

    public boolean isPlaying() {
        return orchestra.isPlaying();
    }

    @Override
    public void periodic() {
    }
}

