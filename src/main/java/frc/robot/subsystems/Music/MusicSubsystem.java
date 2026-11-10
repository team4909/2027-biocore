package frc.robot.subsystems.Music;


import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Random;

import java.util.List;

public class MusicSubsystem extends SubsystemBase {
    //Random is an object so you have to instantiate it.
    private final Random randomSongSelector = new Random();

    private final Orchestra orchestra;
    private final List<TalonFX> motors;

    //Constructed by RobotContainer, which passes in the motors to be used for music.
    public MusicSubsystem(List<TalonFX> motors) {
        this.motors = motors;
        //Orchestra is an object that lets you play music.
        //you have to add an instrument and pass in a motor or anything
        //else that can play music.
        this.orchestra = new Orchestra();
        //here we go through every motor and add it to
        //the orchestra so it can use it to play music.
        for (TalonFX motor : motors) {
            orchestra.addInstrument(motor);
        }
    }

    //called by robot container when pressing
    public void playRandomSong() {
        //interesting way to play music.
        //a better way to do this could be to
        //loop through a folder with the songs and play a random one from there.
        //instead of hardcoding the songs in the code.
        String[] songs = {"dieconunsmile.chrp", "espresso.chrp", "mario.chrp", "rickroll.chrp", "staychrp.chrp"}; // Add your song file names here
        int randomIndex = randomSongSelector.nextInt(songs.length); // Generate a random index
        String selectedSong = songs[randomIndex]; // Select a random song from the array
        playMusic(selectedSong);
    }

    public void playMusic(String filePath) {
        //uses orchestra

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

