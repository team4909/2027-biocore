package frc.robot.subsystems.Music;


import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Random;

import java.util.List;

    //********************************************************************************//
    // This was kind of a side project, but I wanted to see if I could get            //
    // the music subsystem working. It is a fun little addition to the robot, and     //
    //  it is a good way to test the capabilities of the swerve drive platform.       //
    //  Also, telling people that your motors can play music is a good way to get     //
    //  them interested in the robot.                                                 //
    //  Who knows, maybe it will even help us win some awards at competitions.        //
    //  or at least recruit some new members to the team.                             //
    //  We always need more members.                                                  //
    //  In hind sight, I probably should have put this in a feature branch,           //
    //  but who cares?                                                                //
    //  Here comes: THE MUSIC SUBSYSTEM!                                              //
    //********************************************************************************//

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
            orchestra.addInstrument(motor, 0);
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

