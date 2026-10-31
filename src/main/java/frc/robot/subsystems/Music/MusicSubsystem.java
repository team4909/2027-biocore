package frc.robot.subsystems.Music;


import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    private final Orchestra orchestra;
    private final List<TalonFX> motors;

    public MusicSubsystem(List<TalonFX> motors) {
        this.motors = motors;
        this.orchestra = new Orchestra();
        for (TalonFX motor : motors) {
            orchestra.addInstrument(motor);
        }
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

