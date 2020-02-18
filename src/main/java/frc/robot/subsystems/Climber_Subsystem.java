/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.PWMVictorSPX;
//import com.ctre.phoenix.motorcontrol.can.PWMVictorSPX;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber_Subsystem extends SubsystemBase {
  
    private final PWMVictorSPX liftWinch = new PWMVictorSPX(Constants.VICTORSPX_WINCH);
    private final PWMVictorSPX hookReel = new PWMVictorSPX(Constants.VICTORSPX_HOOK_LIFT);

    /**
     * Creates a new Climber_Subsystem.
     */
    public Climber_Subsystem() {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void Move_Hook(double motor_speed) {
        hookReel.set(1);

    }
    
    public void Climb_Up(){
        liftWinch.set(.5);

    }

    public void Climb_Down(){
        liftWinch.set(-.5);
    }

    public void Stop_Climb(){
        liftWinch.set(0);
       // hookReel.set(ControlMode.PercentOutput, 0.0);
    }
}
