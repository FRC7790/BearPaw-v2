/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.pseudoresonance.pixy2api.*;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

import java.util.ArrayList;


public class PixyCam_Subsystem extends SubsystemBase {

  public Pixy2 pixycam;
  boolean isCamera = false;
  //private SPILink spi;
  int state=- 1;
  /**
   * Creates a new ExampleSubsystem.
   */
  public PixyCam_Subsystem() {

    pixycam = Pixy2.createInstance(Pixy2.LinkType.SPI);
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(!isCamera)
      state = pixycam.init(1); // if no camera present, try to initialize
    isCamera = state>=0;
    
    SmartDashboard.putBoolean("Camera", isCamera);   //publish if we are connected
    pixycam.getCCC().getBlocks(false,255,255); //run getBlocks with arguments to have the camera
                                               //acquire target data
    ArrayList<Block> blocks = pixycam.getCCC().getBlocks(); //assign the data to an ArrayList for convinience
    if(blocks.size() > 0)
    {
      double xcoord = blocks.get(0).getX();       // x position of the largest target
      double ycoord = blocks.get(0).getY();       // y position of the largest target
      String data   = blocks.get(0).toString();   // string containing target info
      SmartDashboard.putBoolean("present", true); // show there is a target present
      SmartDashboard.putNumber("Xccord",xcoord);
      SmartDashboard.putNumber("Ycoord", ycoord);
      SmartDashboard.putString("Data", data );
    }
    else
      SmartDashboard.putBoolean("present", false);
    SmartDashboard.putNumber("size", blocks.size()); //push to dashboard how many targets are detected
  }
  
}
