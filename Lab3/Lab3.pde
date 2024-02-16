 /**
 **********************************************************************************************************************
 * @file       Haptic_Physics_Template.pde
 * @author     Steve Ding, Colin Gallacher
 * @version    V3.0.0
 * @date       27-September-2018
 * @brief      Base project template for use with pantograph 2-DOF device and 2-D physics engine
 *             creates a blank world ready for creation
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */
 
 
 
 /* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  



/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 3;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           rendering_force                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           pos_ee                              = new PVector(0, 0);
PVector           f_ee                                = new PVector(0, 0); 

/* World boundaries */
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 10.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

/* Initialization of virtual tool */
HVirtualCoupling  s;

//common vars
PFont f;
int scene = 1;

//scene 1 vars
FBox floor;
FBox weight;

//scene 2 vars
float x;
float y;
ArrayList<FContact> contactPoint;
boolean touched = false;
float dist=0.0;
/* end elements definition *********************************************************************************************/

//scene 3 vars
FCircle c1;
FCircle c2;
FCircle c3;
FCircle c4;
FCircle c5;
FLine l1;
FLine l2;
FLine l3;
FLine l4;


/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 400);
  
  
  /* device setup */
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */
  haplyBoard          = new Board(this, Serial.list()[2], 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  println(Serial.list());
  
  widgetOne.set_mechanism(pantograph);
  
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);

  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);

  
  widgetOne.device_set_parameters();
  
  
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
  f                   = createFont("Arial", 16, true);
  
  /* Wall Setup */
  floor = new FBox(16,2);
  floor.setPosition(12, 6);
  floor.setNoFill();
  floor.setNoStroke();
  //floor.setFill(255,0,0);
  floor.setStatic(true);
  //world.add(floor);
  
  weight = new FBox(10,2);
  weight.setPosition(12,3);
  //weight.setForce(0,1000);
  weight.setSensor(true);
  weight.setDensity(1000);
  weight.setNoFill();
  weight.setNoStroke();
  //weight.setFill(0,200,0);
  
 
  c1 = new FCircle(4);
  //c1.setFill(0,255,0);
  c1.setNoFill();
  c1.setNoStroke();
  c1.setStatic(true);
  c1.setPosition(5,5);
  
  c2 = new FCircle(4);
  c2.setNoFill();
  c2.setNoStroke();
  //c2.setFill(0,255,0);
  c2.setStatic(true);
  c2.setPosition(10,5);
  
  l1 = new FLine(6,3.7,7.5,4);
  l1.setFill(255,255,0);
  l2 = new FLine(7.5,4,9,3.7);
  l2.setFill(255,255,0);
  
  c3 = new FCircle(4);
  //c3.setFill(0,255,0);
  c3.setNoFill();
  c3.setNoStroke();
  c3.setStatic(true);
  c3.setPosition(15,5);
  
  l3 = new FLine(11,3.7, 12.5,4);
  l3.setFill(255,255,0);
  l4 = new FLine(12.5,4,14,3.7);
  l4.setFill(255,255,0);
  
  c4 = new FCircle(4);
  c4.setNoFill();
  c4.setNoStroke();
  //c4.setFill(0,255,0);
  c4.setStatic(true);
  c4.setPosition(16,5);
  
  c5 = new FCircle(4);
  c5.setNoFill();
  c5.setNoStroke();
  //c5.setFill(0,255,0);
  c5.setStatic(true);
  c5.setPosition(18,5);
  
  
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((1)); 
  s.h_avatar.setDensity(2); 
  s.h_avatar.setFill(255,200,0); 
  
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  
  /* World conditions setup */
  world.setGravity((0.0), (0.0)); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  
  world.draw();
  
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  
  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}



/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(rendering_force == false){
    background(0,0,50);
    textFont(f, 80);
    //textSize(13);
    fill(255, 0, 0);
    textAlign(CENTER);
    text("Word: "+Integer.toString(scene), width/2, height/2); 
    
    world.draw();
  }
   
}
/* end draw section ****************************************************************************************************/

void keyPressed(){
  if (key=='1'){
    scene=1;
    world.add(floor);
    world.add(weight);
  
    //print("1");
    //world.add(w1);
  }
  if (key=='2'){
    scene=2;
    world.remove(weight);
    //floor.setFill(0,0,200);
  }
  if (key=='3'){
    scene=3;
    world.add(c1);
    world.add(c2);
    world.add(c3);
    //world.add(c4);
    //world.add(c5);
    world.add(l1);
    world.add(l2);
    world.add(l3);
    world.add(l4);
    //print("3");
  }
  
}

/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    rendering_force = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
      angles.set(widgetOne.get_device_angles()); 
      pos_ee.set(widgetOne.get_device_position(angles.array()));
      //if(scene == 3){
        //pos_ee.set(device_to_graphics(pos_ee)); 
      //}
      //else{
      //}
      pos_ee.set(pos_ee.copy().mult(200)); 
      s.setToolPosition(edgeTopLeftX+worldWidth/2-(pos_ee).x+2, edgeTopLeftY+(pos_ee).y-7); 
      s.updateCouplingForce();
      f_ee.set(-s.getVCforceX(), s.getVCforceY());
      f_ee.div(20000); //
    }
    
    
    
    torques.set(widgetOne.set_device_torques(f_ee.array()));
    widgetOne.device_write_torques();
    
    if(scene == 1){
      if(s.h_avatar.isTouchingBody(floor)){
        weight.setSensor(false);
        //s.setAvatarVelocity(0,-10);
        //world.add(weight);
      }
    }
    if(scene == 2){
      //world.clear();
      //weight.removeFromWorld();
      if(s.h_avatar.isTouchingBody(floor)){
        contactPoint = floor.getContacts();
        x = contactPoint.get(0).getX();
        y = contactPoint.get(0).getY();
        touched = true;
        //floor.setVelocity(20,0);
        //s.h_avatar.setDamping(700);
      }  
      if(touched){
        dist = Math.abs(x-s.h_avatar.getX())+Math.abs(y-s.h_avatar.getY());
        println(dist);
        if(dist<=6){
          s.h_avatar.setDamping(166*dist);
        }
        else{
          
          //s.setToolVelocity(10,0);
          //s.setToolPosition(width/2, height/2);
          s.h_avatar.setDamping(0.0);
          touched = false;
        }
      }
    }
    if(scene==3){
      world.remove(floor);
      world.remove(weight);
    }
   
    world.step(1.0f/1000.0f);
  
    rendering_force = false;
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/
PVector device_to_graphics(PVector deviceFrame){
  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
}

PVector graphics_to_device(PVector graphicsFrame){
  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
}
/* end helper functions section ****************************************************************************************/
