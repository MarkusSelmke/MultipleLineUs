/*
  -----------------------------------------------------------------------------------------------
  
  Digital Twin of a 4 devices Line-Us Setup (Markus Selmke, markus.selmke@gmx.de)
  Includes keyboard interface (hit 'h' for help) and collisions detection by a bounding box method (rectangles) 
  
  -----------------------------------------------------------------------------------------------
  
  Line-Us Processing Interface by Robert Poll (class LineUs)
  (https://github.com/Line-us/Line-us-Programming/blob/master/Processing/HelloWorld/HelloWorld.pde#L1)
  
  Very simple example of how to use the Line-us API. The most important thing to remember is to read the response after sending a command. 
  The TCP buffer on Line-us is small and it does not cope well with having to buffer more than one message.
  
  -----------------------------------------------------------------------------------------------
  
  Drawing area:
  https://github.com/Line-us/Line-us-Programming/blob/master/Documentation/LineUsDrawingArea.pdf
  https://github.com/Line-us/Line-us-Programming/blob/master/README.md#getting-started-with-line-us-programming
  x in 650 ... 1775, y in -1000, ..., 1000, z in 0 (down), ..., 1000 (up), home: (1000, 1000, 1000) (rectangular configuration and pen up)
     
  (new) GCode commands:
  https://github.com/Line-us/Line-us-Programming/blob/master/Documentation/GCodeSpec.md
  retrieve servo angles E, S & P: "M114 S" -> ok S:135.00 E:45.00 P:135.00
  
  -----------------------------------------------------------------------------------------------
*/

import processing.net.*;
 
PImage img_help;                               //helper screen image
boolean HelpActive = false;                    //helper screen toggle variable

boolean keyCONTROL = false;                    //boolean variable signifying whether CNTRL is pressed or not.
boolean keyC = false;                          //boolean variable signifying if key 'c' is currently pressed.
boolean keyT = false;                          //boolean variable signifying whether 't' is pressed or not.

//define array from class "LineUs" defined below (W-LAN interface to 4 devices. Connect via key 'c')
LineUs[] LineUsDevice = new LineUs[4];                     //create 4 instances of Line-Us devices (Rob's interface class) 
boolean[] LineUsConnected = {false, false, false, false};  //boolean variables signifying if LineUs Devices are connected

//create array of digital twin instances from class "LineUsDigital"
LineUsDigital[] LineUs = new LineUsDigital[4]; //create 4 instances of Line-Us digital twin objects
PVector SetupCenter = new PVector();           //center of the Setup (Line-Us arranged around this)
PVector[] LineUsPositions = new PVector[4];    //positions of the Line-Us Devices (4 devices, 4 vectors A)
PVector[] LineUsEs = new PVector[4];           //positions of the Line-Us Devices (4 devices, 4 vectors E)
PVector[] LineUsFs = new PVector[4];           //positions of the Line-Us Devices (4 devices, 4 vectors F)
PVector[] LineUsELastValid = new PVector[4];   //stored last valid E positions (4 devices, 4 vectors E)
PVector zVec = new PVector(0, 0, 1);           //used for vector maths, e.g. cross-products to find normals
float[] wi = {10, 10, 10, 10, 5, 22.5, 22, 10};//half width of bounding boxes around joint-connection-vectors
int[] bbv = {4, 4, 4, 4, 4, 6, 4, 6};          //# of vertices per bounding box
boolean ToggleBBCD = true;                     //bounding box collision detection                      
float scale = 2;                               //scale used for plotting. i.e. [px] = [mm]*scale
float a=50.0;                                  //arm length a [mm] (longer one)
float b=32.5;                                  //arm length b [mm] (shorter one)
float e=40.0;                                  //arm length e [mm] (extension rod)
float gamma=0*PI/4;                            //extension arm mounting angle relative to pen-arm (0 = straight extension) -> point F
float LineUsLength=96;                         //Line-Us device length [mm]
float LineUsWidth=25;                          //Line-Us device length [mm]
float PondSize=200;                            //operating area size [mm]
float PondWidth=5;                             //width of boundary drawn around operating area [mm]
float PlateSize=265;                           //area size of plate mounting the Line-Us devices [mm]
float PondSaftyWidth=10;                       //safety zone around canvas border [mm]
float dx=0.1;                                  //size of fine-movements (arrow keys) [px]
float CircRadius = 10;                         //radius of draggable disc [px]
float JointRadius = 5;                         //radius of displayed Line-Us joints [px]
color BBDefaulColor = color(100, 100);         //(default) color of bounding boxes (no collision)
color BBIntersectColor = color(255,0,0, 100);  //color of bounding boxes when collision is detected
color ActiveEColor = color(255,255,0, 150);    //color of dragging disc which is active (depends on dragging mode)
color DefaultEColor = color(0,0,255, 50);      //default color of dragging discs (i.e. when not active)
color FColor = color(255, 255, 0);             //color of extension arm points F (to be controlled)
PFont myFont;                                  //default font, also in table
PFont myFontBold;                              //font for table headers
int ActiveLineUs = 0;                          //default / intially active LineUs
int DragMode = 0;                              //drag mode: 0: drag pen holder position (E, default), 1: drag extension point F, 2: drag/move a

Table PosTable;                                //table for importing and storing F vectors
boolean FileLoaded = false;                    //boolean variable storing if file has been loaded or not
int ConfID = 0;                                //index of active table row, i.e. current configuration if table is loaded
float ConfPreviewScale = 0.25;                 //scale of the small preview of the read-in tables of configurations for the F's
float ConfScale = 1.0;                         //scale factor for configurations
boolean GoToActive = false;                    //boolean variable signifying state of moving towards target F
PVector[] FStart = new PVector[4];             //array of vectors holding the start positions for a given movement of F's
PVector[] FToTargetF = new PVector[4];         //array of vectors holding the directions to target F for a given movement
int Goj = 1;                                   //iteration integer variable for moving F's
int[] FNjMax = {0, 0, 0, 0};                   //jMax, maximum # of iterations needed to (almost) finish move to target F's
float ds = 2.5;                                //step size in pixels (mm would be ds/scale)
int DefaultDelay = 0;                          //delay after each device move in [ms]

boolean[] TippingActive = {false, false, false, false}; //tipping mode: moves z up and down
int nzi = 0;                                   //integer z-coordinate in devices' coordinate system
float dnzi = 0.05;                             //z-step-size in sin() for tipping (together with frame rate this determines the tipping frequency)
float ziAmp = 400;                             //z-Amplitude of tipping, must be <1000

void setup() {
  size(1200,600);                              //set size of canvas, variables width, height;
  frameRate(25);

  img_help = loadImage("Help.png");            //load helper screen image
  
  //set digital twin drawing area center, i.e. global common coordinate center
  SetupCenter.x = height/2;
  SetupCenter.y = height/2;
  
  //load F-positions [mm] from file "positions.csv"
  PosTable = loadTable("positions.csv", "header, csv");
  if (PosTable != null) {
    FileLoaded = true;
  }
  if (FileLoaded==true) {
    println(PosTable.getRowCount() + " total rows (configurations) in table");
    println(PosTable.getColumnCount() + " total columns in table (should be 8, 2 coords x,y per 1 out of 4 devices)");
  }
  
  myFont =createFont("Arial", 12, true);
  myFontBold = createFont("Arial Bold", 12, true);
  
  //Scene Setup: Line-Us default to some device positions (A)
  LineUsPositions[0] = new PVector(SetupCenter.x - 0.25*(PondSize + PlateSize + 2.0*PondWidth)*scale, SetupCenter.y, 0);  //Line-Us 1 position
  LineUsPositions[1] = new PVector(SetupCenter.x, SetupCenter.y - 0.25*(PondSize + PlateSize + 2.0*PondWidth)*scale, 0);    //Line-Us 2 position
  LineUsPositions[2] = new PVector(SetupCenter.x + 0.25*(PondSize + PlateSize + 2.0*PondWidth)*scale, SetupCenter.y, 0);  //Line-Us 1 position
  LineUsPositions[3] = new PVector(SetupCenter.x, SetupCenter.y + 0.25*(PondSize + PlateSize + 2.0*PondWidth)*scale, 0);    //Line-Us 1 position
  
  //Scene setup: Initial pen positions (E) & initial target for extension arms (Fs)
  PrepareEs();
  PrepareFs();
  
  //create Digital Twins of the Line-Us devices used for collision detection. Sets IDs, As, Es and device orientations initially
  LineUs[0] = new LineUsDigital(0, LineUsPositions[0], LineUsEs[0], radians(0)); //0, 0
  LineUs[1] = new LineUsDigital(1, LineUsPositions[1], LineUsEs[1], radians(90)); //1, 90
  LineUs[2] = new LineUsDigital(2, LineUsPositions[2], LineUsEs[2], radians(180)); //2, 180
  LineUs[3] = new LineUsDigital(3, LineUsPositions[3], LineUsEs[3], radians(270)); //3, 270
  
  //initially: active Line-Us is Line-Us ID=1
  ToggleLineUsActive(ActiveLineUs);
}

void PrepareEs() {
  //use default position initialization, somewhere reasonable & safe on the canvas
  LineUsEs[0] = new PVector(SetupCenter.x - 0.25*PondSize*scale, SetupCenter.y, 0); //initial Device 1 pen holder coords
  LineUsEs[1] = new PVector(SetupCenter.x, SetupCenter.y - 0.25*PondSize*scale, 0); //initial Device 2 pen holder coords
  LineUsEs[2] = new PVector(SetupCenter.x + 0.25*PondSize*scale, SetupCenter.y, 0); //initial Device 3 pen holder coords
  LineUsEs[3] = new PVector(SetupCenter.x, SetupCenter.y + 0.25*PondSize*scale, 0); //initial Device 4 pen holder coords
  
  LineUsELastValid[0] = new PVector(SetupCenter.x - 0.25*PondSize*scale, SetupCenter.y, 0); //initial Device 1 pen holder coords
  LineUsELastValid[1] = new PVector(SetupCenter.x, SetupCenter.y - 0.25*PondSize*scale, 0); //initial Device 2 pen holder coords
  LineUsELastValid[2] = new PVector(SetupCenter.x + 0.25*PondSize*scale, SetupCenter.y, 0); //initial Device 3 pen holder coords
  LineUsELastValid[3] = new PVector(SetupCenter.x, SetupCenter.y + 0.25*PondSize*scale, 0); //initial Device 4 pen holder coords
}

//move digital twins to E's stored in LineUsEs[] array
void SetEs() {
  for(int i=0; i<LineUs.length; i++) {
    LineUs[i].setLineUsE(LineUsEs[i]);
  } 
}

//set digital twins target F's. Reads four F's from configuration file, row: ConfID
void PrepareFs() {
  if (FileLoaded==true) { //use positions.cvs and current Conf. ID (default: 0)
    for (int i=0; i<4; i++) {
      LineUsFs[i] = new PVector(SetupCenter.x + ConfScale*scale*PosTable.getFloat(ConfID, 2*i), SetupCenter.y + ConfScale*scale*PosTable.getFloat(ConfID, 2*i+1), 0);
    }
  } else { //some resonable default values
    LineUsFs[0] = new PVector(SetupCenter.x - 0.25*PondSize*scale, SetupCenter.y, 0); //initial Device 1 F coords
    LineUsFs[1] = new PVector(SetupCenter.x, SetupCenter.y - 0.25*PondSize*scale, 0); //initial Device 2 F coords
    LineUsFs[2] = new PVector(SetupCenter.x + 0.25*PondSize*scale, SetupCenter.y, 0); //initial Device 3 F coords
    LineUsFs[3] = new PVector(SetupCenter.x, SetupCenter.y + 0.25*PondSize*scale, 0); //initial Device 4 F coords
  }
}

void draw() {
  background(255);
  
  //create grid lines in the backgorund (10mm spacing)
  stroke(240);
  for(float x=-height/(2*scale); x<0.95*height/(2*scale); x=x+10){
    line(SetupCenter.x + x*scale, 0, SetupCenter.x + x*scale, height);                            //vertical lines
    line(0, SetupCenter.y + x*scale, SetupCenter.x + 0.95*height/2, SetupCenter.y + x*scale);     //horizontal lines
  }
  strokeWeight(3); line(0,SetupCenter.y, SetupCenter.x + height/2,SetupCenter.y); line(SetupCenter.x,0, SetupCenter.x,height);
  
  //display ds & dx
  fill(0);
  textAlign(LEFT, CENTER); text("move step size: " + nf(ds,1,0), 10, 10);
  textAlign(LEFT, CENTER); text("step size: " + nf(dx,1,1), 10, 25);
  
  //draw pond / canvas / mounting plate
  strokeWeight(1); stroke(0); fill(color(0,155,255,50)); rectMode(CENTER);
  rect(SetupCenter.x, SetupCenter.y, PondSize*scale, PondSize*scale);
  rect(SetupCenter.x, SetupCenter.y, (PondSize+2*PondWidth)*scale, (PondSize+2*PondWidth)*scale);
  rect(SetupCenter.x, SetupCenter.y, PlateSize*scale, PlateSize*scale);
  //draw saftey zone inside pond / canvas area
  rectMode(CORNER); fill(BBDefaulColor); strokeWeight(0);
  rect(SetupCenter.x - 0.5*PondSize*scale, SetupCenter.y - 0.5*PondSize*scale, PondSaftyWidth*scale, PondSize*scale);
  rect(SetupCenter.x + (0.5*PondSize - PondSaftyWidth)*scale, SetupCenter.y - 0.5*PondSize*scale, PondSaftyWidth*scale, PondSize*scale);
  rect(SetupCenter.x - (0.5*PondSize - PondSaftyWidth)*scale, SetupCenter.y - 0.5*PondSize*scale, (PondSize - 2*PondSaftyWidth)*scale, PondSaftyWidth*scale);
  rect(SetupCenter.x - (0.5*PondSize - PondSaftyWidth)*scale, SetupCenter.y + (0.5*PondSize - PondSaftyWidth)*scale, (PondSize - 2*PondSaftyWidth)*scale, PondSaftyWidth*scale);
  rectMode(CENTER);
  
  //draw global coord. system in upper left corner
  strokeWeight(1);
  stroke(265,0,0); //local x-axis is red
  line(0, 1, 0 + 50, 1);
  stroke(0,0,255); //local y-axis is blue
  line(1, 0, 1, 0 + 50);
  
  //draw global coord. system centered on S
  stroke(265,0,0); //local x-axis is red
  line(SetupCenter.x, SetupCenter.y, SetupCenter.x + 50, SetupCenter.y);
  stroke(0,0,255); //local y-axis is blue
  line(SetupCenter.x, SetupCenter.y, SetupCenter.x, SetupCenter.y + 50);
  
  //update & draw all Line-Us devices
  for(int i=0; i<LineUs.length; i++){
    LineUs[i].update();
    LineUs[i].display();
  }
  
  //do iterative steps towards target Fs stored in LineUsFs[] array when in move mode. 
  //This also moves real devices via .g01 if connected (in subroutine setLineUsE() called from setLineUsF())
  if (GoToActive == true) {
    for(int i=0; i<LineUs.length; i++) {                             //go through all Line-Us devices (digital twins)
      if (Goj==1) {                                                  //beginning of movement towards target
        FStart[i] = LineUs[i].getLineUsF();                          //store start F of i-th Line-Us device (digital twin)
        FNjMax[i] = ceil(PVector.dist(FStart[i], LineUsFs[i])/ds);   //# of segments movement is partitioned into for i-th Line-Us
        FToTargetF[i] = PVector.sub(LineUsFs[i], FStart[i]);         //unit vector of Start F to target F
        FToTargetF[i].normalize();
      } 
      if (Goj < FNjMax[i]) {                                         //iterative (jth) step in move towards target F
          PVector Fj = PVector.add(FStart[i], PVector.mult(FToTargetF[i], ds*Goj)); //j-th target F
          LineUs[i].setLineUsF(Fj);
      } else if (Goj == FNjMax[i]) {                                 //final move
          LineUs[i].setLineUsF(LineUsFs[i]);
      }
    }
    Goj++;                                                           //increase iterator integer for next draw() call/step
    if (Goj == max(FNjMax)+1) {
      GoToActive = false;                                            //end movement
      Goj = 1;                                                       //reset iterator integer
    }
  }
  
  //do tipping in z around mid-position (z=500, z-Range = [0, 1000], where 1000 is up)
  for(int i=0; i<LineUsDevice.length; i++){
    if (TippingActive[i]== true) {
      int zi = int(sin(nzi * dnzi)*ziAmp/2 + 500.0);
      LineUs[i].setLineUsElocZ(zi);
    }
    nzi++;
  }
  
  //show table
  drawTable();
  
  //show help
  if (HelpActive==true) {
    image(img_help, 0, 0, width, height);
  }
}

//function drawing the table
void drawTable() {
  if (FileLoaded==true) {
    float TableYPos = SetupCenter.y - 0.25*(PondSize+PlateSize+PondWidth)*scale;
    float TableXPos = height + 45;
    float TableLineSpacing = 18;
    float TableColumnSpacing = (width-height)/12;
  
    textFont(myFontBold); fill(0); textAlign(RIGHT, CENTER); int j=0; strokeWeight(1);
    for (int i=0; i<8; i++) { //write table column headers
      fill(255*int(i%2==0), 0, 255*int(i%2==1));
      text(PosTable.getColumnTitle(i), TableXPos + (i+2)*TableColumnSpacing, TableYPos - TableLineSpacing);
      if (i%2==0) {           //vertical lines
        stroke(0);
        line(TableXPos + (i+1.1)*TableColumnSpacing, TableYPos - 1.2*TableLineSpacing, TableXPos + (i+1.1)*TableColumnSpacing, TableYPos + TableLineSpacing*(4*PosTable.getRowCount()) - 0.5*TableLineSpacing);
      }
    }
    //extra vertical line for config. mini previews
    fill(0); text("scale " + nf(ConfScale,1,1), TableXPos + 10.4*TableColumnSpacing, TableYPos - TableLineSpacing);
    line(TableXPos + 9.1*TableColumnSpacing, TableYPos - 1.2*TableLineSpacing, TableXPos + 9.1*TableColumnSpacing, TableYPos + TableLineSpacing*(4*PosTable.getRowCount()) - 0.5*TableLineSpacing);
    //horizontal table lines
    for (int i=0; i<PosTable.getRowCount(); i++) {
      line(TableXPos - 35, TableYPos + 4*i*TableLineSpacing - 0.45*TableLineSpacing, TableXPos + 10.5*TableColumnSpacing, TableYPos + 4*i*TableLineSpacing - 0.45*TableLineSpacing);
    }
    
    for (TableRow row : PosTable.rows()) {
      if (j==ConfID) {
        fill(255,0,0); //red
      } else {
        fill(0);       //black
      }
      textFont(myFontBold); textAlign(RIGHT, CENTER); //write table row headers
      text("Conf " + str(j+1) + ", F [mm]:", TableXPos + 0.9*TableColumnSpacing, TableYPos + 4*j*TableLineSpacing);
      text("curr. F [mm]:", TableXPos + 0.9*TableColumnSpacing, TableYPos + (4*j+1)*TableLineSpacing);
      text("curr. E [px]:", TableXPos + 0.9*TableColumnSpacing, TableYPos + (4*j+2)*TableLineSpacing);
      text("loc. E [u]:", TableXPos + 0.9*TableColumnSpacing, TableYPos + (4*j+3)*TableLineSpacing);
      textFont(myFont);
      for (int i=0; i<8; i++) { //write table rows
        float v = row.getFloat(i); //show all rows from table data
        String Ei; String Fi; String Eloci;
        if (j==ConfID) {
          PVector FVec = LineUs[floor(i/2)].getLineUsF(); //show live: actual target F vector
          Fi = ((i%2==0) ? nf((FVec.x - SetupCenter.x)/scale,1,1) : nf((FVec.y - SetupCenter.y)/scale,1,1));
          PVector EofF = LineUs[floor(i/2)].getLineUsE();
          Ei = ((i%2==0) ? nf(EofF.x,1,1) : nf(EofF.y,1,1));
          PVector Elocal = LineUs[floor(i/2)].getLineUsEloc();
          Eloci = ((i%2==0) ? nf(int(Elocal.x),1,0) : nf(int(Elocal.y),1,0));
        } else {
          Ei = "";
          Fi = "";
          Eloci="";
        }
        text(nf(v,1,1), TableXPos + (i+2)*TableColumnSpacing, TableYPos + 4*j*TableLineSpacing);
        text(Fi, TableXPos + (i+2)*TableColumnSpacing, TableYPos + (4*j+1)*TableLineSpacing);
        text(Ei, TableXPos + (i+2)*TableColumnSpacing, TableYPos + (4*j+2)*TableLineSpacing);
        text(Eloci, TableXPos + (i+2)*TableColumnSpacing, TableYPos + (4*j+3)*TableLineSpacing);
      }
      for (int i =0; i<4; i++) { //draw mini configuration previews
        float minix = row.getFloat(2*i); float miniy = row.getFloat(2*i+1);
        fill(FColor); strokeWeight(1);
        ellipse(TableXPos + 9.75*TableColumnSpacing + minix*ConfPreviewScale, TableYPos + (4*j+1.5)*TableLineSpacing + miniy*ConfPreviewScale, JointRadius,JointRadius);
      }
      j++;
    }
  }
}

//class defining digital twins of Line-Us devices
class LineUsDigital {
  int LineUsID;                             //ID of this Line-Us device
  PVector LineUsPos;                        //position of Line-Us device (A)
  PVector LineUsE;                          //Position of Line-Us device's Pen holder (E)
  PVector LineUsEloc;                       //Position of Line-Us device's Pen holder (E) in devices' coordinate system
  int LineUsElocZ;                          //stored z-coordinate
  PVector LineUsF;                          //Position of Line-Us device's extension point (F)
  float LineUsOrientation;                  //Orientation of Line-Us device (angle relative to horizontal)
  PVector[] Points = new PVector[6];        //Array that can hold 6 PVectors: D, A, B, C, E=(x,y), F
  PShape[] BoundingBoxes = new PShape[8];   //Array that can hold 6 PShapes (the bounding boxes). Each vertex (4 per PShape) of a shape is a PVector. 3 extra BB (pen holder)
  boolean IsActive;                         //boolean variable storing if this Line-Us device is the active one
  boolean ValidPosition;                    //boolean variable signifying if position is a valid one (no bb collision, E&F inside pond/drawing area)
  
  //constructor routine, called upon instancing. Variables with underscore are passed over to object. Contains initializations.
  LineUsDigital (int _LineUsID, PVector _LineUsPos, PVector _LineUsE, float _LineUsOrientation) {
    LineUsID = _LineUsID;
    LineUsPos = _LineUsPos;
    LineUsE = _LineUsE;
    LineUsOrientation = _LineUsOrientation;
    
    LineUsEloc = new PVector(1000,1000,0); //initialize
    LineUsElocZ = 1000;
    for(int i=0; i<Points.length; i++){
      Points[i] = new PVector(0, 0, 0);
    }
    for(int i=0; i<BoundingBoxes.length; i++){
      BoundingBoxes[i] = createShape();
      BoundingBoxes[i].beginShape();
      for (int j=0; j<bbv[i]; j++) {
        BoundingBoxes[i].vertex(0,0);
      }
      BoundingBoxes[i].endShape(CLOSE);
      BoundingBoxes[i].setStroke(false); 
      BoundingBoxes[i].setFill(BBDefaulColor);
    }
  }
  
  //function setting the Active state of this Line-Us
  void setLineUsActive (boolean ActiveValue) {
    IsActive = ActiveValue;
  }
  
  //manually set E of this Line-Us (also called from mouse dragging, setLineUsF() and setLineUsEloc())
  void setLineUsE(PVector setPos) {
    LineUsE.set(setPos);
    if ((LineUsConnected[LineUsID] == true) && (ValidPosition==true)) {
      PVector Elocal = LineUs[LineUsID].getLineUsEloc();
      LineUsDevice[LineUsID].g01(int(Elocal.x), int(Elocal.y), int(Elocal.z)); delay(DefaultDelay);
    }
  }
  
  //manually set F of this Line-Us (also called from mouse dragging)
  void setLineUsF(PVector targetF) {
    PVector Eroot = getEfromF(LineUsPos, LineUsE, targetF);        //get E(F) via root search
    LineUs[LineUsID].setLineUsE(Eroot);                            //set this Line-Us's E to E(F)
  }
  
  //return PVector of this Line-Us device's Point E
  PVector getLineUsE() {
    return LineUsE;
  }
  
  //return PVector of this Line-Us device's Point E
  PVector getLineUsF() {
    return LineUsF;
  }
  
  //return PVector of this Line-Us device's Point E in device's own (rotated) coordinate system
  PVector getLineUsEloc() {
    return LineUsEloc;
  }
  
  //.z component of PVector LineUsEloc
  void setLineUsElocZ(int targetZ) {
    LineUsElocZ = targetZ; //LineUsEloc.z is set to this value at the end of .update() routine
  }
  
  //set PVector of this Line-Us device's Point E, inferred from E in device's own (rotated) coordinate system
  void setLineUsEloc(PVector targetEloc) {
    PVector LineUsETemp = PVector.mult(targetEloc, (5.0/100)*scale);                               //rescale to [px], 100 units per 5mm, LineUsE/scale in [mm]
    PVector LineUsETarget = new PVector(0, 0, 0);
    LineUsETarget.x = LineUsETemp.x*cos(LineUsOrientation) - LineUsETemp.y*sin(LineUsOrientation); //rotate to canvas coordinate system orientation
    LineUsETarget.y = LineUsETemp.x*sin(LineUsOrientation) + LineUsETemp.y*cos(LineUsOrientation);
    LineUsETarget = PVector.add(LineUsPos, LineUsETarget);                                         //local coordinates centered on A=LineUsPos
    LineUs[LineUsID].setLineUsE(LineUsETarget);                                                    //sets this devices' corresponding E in canvas coordinate system (z=0)
  }
  
  //get PVector of this Line-Us device's Point A
  PVector getLineUsA() {
    return LineUsPos;
  }
  
  //set PVector of this Line-Us device's Point A
  void setLineUsA(PVector targetA) {
    LineUsPos = targetA;
  }
  
  //function to return a certain bounding box (used in collision detection)
  PShape getBoundingBox(int i) {
    return BoundingBoxes[i];
  }
  
  //function to set a certain bounding boxes' color of this Line-Us device
  void setBoundigBoxColor(int i, color col) {
    BoundingBoxes[i].setFill(col);
  }
  
  //update LineUs, check dragging, get arms & bounding boxes, do collision testing
  void update() {
    if (DragMode==0) {
      //drag E (DragMode=0, default)
      if (mousePressed==true & dist(LineUsE.x, LineUsE.y, mouseX, mouseY) <= CircRadius) {
        PVector setEVec = new PVector(mouseX, mouseY, 0);       //set E to cursor
        ActiveLineUs = LineUsID; ToggleLineUsActive(LineUsID);
        LineUs[LineUsID].setLineUsE(setEVec);
      }
    } else if (DragMode==1) {
      //drag F
      if (mousePressed==true & dist(LineUsF.x, LineUsF.y, mouseX, mouseY) <= CircRadius) {
        PVector LineUsFTarget = new PVector(mouseX, mouseY, 0); //set target F to cursor
        ActiveLineUs = LineUsID; ToggleLineUsActive(LineUsID);
        LineUs[LineUsID].setLineUsF(LineUsFTarget);             //set this Line-Us's F to target F (init root search)
      }
    } else if (DragMode==2) {
      //drag A
      if (mousePressed==true & dist(LineUsPos.x, LineUsPos.y, mouseX, mouseY) <= CircRadius) {
        PVector LineUsATarget = new PVector(mouseX, mouseY, 0); //set target F to cursor
        ActiveLineUs = LineUsID; ToggleLineUsActive(LineUsID);
        LineUs[LineUsID].setLineUsA(LineUsATarget);             //set this Line-Us's F to target F (init root search)
      }
    }
    
    //calculate arm points from Line-Us position = A and (x,y)=E
    Points = getArmCoords(LineUsPos, LineUsE, scale*a, scale*b, scale*e);
    LineUsE = Points[4]; //update E for this object
    LineUsF = Points[5]; //get precise & consistent F from (A,E)
    
    //check if F & E are inside
    boolean RodOnCanvasQ = RodOnCanvas(Points[4], Points[5]);
    
    boolean intersect = false;
    if (ToggleBBCD==true) {
      //create 5 bounding boxes (width 2w) around line-segments connecting new arm points {D, A, B, C, E, F}
      for(int i=0; i<5; i++){
        PVector uAmB = PVector.sub(Points[i], Points[i+1]); uAmB.normalize(); //unit vector along P1-P2 (here called A & B)
        PVector unAmB = zVec.cross(uAmB);                                     //unit vector normal to P1-P2
        BoundingBoxes[i].setVertex(0, Points[i].x + wi[i]*(uAmB.x - unAmB.x), Points[i].y + wi[i]*(uAmB.y - unAmB.y));
        BoundingBoxes[i].setVertex(1, Points[i].x + wi[i]*(uAmB.x + unAmB.x), Points[i].y + wi[i]*(uAmB.y + unAmB.y));
        BoundingBoxes[i].setVertex(2, Points[i+1].x - wi[i]*(uAmB.x - unAmB.x), Points[i+1].y - wi[i]*(uAmB.y - unAmB.y));
        BoundingBoxes[i].setVertex(3, Points[i+1].x - wi[i]*(uAmB.x + unAmB.x), Points[i+1].y - wi[i]*(uAmB.y + unAmB.y));
      }
      //extra bounding box (hexagon) for pen-holder around E=Points[4]
        PVector uAmB = PVector.sub(Points[4], Points[3]); uAmB.normalize();  //unit vector along C-E
        PVector unAmB = zVec.cross(uAmB);                                    //unit vector normal to C-E
        PVector ShapeVertj = PVector.add(Points[4], PVector.add(PVector.mult(uAmB, wi[5]*sqrt(3)/2), PVector.mult(unAmB, -wi[5]/2)));
        for (int j=0; j<6; j++) {
          ShapeVertj = PVector.add(ShapeVertj, PVector.mult(unAmB, wi[5]));
          BoundingBoxes[5].setVertex(j, ShapeVertj.x, ShapeVertj.y);
          unAmB.rotate(radians(60.0));
        }
      //extra bounding box (rectangle) for screw attached to pen holder at E
        float Screwz1 = 0.85;
        float Screwz2 = 1.8;
        BoundingBoxes[6].setVertex(0, Points[4].x + wi[6]*(Screwz1*uAmB.x + unAmB.x), Points[4].y + wi[6]*(Screwz1*uAmB.y + unAmB.y));
        BoundingBoxes[6].setVertex(1, Points[4].x + wi[6]*(Screwz2*uAmB.x + unAmB.x), Points[4].y + wi[6]*(Screwz2*uAmB.y + unAmB.y));
        BoundingBoxes[6].setVertex(2, Points[4].x + wi[6]*(Screwz2*uAmB.x - unAmB.x), Points[4].y + wi[6]*(Screwz2*uAmB.y - unAmB.y));
        BoundingBoxes[6].setVertex(3, Points[4].x + wi[6]*(Screwz1*uAmB.x - unAmB.x), Points[4].y + wi[6]*(Screwz1*uAmB.y - unAmB.y));
      //extra bounding box (hexagon) around F=Point[5]
        uAmB = PVector.sub(Points[5], Points[4]); uAmB.normalize();  //unit vector along E-F
        unAmB = zVec.cross(uAmB);                                    //unit vector normal to E-C
        ShapeVertj = PVector.add(Points[5], PVector.add(PVector.mult(uAmB, wi[7]*sqrt(3)/2), PVector.mult(unAmB, -wi[7]/2)));
        for (int j=0; j<6; j++) {
          ShapeVertj = PVector.add(ShapeVertj, PVector.mult(unAmB, wi[7]));
          BoundingBoxes[7].setVertex(j, ShapeVertj.x, ShapeVertj.y);
          unAmB.rotate(radians(60.0));
        }
       
      //collision test of this Line-Us with all other devices if there are more than 1
      if(LineUs.length > 1) {
        for(int i=0; i<BoundingBoxes.length; i++) { //go through all bounding boxes i of this device
          boolean intersecti = false;
          for(int k=0; k<LineUs.length; k++){ //go through all other LineUsDevices
            if (k != LineUsID) {
              for(int j=0; j<BoundingBoxes.length; j++){ //go through bounding boxes of kth Line-Us device
                boolean intersectij = intersects(LineUs[k].getBoundingBox(j), BoundingBoxes[i]); //i-th bounding box intersects j-th bounding box of k-th Line-Us?
                intersecti = intersecti || intersectij; //i-th bounding box of this Line-Us intersects with any other Line-Us?
                if (intersectij == true) {
                  LineUs[k].setBoundigBoxColor(j, BBIntersectColor);
                } else {
                  LineUs[k].setBoundigBoxColor(j, BBDefaulColor);
                }
              }
            }
          }
          intersect = intersect || intersecti; //Line-Us intersects with any other Line-Us?
          if (intersecti == true) {
            BoundingBoxes[i].setFill(BBIntersectColor);
          } else {
            BoundingBoxes[i].setFill(BBDefaulColor);
          }
        }
      }
    } //end BB calculation & collision detection (in void update())
    
    //valid position if no intersection of BB as well as F&E in pond (/on canvas)
    ValidPosition = RodOnCanvasQ && (!intersect);
    
    //check if update resulted in valid position. If not, set to last valid.
    if ((LineUsF.x != LineUsF.x) || (ValidPosition==false)) { //F is NaN, i.e. calc. failed, invalid positioning attempt //
      LineUsE.set(LineUsELastValid[LineUsID]);
      Points = getArmCoords(LineUsPos, LineUsE, scale*a, scale*b, scale*e);
      LineUsF.set(Points[5]);
      GoToActive = false;
    } else {
      LineUsELastValid[LineUsID] = LineUsE;
    }
    
    //get (valid) E in local coordinate system of associated device (relies on synchronous definitions)
    PVector LineUsElocTemp = PVector.mult(PVector.sub(LineUsE, LineUsPos), (1/scale)*(100/5)); //100 units per 5mm, LineUsE/scale in [mm], local coordinates centered on A=LineUsPos
    PVector CoordFrameXVec = new PVector(1,0);
      CoordFrameXVec.rotate(LineUsOrientation);
    PVector CoordFrameYVec = new PVector(0,1);
      CoordFrameYVec.rotate(LineUsOrientation);
    LineUsEloc.x = PVector.dot(LineUsElocTemp, CoordFrameXVec);
    LineUsEloc.y = PVector.dot(LineUsElocTemp, CoordFrameYVec);
    LineUsEloc.z = float(LineUsElocZ);
  }
  //end void update()
  
  //Auxillary function: collision test of this Line-Us device with any PShape
  boolean CollisionTest(PShape TestShape) {
    boolean intersectQ = false;
    for(int i=0; i<BoundingBoxes.length; i++){
      intersectQ = intersectQ || intersects(TestShape, BoundingBoxes[i]);
    }
    return intersectQ;
  }
  
  //draw Line-Us device, arms, bounding boxes, joints, dragging discs 
  void display() {
    //draw this Line-Us device
    strokeWeight(2); stroke(0); fill(int(ValidPosition)*255, 200); ellipseMode(CENTER);
    pushMatrix();
      translate(LineUsPos.x, LineUsPos.y);
      rotate(LineUsOrientation);
      ellipse(0, - 0.5*LineUsLength*scale, LineUsWidth*scale/2, LineUsWidth*scale/2);
      rect(0, - 0.2*LineUsLength*scale, LineUsWidth*scale, LineUsLength*scale);
      fill(0); textAlign(CENTER, CENTER);
      text(str(LineUsElocZ), 0, - 0.5*LineUsLength*scale); //display z-Value in local coordinate system (0,1000)
      textFont(myFont); fill(0); textAlign(CENTER, CENTER); textSize(30); text(str(LineUsID+1), 0, - 0.2*LineUsLength*scale); //Line-Us ID
    popMatrix();
    
    //draw local coord. system
    strokeWeight(1);
    PVector CoordFrameXVec = new PVector(1,0);
      CoordFrameXVec.rotate(LineUsOrientation);
      CoordFrameXVec.mult(100);
      CoordFrameXVec = PVector.add(LineUsPos, CoordFrameXVec);
    PVector CoordFrameYVec = new PVector(0,1);
      CoordFrameYVec.rotate(LineUsOrientation);
      CoordFrameYVec.mult(100);
      CoordFrameYVec = PVector.add(LineUsPos, CoordFrameYVec);
    stroke(265,0,0); //local x-axis is red
    line(LineUsPos.x, LineUsPos.y, CoordFrameXVec.x, CoordFrameXVec.y);
    stroke(0,0,255); //local y-axis is blue
    line(LineUsPos.x, LineUsPos.y, CoordFrameYVec.x, CoordFrameYVec.y);
    
    //draw arms (lines connecting points, bounding boxes, joints), connects segments like {D-A-B-C-E-F} 
    stroke(0); strokeWeight(4);
    for(int i=0; i<Points.length; i++){
      if(i<Points.length-1) {
        line(Points[i].x, Points[i].y, Points[i+1].x, Points[i+1].y);
      }
      fill(0); ellipse(Points[i].x, Points[i].y, JointRadius,JointRadius);
    }
    if (ToggleBBCD==true) {
      for(int i=0; i<BoundingBoxes.length; i++){
        shape(BoundingBoxes[i]);
      }
    }
    
    //draw interactive dragging discs
    noStroke(); strokeWeight(2);
    if (IsActive==true) {
      fill(ActiveEColor); 
    } else {
      fill(DefaultEColor);
    }
    if (DragMode==0) {
      ellipse(LineUsE.x,LineUsE.y, CircRadius*2, CircRadius*2);
    } else if (DragMode==1) {
      ellipse(LineUsF.x,LineUsF.y, CircRadius*2, CircRadius*2);
    } else if (DragMode==2) {
      ellipse(LineUsPos.x,LineUsPos.y, CircRadius*2, CircRadius*2);
    }
    textFont(myFont); fill(255); textAlign(CENTER, CENTER); textSize(10);
    
    //draw extension arm positions F
    fill(FColor); ellipse(Points[5].x, Points[5].y, JointRadius,JointRadius);
  }
}

//==============================================================================================================
//======================================INVERSE KINEMATICS======================================================
//==============================================================================================================

PVector getEfromF(PVector A, PVector E, PVector F) {
  float eps = 1E-6;
  
  //Gradient method, starting from phi corresponding to current E
  float alpha = 1E-5; int NMAX=200; int N=0;
  PVector EmF = PVector.sub(E, F);
  float phi = atan2(EmF.y, EmF.x); //get proper initial value, otherwise search might fail or give NaNs
  while ((getEfromFfunc(A, F, phi) > eps) && (N<NMAX)) {
    phi = phi - alpha * getEfromFfuncp(A, F, phi);
    N++;
  }
  
  return Echeckphi(F, phi);
}

float getEfromFfuncp(PVector A, PVector F, float phi) {
  float dphi = 1E-3;
  return (getEfromFfunc(A, F, phi + dphi) - getEfromFfunc(A, F, phi - dphi)) / (2*dphi);
}

//E testwise around F (angle phi relative to positive x-axis)
PVector Echeckphi(PVector F, float phi) {
  PVector ux = new PVector(1, 0, 0);
  PVector uy = new PVector(0, 1, 0);
  return PVector.add(F, PVector.add(PVector.mult(ux, scale*e*cos(phi)), PVector.mult(uy, scale*e*sin(phi))));
}

//function used to minimize target F from F(E(phi)), with E(phi) on circle (radius e) around target F.
float getEfromFfunc(PVector A, PVector F, float phi) {
  PVector Echeck = Echeckphi(F, phi);
  PVector[] Pnts = new PVector[6];
  Pnts = getArmCoords(A, Echeck, a*scale, b*scale, e*scale); //Pnts[5] is F calculated for trial Echeck(x)
  return PVector.dist(Pnts[5], F);                           //compare with target F to see if Echeck was the right one
}

//Points = {D, A, B, C, E, F} for given A(origin), E=(x,y) and arm lengths a (long), b (short). Order chosen for easy plotting.
PVector[] getArmCoords(PVector A, PVector E, float a, float b, float e) {
  PVector[] Pnts = new PVector[6]; //= {D, A, B, C, E, F}, each a 3D vector
  
  float c = sqrt(pow(E.x - A.x, 2)+pow(E.y - A.y, 2));
  float d = sqrt(a*a - c*c/4);
  Pnts[1] = new PVector(A.x, A.y, 0);                                                             //=A
  Pnts[4] = new PVector(E.x, E.y, 0);                                                             //=E
  PVector uEmA = PVector.sub(Pnts[4], Pnts[1]); uEmA.normalize();                                 //unit vector (E-A)/|E-A|
  PVector unEmA = zVec.cross(uEmA);                                                               //unit normal vector to (E-A)
  Pnts[0] = PVector.add(Pnts[1], PVector.add(PVector.mult(uEmA, c/2), PVector.mult(unEmA, d)));   //D = A + uEmA*(c/2) + unEmA*d
  Pnts[3] = PVector.add(Pnts[4], PVector.mult(PVector.sub(Pnts[0], Pnts[4]), (b+a)/a));           //C = E + (D-E)*(b/a)
  Pnts[2] = PVector.add(Pnts[3], PVector.sub(Pnts[1], Pnts[0]));                                  //B = C + (A-D)
  PVector uEmD = PVector.sub(Pnts[4], Pnts[0]); uEmD.normalize();                                 //unit vector (E-D)/|E-D|
  PVector unEmD = zVec.cross(uEmD);                                                               //unit normal vector to (E-D)
  Pnts[5] = PVector.add(Pnts[4], PVector.add(PVector.mult(uEmD, e*cos(gamma)), PVector.mult(unEmD, e*sin(gamma)))); //F = E + e*(cos(gamma)*uEmD+sin(gamma)*unEmD)
  
  return Pnts;
}

//Points = {D, A, B, C, E, F} for given A(origin), angles S & E and arm lengths a (long), b (short). (currently not used)
PVector[] getArmCoordsSE(PVector A, float S, float E, float a, float b, float e) {
  PVector[] Pnts = new PVector[6]; //= {D, A, B, C, E, F}, each a 3D vector
  PVector ux = new PVector(1, 0, 0);
  PVector uy = new PVector(0, 1, 0);
  
  Pnts[1] = new PVector(A.x, A.y, 0);                                                                               //=A
  Pnts[0] = PVector.add(Pnts[1], PVector.add(PVector.mult(ux, a*cos(S-PI/4)), PVector.mult(uy, a*sin(S-PI/4))));    //D=A+cos()...+sin()..
  Pnts[2] = PVector.add(Pnts[1], PVector.add(PVector.mult(ux, -b*cos(E-PI/4)), PVector.mult(uy, b*sin(E-PI/4))));   //B=A+cos()..+sin()..
  Pnts[3] = PVector.add(Pnts[2], PVector.sub(Pnts[0], Pnts[1]));                                                    //C=B+(D-A)
  Pnts[4] = PVector.add(Pnts[0], PVector.mult(PVector.sub(Pnts[0], Pnts[3]), (a+b)/b));                             //E=D+(D-C)*(a+b)/b
  PVector uEmD = PVector.sub(Pnts[4], Pnts[0]); uEmD.normalize();                                                   //unit vector (E-D)/|E-D|
  PVector unEmD = zVec.cross(uEmD);                                                                                 //unit normal vector to (E-D)
  Pnts[5] = PVector.add(Pnts[4], PVector.add(PVector.mult(uEmD, e*cos(gamma)), PVector.mult(unEmD, e*sin(gamma)))); //F = E + e*(cos(gamma)*uEmD+sin(gamma)*unEmD)
  
  return Pnts;
}

//==============================================================================================================
//======================================OBB-BBO COLLISION DETECTION=============================================
//==============================================================================================================
boolean RodOnCanvas(PVector E, PVector F) {
  float Lim = (0.5*PondSize - PondSaftyWidth)*scale;
  boolean checkF = (F.x > SetupCenter.x - Lim) && (F.x < SetupCenter.x + Lim) && (F.y > SetupCenter.y - Lim) && (F.y < SetupCenter.y + Lim);
  boolean checkE = (E.x > SetupCenter.x - Lim) && (E.x < SetupCenter.x + Lim) && (E.y > SetupCenter.y - Lim) && (E.y < SetupCenter.y + Lim);
  return checkF && checkE;
}

// OBB-OBB (oriented bounding box) collision detection (here: assumes equal number of vertices per PShape)
// adopted from https://gamedev.stackexchange.com/questions/25397/obb-vs-obb-collision-detection
boolean intersects(PShape shape1, PShape shape2) {
  //define number of vertices (of shape1 and shape2) and get their Corners
  int NVert1 = shape1.getVertexCount(); 
  int NVert1m1 = NVert1 - 1;
  PVector[] Corners1 = new PVector[NVert1];
  for(int i = 0; i < NVert1; i++) {
    Corners1[i] = shape1.getVertex(i);
  }
  int NVert2 = shape2.getVertexCount(); 
  int NVert2m1 = NVert2 - 1;
  PVector[] Corners2 = new PVector[NVert2];
  for(int i = 0; i < NVert2; i++) {
    Corners2[i] = shape2.getVertex(i);
  }
  // Get the normals for shape1
  PVector[] normals1 = new PVector[NVert1];
  for(int i = 0; i < NVert1; i++) {
    PVector sidei = (i==NVert1m1) ? PVector.sub(Corners1[NVert1m1], Corners1[0]) : PVector.sub(Corners1[i+1], Corners1[i]);
    normals1[i] = zVec.cross(sidei); normals1[i].normalize();
  }
  //project all bounding boxes on normals of shape1
  float[] minmaxAlong1, minmaxAlong2 = new float[2];
  for(int i = 0; i < NVert1; i++) {
    minmaxAlong1 = SATtest(normals1[i], Corners1);
    minmaxAlong2 = SATtest(normals1[i], Corners2);
    if(!overlaps(minmaxAlong1, minmaxAlong2)) return false; // NO INTERSECTION
  }
  // Get the normals for shape2
  PVector[] normals2 = new PVector[NVert2];
  for(int i = 0; i < NVert2; i++) {
    PVector sidei = (i==NVert2m1) ? PVector.sub(Corners2[NVert2m1], Corners2[0]) : PVector.sub(Corners2[i+1], Corners2[i]);
    normals2[i] = zVec.cross(sidei); normals2[i].normalize();
  }
  //project all bounding boxes on normals of shape2
  for(int i = 0; i < NVert2; i++) {
    minmaxAlong1 = SATtest(normals2[i], Corners1);
    minmaxAlong2 = SATtest(normals2[i], Corners2);
    if(!overlaps(minmaxAlong1, minmaxAlong2)) return false; // NO INTERSECTION
  }
  return true; // if overlap occurred in ALL AXES, then they do intersect
}

float[] SATtest(PVector axis, PVector[] ptSet) {
  float[] minmaxAlong = new float[2];
  minmaxAlong[0] =100000;  //min 
  minmaxAlong[1] =-100000; //max
  for(int i = 0; i < ptSet.length; i++) { // just dot it to get the min/max along this axis.
    float dotVal = ptSet[i].dot(axis);
    if(dotVal < minmaxAlong[0]) minmaxAlong[0]=dotVal;
    if(dotVal > minmaxAlong[1]) minmaxAlong[1]=dotVal;
  }
  return minmaxAlong;
}

boolean overlaps(float[] minmaxAlong1, float[] minmaxAlong2) {
  return isBetweenOrdered(minmaxAlong2[0], minmaxAlong1[0], minmaxAlong1[1]) || isBetweenOrdered(minmaxAlong1[0], minmaxAlong2[0], minmaxAlong2[1]);
}

boolean isBetweenOrdered(float val, float lowerBound, float upperBound) {
  return lowerBound <= val && val <= upperBound;
}

//==============================================================================================================
//===============================PHYSICAL INTERFACE============================================================
//==============================================================================================================

class LineUs {
  Client lineUs;
  String LineUsMachineName;
  
  Boolean connected = false;
  String helloMessage;
  
  LineUs(PApplet papp, String LineUsMachineName) {
    lineUs = new Client(papp, LineUsMachineName, 1337);
    connected = true;
    helloMessage = readResponse();
  }
  
  String getHelloString() {
    if(connected) {
      return helloMessage;
    } else {
      return("Not connected");
    }
  }
  
  //Close the connection to the Line-us
  void disconnect() {
    lineUs.stop();
    connected = false;
  }
  
  //Send a G01 (interpolated move), and wait for the response before returning
  void g01(int x, int y, int z) {
    String cmd = "G01 X";
    cmd += str(x);
    cmd += " Y";
    cmd += str(y);
    cmd += " Z";
    cmd += str(z);
    sendCommand(cmd);
    readResponse();
  }
  
  //Send a M114 S command (retrieve angles E, S, P), and wait for the response before returning
  //requires firmware update to latest version
  void M114() {
    String cmd = "M114 S";
    sendCommand(cmd);
    readResponse();
  }
  
  //Read from the socket one byte at a time until we get a null
  String readResponse() {
    String line = "";
    int c;
    while(true) {
       c = lineUs.read();
       if(c != 0 && c != -1) {
         line += (char) c;
       } else if(c == 0) {
         break;
       }
    }
    return line;
  }
  
  //Send the command to Line-us
  void sendCommand(String command) {
    command += "\0";
    lineUs.write(command);
  }
  
  boolean getConnectionStatus() {
    return connected;
  } 
}

//connect to a given named Line-Us device (named "line-us#" with #=1,2,3,4 via settings in Line-Us App)
void ConnectToLineUs(int ID) {
  String DeviceName = "line-us" + str(ID+1) + ".local";
    print("trying to connect to Line-Us device" + DeviceName + " as LineUsDevice[" + int(ID) + "]\n");
  LineUsDevice[ID] = new LineUs(this, DeviceName); // "line-us1" instance
  LineUsConnected[ID] = LineUsDevice[ID].getConnectionStatus();
    print(" ... Line-Us device" + DeviceName + " connected as LineUsDevice[" + int(ID) + "]\n");
}

//==============================================================================================================
//=================================KEYBOARD INTERFACE===========================================================
//==============================================================================================================

void keyPressed() {
  if (key == CODED) {
    if (keyCode == UP) {
      if (keyCONTROL == true) {
        ds = min(10, ds + 1.0);
      } else {
        PVector inc = new PVector(0, -dx, 0);
        incPos(inc);
      }
    } else if (keyCode == DOWN) {
      if (keyCONTROL == true) {
        ds = max(ds - 1.0, 1);
      } else {
        PVector inc = new PVector(0, dx, 0);
        incPos(inc);
      }
    } else if (keyCode == LEFT) {
      if (keyCONTROL == true) {
        dx = max(dx - 0.1, 0.1);
      } else {
        PVector inc = new PVector(-dx, 0, 0);
        incPos(inc);
      }
    } else if (keyCode == RIGHT) {
      if (keyCONTROL == true) {
        dx = min(10, dx + 0.1);
      } else {
        PVector inc = new PVector(dx, 0, 0);
        incPos(inc);
      }
    } else if (keyCode == CONTROL) {
      keyCONTROL = true;
    }
  } else {
    switch (key) {
      case('1'): case('2'): case('3'): case('4'): case('5'): case('6'): case('7'): case('8'): case('9'):
        int num = int(key)-48;           //get number pressed
        if (keyCONTROL == true) {
          ConfID = num-1;               //set active configuration
          if (FileLoaded==true) {
            ConfID = min(ConfID, PosTable.getRowCount()-1);
          }
          PrepareFs();
        } else if (keyC == true) {
          if (num<= 4) {
            ConnectToLineUs(num-1);
          }
        } else if (keyT == true) {
          if (num<= 4) {
            TippingActive[num-1] = !TippingActive[num-1];
          }
        } else {
          if (num<= 4) {
            ActiveLineUs = num-1; ToggleLineUsActive(ActiveLineUs);
          }
        }
        break;
      case('i'):
        PrepareEs(); SetEs();
        break;
      case('f'):
        DragMode=1;
        break;
      case('w'):
        ToggleBBCD = !ToggleBBCD;
        break;
      case('e'):
        DragMode=0;
        break;
      case('a'):
        if (keyCONTROL == true) {
          ConnectToLineUs(0); ConnectToLineUs(1); ConnectToLineUs(2); ConnectToLineUs(3);
        } else {  
          DragMode=2;
        }
        break;
      case('t'):
        keyT = true; //toggle tipping if pressed in combination with #
        break;
      case('o'):
        PVector Ehome = new PVector(1000.0, 1000.0, 1000.0);
        for(int i=0; i<LineUs.length; i++) {     //go through all Line-Us devices (digital twins)
          LineUs[i].setLineUsEloc(Ehome);
          LineUs[i].setLineUsElocZ(int(Ehome.z));
          LineUsEs[i] = LineUs[i].getLineUsE();
        }
        SetEs();
        break;
      case('+'):
        ConfID = ConfID +1;
        if (FileLoaded==true) {
          ConfID = min(ConfID, PosTable.getRowCount()-1);
        }
        PrepareFs();
        break;
      case('-'):
        ConfID = max(ConfID -1, 0);
        PrepareFs();
        break;
      case('.'):
        ConfScale = ConfScale + 0.1; //scale up
        break;
      case(','):
        ConfScale = max(0, ConfScale - 0.1); //scale down
        break;
      case('c'):
        keyC = true;
        break;
      case(ENTER):
        PrepareFs();
        GoToActive = !GoToActive; //toggle move or no move towards target
        if (GoToActive == true) {
          Goj = 1;
        }
        break;
      case('s'):
        if (FileLoaded==true) {
          //update table's F's
          for(int i=0; i<LineUs.length; i++) {                                     //go through all Line-Us devices (digital twins) 
            PVector currentFi = LineUs[i].getLineUsF();
            PosTable.setFloat(ConfID, 2*i, (currentFi.x - SetupCenter.x)/scale);   //save in [mm]
            PosTable.setFloat(ConfID, 2*i+1, (currentFi.y - SetupCenter.y)/scale); //save in [mm]
            PrepareFs();
            print("stored");
          }
        }//save current row
        break;
      case('h'):
        HelpActive = !HelpActive;
        break;
    }
  }
}

void keyReleased() {
  if (key == CODED) {
    if (keyCode == CONTROL) {
      keyCONTROL=false;
    }
  } else {
    if (key == 'c') {
      keyC = false; 
    } else if (key == 't') {
      keyT = false;
    }
  }
}

void incPos(PVector inc) {
  if (DragMode==0) {
    PVector currentPos = LineUs[ActiveLineUs].getLineUsE();
    LineUs[ActiveLineUs].setLineUsE(PVector.add(currentPos, inc));
  } else if (DragMode==1) {
    PVector currentPos = LineUs[ActiveLineUs].getLineUsF();
    LineUs[ActiveLineUs].setLineUsF(PVector.add(currentPos, inc));
  } else if (DragMode==2) {
    PVector currentPos = LineUs[ActiveLineUs].getLineUsA();
    LineUs[ActiveLineUs].setLineUsA(PVector.add(currentPos, inc));
  }
}

//set Line-Us[ActiveLU] to active and all others to inactive
void ToggleLineUsActive(int ActiveLU) {
  for(int i=0; i<LineUs.length; i++) {
    if (i == ActiveLU) {
      LineUs[i].setLineUsActive(true);
    } else {
      LineUs[i].setLineUsActive(false);
    }
  }
}
