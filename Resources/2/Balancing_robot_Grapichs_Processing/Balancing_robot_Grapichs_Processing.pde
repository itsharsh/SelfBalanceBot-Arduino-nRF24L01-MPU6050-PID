import processing.serial.*;
Serial myPort;
int x = 0;


void setup() {
  size(600, 400);
  println(Serial.list());  //list of available serial ports
  String portName = Serial.list()[1]; //replace 0 with whatever port you want to use.
   myPort = new Serial(this, portName, 9600);
}

byte LastData[] = {0,0,0,0};

void draw() {
    
  byte[] data = new byte[4];
  
  while (myPort.available() > 0) {
    
    myPort.readBytes(data);  // read serial port
    
    // Check the security value (57)
    if (data[0] == 57){
      
      // First graph
      stroke(255, 0, 0); 
      line(x-3, height/2-LastData[0], x, height/2 - data[1]); 
      
      // Second graph 
      stroke(0, 255, 0);
      line(x-3, height/2-LastData[1], x, height/2 - data[2]); 
      
      // Third graph
      stroke(0, 0, 255);
      line(x-3, height/2-LastData[2], x, height/2 - data[3]); 
      
      // If the cursor is at the end, reset
      if (x >=width) {
        x=0;
        background(0);
        stroke(255, 255, 255); 
        line(0, height/2, width, height/2);        
      }
          
      // Actualice last data
      LastData[0] = data[1];
      LastData[1] = data[2];
      LastData[2] = data[3];
                   
      x++;    // next point
          
    } // end if
        
  } // end while
      
} // end draw



