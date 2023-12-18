
import processing.serial.*;

Serial myPort;  // Create object from Serial class
byte[] rx_buffer = new byte[720];      // Data received from the serial port
int[]  lidar_value = new int[360];

void setup() 
{
  size(720, 720);
  //String portName = Serial.list()[0];
  myPort = new Serial(this, "COM13", 38400);
  myPort.buffer(720);
  myPort.write(0xAA); //<>//
}

void draw()
{
  if ( myPort.available() > 0) {  // If data is available,       // read it and store it in val
  }
  background(255);             // Set background to white
  circle(360,360,60);
  float origine_x = 360;
  float origine_y = 360;
  float angle_btwn_line = PI/180;
  for(int i=0; i<360 ; i++){
    float x_A = cos(angle_btwn_line*i)*30;
    float y_A = sin(angle_btwn_line*i)*30;
    float x_B = cos(angle_btwn_line*i)*(abs(lidar_value[i]/2)) + x_A;
    float y_B = sin(angle_btwn_line*i)*(abs(lidar_value[i]/2)) + y_A;
    circle(origine_x + x_B, origine_y + y_B,  5);
    //line(origine_x + x_A, origine_y + y_A, origine_x + x_B, origine_y + y_B);
  }
}

void serialEvent(Serial myPort){
  rx_buffer = myPort.readBytes();
  for(int i=0;i<720;i++){
    if(i%2==0){
      lidar_value[i>>1] = rx_buffer[i];
      lidar_value[i>>1] = lidar_value[i>>1]<<8;
    }else{
      lidar_value[i>>1] = lidar_value[i>>1] + rx_buffer[i];
    }
  }
  myPort.write(0xAA);
}
