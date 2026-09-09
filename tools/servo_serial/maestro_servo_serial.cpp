


//This is the final program. Edit the line to set the target
// NOTE: The Maestro's serial mode must be set to "USB Dual Port".
//This can be done by changing the mode in ./MaestroControlCenter.
//In the application from the tab of serial setting
//Target represents the pulse width to transmit in units of quarter-microseconds
//A target value of 0 tells the Maestro to stop sending pulses to the servo.
//This servo can operate 180° when given a pulse signal ranging from 600usec to 2400usec.
//2400=-90degree /  9600= +90 (ideal which is not happening)
//target= 45*Degree + 6000



#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
 
#ifdef _WIN32
#define O_NOCTTY 0
#else
#include <termios.h>
#endif
 
// Gets the position of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
int maestroGetPosition(int fd, unsigned char channel)
{
  unsigned char command[] = {0x90, channel};
  if(write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }
   
  unsigned char response[2];
  if(read(fd,response,2) != 2)
  {
    perror("error reading");
    return -1;
  }
   
  return response[0] + 256*response[1];
}
 
// Sets the target of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
// The units of 'target' are quarter-microseconds.
int maestroSetTarget(int fd, unsigned char channel, unsigned short target)
{
  unsigned char command[] = {0x84, channel, target & 0x7F, target >> 7 & 0x7F};//bit shifting >> 7 to the right)
  if (write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }
  return 0;
}
 
int main()
{
  // Open the Maestro's virtual COM port.
  const char * device = "/dev/ttyACM0";  // Linux
  int fd = open(device, O_RDWR | O_NOCTTY);
  if (fd == -1)
  {
    perror(device);
    return 1;
  }
 
#ifndef _WIN32
  struct termios options;
  tcgetattr(fd, &options);
  options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  options.c_oflag &= ~(ONLCR | OCRNL);
  tcsetattr(fd, TCSANOW, &options);
#endif
   
  int position = maestroGetPosition(fd, 0);
  printf("Current position is %d.\n", position);
  int degree = 39 ; //chose the target in degree between -45 and +45
  int target = ( degree*45 ) +  6000;
  printf("Setting target to %d (%d us).\n", target, target/4);
  maestroSetTarget(fd, 0, target);
   
  close(fd);
  return 0;
}
