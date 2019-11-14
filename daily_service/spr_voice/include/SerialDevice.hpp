
#ifndef SERIAL_DEVICE_HPP
#define SERIAL_DEVICE_HPP

//#define SERIAL_DEVICE_DEBUG

#include <c++/5.4.0/iostream>
#include <fcntl.h>
#include <x86_64-linux-gnu/sys/ioctl.h>
#include <termios.h>
#include <x86_64-linux-gnu/sys/time.h>
#include <errno.h>


#define MAX_DEVICE_NAME_LENGTH 128
#define DEFAULT_READ_TIMEOUT_US 1000000

//list the io modes here
enum {IO_BLOCK_W_TIMEOUT,IO_NONBLOCK_POLL_W_DELAY_W_TIMEOUT,IO_BLOCK_WO_TIMEOUT,IO_NONBLOCK_WO_TIMEOUT,IO_BLOCK_W_TIMEOUT_W_TERM_SEQUENCE,
      IO_BLOCK_WO_TIMEOUT_W_TERM_SEQUENCE};

#define DEFAULT_IO_MODE IO_BLOCK_W_TIMEOUT
#define MAX_NUM_TERM_CHARS 128

class SerialDevice{
  public:

  SerialDevice();
  ~SerialDevice();

  int Connect(char * device, int speed);          //connect to the device and set the baud rate
  int Disconnect();                              //disconnect from the device
  int SetBaudRate(int baud);                     //set the baud rate
  
  bool IsConnected();

  int FlushInputBuffer();                        //flush input buffer

  int ReadChars(char * data, int byte_count, int timeout_us=DEFAULT_READ_TIMEOUT_US);       //read a number of characters
  int WriteChars(char * data, int byte_count, int delay_us=0);      //write a number of characters

  int Set_IO_BLOCK_W_TIMEOUT();
  int Set_IO_NONBLOCK_POLL_W_DELAY_W_TIMEOUT(int delay);
  int Set_IO_BLOCK_WO_TIMEOUT();
  int Set_IO_NONBLOCK_WO_TIMEOUT();
  int Set_IO_BLOCK_W_TIMEOUT_W_TERM_SEQUENCE(char * termSequence, int numTermChars, bool retTermSequence=true);
  int Set_IO_BLOCK_WO_TIMEOUT_W_TERM_SEQUENCE(char * termSequence, int numTermChars, bool retTermSequence=true);


  private:

  char _device[MAX_DEVICE_NAME_LENGTH];   //devince name
  speed_t _baud;                          //baud rate
  int _fd;                                //file descriptor
  bool _connected;                        //status
  bool _block;                            //block / non-block IO
  
  int _ioMode, _delay_us, _numTermChars;
  char _termSequence[MAX_NUM_TERM_CHARS];
  bool _retTermSequence;

  int _SetBlockingIO();                           //set blocking IO
  int _SetNonBlockingIO();                        //set non-blocking IO
  int _SpeedToBaud(int speed, speed_t & baud);         //convert integer speed to baud rate setting

  struct termios _oldterm,_newterm;       //terminal structs

};




//constructor
SerialDevice::SerialDevice(){
  _fd=-1;
  _connected=false;
  _block=true;
  _baud=B2400;
  
  _ioMode=IO_BLOCK_W_TIMEOUT;
  _delay_us=0;
  _numTermChars=0;
  _retTermSequence=false;
}

//destructor
SerialDevice::~SerialDevice(){
  Disconnect();
}

//connect to the device
int SerialDevice::Connect(char * device, int speed){

  //store the device name
  strncpy(_device,device,MAX_DEVICE_NAME_LENGTH);

  // Open the device
  if((_fd = open(_device, O_RDWR | O_NOCTTY)) < 0) {
		#ifdef SERIAL_DEVICE_DEBUG
    std::cerr << "SerialDevice::Connect: Error: Unable to open serial port" << std::endl;
		#endif
    return -1;
  }

  //update the connected flag
  _connected=true;
  
  //save current attributes so they can be restored after use
  if( tcgetattr( _fd, &_oldterm ) < 0 ){
		#ifdef SERIAL_DEVICE_DEBUG
		std::cerr<<"SerialDevice::Connect: Error: Unable to get old serial port attributes" << std::endl;
		#endif
    close(_fd);
    _connected=false;
		return -1;
  }
	
  //set up the terminal and set the baud rate
  if (SetBaudRate(speed)){
		#ifdef SERIAL_DEVICE_DEBUG
    std::cerr << "SerialDevice::Connect: Error: Unable to set baud rate" << std::endl;
		#endif
    close(_fd);
    _connected=false;
    return -1;
  }

  //set the default IO mode
  if (Set_IO_BLOCK_W_TIMEOUT()){
		#ifdef SERIAL_DEVICE_DEBUG
    std::cerr << "SerialDevice::Connect: Error: Unable to set the io mode" << std::endl;
		#endif
    close(_fd);
    _connected=false;
    return -1;
  }

  return 0;
}

//disconnect from the device
int SerialDevice::Disconnect(){
  //check whether we are connected to the device  
  if (!_connected){
    return 0;
  }  

  // Restore old terminal settings
  if(tcsetattr(_fd,TCSANOW,&_oldterm) < 0) {
		#ifdef SERIAL_DEVICE_DEBUG
    std::cerr << "SerialDevice::Disconnect: Failed to restore attributes!" << std::endl;
		#endif
  }
  
	// Actually close the device
	if(close(_fd) != 0) {
		#ifdef SERIAL_DEVICE_DEBUG
		std::cerr << "SerialDevice::Disconnect: Failed to close device!" << std::endl;
		#endif
		_connected=false;
		return -1;
	}
	
  _connected=false;
	return 0;
}

bool SerialDevice::IsConnected(){
  return _connected;
}


//convert speed (integer) to baud rate (speed_t)
int SerialDevice::_SpeedToBaud(int speed, speed_t & baud){
  switch (speed) {
    case 2400:
      baud=B2400;
      return 0;
    case 4800:
      baud=B4800;
      return 0;
    case 9600:
      baud=B9600;
      return 0;
    case 19200:
      baud=B19200;
      return 0;
    case 38400:
      baud=B38400;
      return 0;
    case 57600:
      baud=B57600;
      return 0;
    case 115200:
      baud=B115200;
      return 0;

    default:
			#ifdef SERIAL_DEVICE_DEBUG
      std::cerr << "SerialDevice::speedToBaud: ERROR: unknown baud rate" <<std::endl;
			#endif
      return -1;
  }
}

//set the terminal baud rate. Argument can be either an integer (ex. 115200) or speed_t (ex. B115200)
int SerialDevice::SetBaudRate(int speed){
  speed_t tempBaud;

  //check whether we are connected to the device
  if (!_connected){
		#ifdef SERIAL_DEVICE_DEBUG
    std::cerr << "SerialDevice::FlushInputBuffer: Error: not connected to the device" << std::endl;
		#endif
    return -1;
  }

  //convert the integer speed value to speed_t if needed
  if (_SpeedToBaud(speed,tempBaud)){
		#ifdef SERIAL_DEVICE_DEBUG
    std::cerr<<"SerialDevice::SetBaudRate: Error: bad baud rate" << std::endl;
		#endif
		return -1;
  }
  
  //get current port settings
  if( tcgetattr( _fd, &_newterm ) < 0 ){
		#ifdef SERIAL_DEVICE_DEBUG
		std::cerr<<"SerialDevice::SetBaudRate: Error: Unable to get serial port attributes" << std::endl;
		#endif
		return -1;
	}

	//cfmakeraw initializes the port to standard configuration. Use this!
	cfmakeraw( &_newterm );
	
  //set input baud rate
  if (cfsetispeed( &_newterm, tempBaud ) < 0 ){
		#ifdef SERIAL_DEVICE_DEBUG
    std::cerr<<"SerialDevice::SetBaudRate: Error: Unable to set baud rate" <<std::endl;
		#endif
		return -1;
  }
	
  //set output baud rate
  if (cfsetospeed( &_newterm, tempBaud ) < 0 ){
		#ifdef SERIAL_DEVICE_DEBUG
    std::cerr<<"SerialDevice::SetBaudRate: Error: Unable to set baud rate" <<std::endl;
		#endif
		return -1;
  }
	
	//set new attributes 
	if( tcsetattr( _fd, TCSAFLUSH, &_newterm ) < 0 ) {
		#ifdef SERIAL_DEVICE_DEBUG
		std::cerr<<"SerialDevice::SetBaudRate: Error: Unable to set serial port attributes" <<std::endl;
		#endif
		return -1;
	}
	
	//make sure queue is empty
	tcflush(_fd, TCIOFLUSH);

  //save the baud rate value
  _baud=tempBaud;

  return 0;
}


int SerialDevice::Set_IO_BLOCK_W_TIMEOUT(){
  //check whether we are connected to the device  
  if (!_connected){
		#ifdef SERIAL_DEVICE_DEBUG
    std::cerr << "SerialDevice::Set_IO_BLOCK_WO_TIMEOUT: Error: not connected to the device" << std::endl;
		#endif
    return -1;
  }
  
  if (_SetBlockingIO()){
    #ifdef SERIAL_DEVICE_DEBUG
    std::cerr << "SerialDevice::Set_IO_BLOCK_WO_TIMEOUT: Error: could not set blocking io" << std::endl;
    #endif
    return -1;
  }
  _ioMode=IO_BLOCK_W_TIMEOUT;
  _delay_us=0;
  _numTermChars=0;
  _retTermSequence=false;
  
  return 0;
}

int SerialDevice::Set_IO_BLOCK_WO_TIMEOUT(){
  //check whether we are connected to the device  
  if (!_connected){
		#ifdef SERIAL_DEVICE_DEBUG
    std::cerr << "SerialDevice::Set_IO_BLOCK_WO_TIMEOUT: Error: not connected to the device" << std::endl;
		#endif
    return -1;
  }
  
  if (_SetBlockingIO()){
    #ifdef SERIAL_DEVICE_DEBUG
    std::cerr << "SerialDevice::Set_IO_BLOCK_WO_TIMEOUT: Error: could not set blocking io" << std::endl;
    #endif
    return -1;
  }
  _ioMode=IO_BLOCK_WO_TIMEOUT;
  _delay_us=0;
  _numTermChars=0;
  _retTermSequence=false;
  
  return 0;
}

int SerialDevice::Set_IO_BLOCK_W_TIMEOUT_W_TERM_SEQUENCE(char * termSequence, int numTermChars, bool retTermSequence){
  //check whether we are connected to the device  
  if (!_connected){
		#ifdef SERIAL_DEVICE_DEBUG
    std::cerr << "SerialDevice::Set_IO_BLOCK_W_TIMEOUT_W_TERM_SEQUENCE: Error: not connected to the device" << std::endl;
		#endif
    return -1;
  }
  
  if (_SetBlockingIO()){
    #ifdef SERIAL_DEVICE_DEBUG
    std::cerr << "SerialDevice::Set_IO_BLOCK_W_TIMEOUT_W_TERM_SEQUENCE: Error: could not set blocking io" << std::endl;
    #endif
    return -1;
  }
  
  if (numTermChars < 1 || numTermChars > MAX_NUM_TERM_CHARS){
    std::cerr<<" IOMode::Set_IO_BLOCK_W_TIMEOUT_W_TERM_SEQUENCE: ERROR: bad number of chars: " <<numTermChars<<std::endl;
    return -1;
  }
  _ioMode=IO_BLOCK_W_TIMEOUT_W_TERM_SEQUENCE;
  _numTermChars=numTermChars;
  _retTermSequence=retTermSequence;
  memcpy(_termSequence,termSequence, numTermChars*sizeof(char));
  _delay_us=0;
  
  return 0;
}

int SerialDevice::Set_IO_NONBLOCK_WO_TIMEOUT(){
  //check whether we are connected to the device  
  if (!_connected){
		#ifdef SERIAL_DEVICE_DEBUG
    std::cerr << "SerialDevice::Set_IO_NONBLOCK_WO_TIMEOUT: Error: not connected to the device" << std::endl;
		#endif
    return -1;
  }
  
  if (_SetNonBlockingIO()){
    #ifdef SERIAL_DEVICE_DEBUG
    std::cerr << "SerialDevice::Set_IO_NONBLOCK_WO_TIMEOUT: Error: could not set non-blocking io" << std::endl;
    #endif
    return -1;
  }

  _ioMode=IO_NONBLOCK_WO_TIMEOUT;
  _delay_us=0;
  _numTermChars=0;
  _retTermSequence=false;
  
  return 0;
}

int SerialDevice::Set_IO_NONBLOCK_POLL_W_DELAY_W_TIMEOUT(int delay_us){
  //check whether we are connected to the device  
  if (!_connected){
		#ifdef SERIAL_DEVICE_DEBUG
    std::cerr << "SerialDevice::Set_IO_NONBLOCK_POLL_W_DELAY_W_TIMEOUT: Error: not connected to the device" << std::endl;
		#endif
    return -1;
  }
  
  if (_SetNonBlockingIO()){
    #ifdef SERIAL_DEVICE_DEBUG
    std::cerr << "SerialDevice::Set_IO_NONBLOCK_POLL_W_DELAY_W_TIMEOUT: Error: could not set non-blocking io" << std::endl;
    #endif
    return -1;
  }

  _ioMode=IO_NONBLOCK_POLL_W_DELAY_W_TIMEOUT; 
  _delay_us=delay_us;
  _numTermChars=0;
  _retTermSequence=false;
  
  return 0;
}


//set blocking terminal mode
int SerialDevice::_SetBlockingIO() {
  
  //check whether we are connected to the device  
  if (!_connected){
		#ifdef SERIAL_DEVICE_DEBUG
    std::cerr << "SerialDevice::SetBlockingIO: Error: not connected to the device" << std::endl;
		#endif
    return -1;
  }	

  // Read the flags
	int flags;
	if((flags = fcntl(_fd,F_GETFL)) < 0) {
		#ifdef SERIAL_DEVICE_DEBUG
		std::cerr << "SerialDevice::_setBlockingIO: unable to get device flags" << std::endl;
		#endif
		return -1;
	}
	
	// Set the new flags
	if(fcntl(_fd,F_SETFL,flags & (~O_NONBLOCK)) < 0) {
		#ifdef SERIAL_DEVICE_DEBUG
		std::cerr << "SerialDevice::_setBlockingIO: unable to set device flags" << std::endl;
		#endif
		return -1;
	}
	
  _block=true;
	return 0;
}

//set non-blocking terminal mode
int SerialDevice::_SetNonBlockingIO() {

  //check whether we are connected to the device  
  if (!_connected){
		#ifdef SERIAL_DEVICE_DEBUG
    std::cerr << "SerialDevice::SetNonBlockingIO: Error: not connected to the device" << std::endl;
		#endif
    return -1;
  }	

  // Read the flags
	int flags;
	if((flags = fcntl(_fd,F_GETFL)) < 0) {
		#ifdef SERIAL_DEVICE_DEBUG
		std::cerr << "SerialDevice::_setNonBlockingIO failed! - unable to retrieve device flags" << std::endl;
		#endif
		return -1;
	}
	
	// Set the new flags
	if(fcntl(_fd,F_SETFL,flags | O_NONBLOCK) < 0) {
		#ifdef SERIAL_DEVICE_DEBUG
		std::cerr << "SerialDevice::_setNonBlockingIO failed! - unable to set device flags" << std::endl;
		#endif
		return -1;
	}
	
  _block=false;
	return 0;
}

//delete all the data in the input buffer
int SerialDevice::FlushInputBuffer(){
  char c[1000];
  //check whether we are connected to the device
  if (!_connected){
		#ifdef SERIAL_DEVICE_DEBUG
    std::cerr << "SerialDevice::FlushInputBuffer: Error: not connected to the device" << std::endl;
		#endif
    return -1;
  }
  
  //TODO tcflush for some reason does not work.. 
  //tcflush(_fd, TCIFLUSH);
  
  bool block=_block;
  _SetNonBlockingIO();
  
  //read off all the chars
  while (read(_fd,c,1000) > 0){}
  
  if (block){
    _SetBlockingIO();
  }
  
  return 0;
}

//read characters from device
int SerialDevice::ReadChars(char * data, int byte_count, int timeout_us){
  //check whether we are connected to the device
  if (!_connected){
		#ifdef SERIAL_DEVICE_DEBUG
    std::cerr << "SerialDevice::ReadChars: Error: not connected to the device" << std::endl;
		#endif
    return -1;
  }  

  fd_set watched_fds;
  struct timeval timeout, start, end;
  int bytes_read_total = 0;
  int bytes_left = byte_count;
  int retval;
  int bytes_read;
  int charsMatched=0;

  switch (_ioMode){

    case IO_BLOCK_W_TIMEOUT:
			//set up for the "select" call
      FD_ZERO(&watched_fds);
      FD_SET(_fd, &watched_fds);
      timeout.tv_sec = timeout_us / 1000000;
      timeout.tv_usec = timeout_us % 1000000;


      while (bytes_left) {
        if ((retval = select(_fd + 1, &watched_fds, NULL, NULL, &timeout)) < 1) {	//block until at least 1 char is available or timeout
          if (retval < 0) {																												//error reading chars
						#ifdef SERIAL_DEVICE_DEBUG
            perror("SerialDevice::ReadChars");
						#endif
          }
          else {																																	//timeout
						#ifdef SERIAL_DEVICE_DEBUG
	          std::cerr << "SerialDevice::ReadChars: Error: timeout. #chars read= "<<bytes_read_total <<", requested= "<< byte_count << std::endl;
						#endif
          }
          return bytes_read_total;
        }
        bytes_read        = read(_fd, &(data[bytes_read_total]), bytes_left);
        
        if (bytes_read > 0){
          bytes_read_total += bytes_read;
          bytes_left       -= bytes_read;
        }
      }
      return bytes_read_total;

    

    case IO_NONBLOCK_POLL_W_DELAY_W_TIMEOUT:
      gettimeofday(&start,NULL);
      
      while (bytes_left) {
        bytes_read = read(_fd,&(data[bytes_read_total]),bytes_left);
        if ( bytes_read < 1){
          // If a time out then return false
		      gettimeofday(&end,NULL);
		      if((end.tv_sec*1000000 + end.tv_usec) - (start.tv_sec*1000000 + start.tv_usec) > timeout_us) {
						#ifdef SERIAL_DEVICE_DEBUG
			      std::cerr << "SerialDevice::ReadChars: Error: timeout. #chars read= "<<bytes_read_total <<", requested= "<< byte_count << std::endl;
						#endif
			      return bytes_read_total;
		      }
          usleep(_delay_us);
          continue;
		    }

        bytes_read_total += bytes_read;
        bytes_left       -= bytes_read;
      }
      return bytes_read_total;


    case IO_BLOCK_WO_TIMEOUT:
      while (bytes_left) {
        bytes_read = read(_fd,&(data[bytes_read_total]),bytes_left);
        if (bytes_read < 1){
          return -1;
        }
        
        bytes_read_total += bytes_read;
        bytes_left       -= bytes_read;
      }
      return bytes_read_total;

    case IO_NONBLOCK_WO_TIMEOUT:
      bytes_read = read(_fd,&(data[0]),bytes_left);
      if (bytes_read < 0) bytes_read=0;
      return bytes_read;
      
      
    case IO_BLOCK_W_TIMEOUT_W_TERM_SEQUENCE:
    
      //set up for the "select" call
      FD_ZERO(&watched_fds);
      FD_SET(_fd, &watched_fds);
      timeout.tv_sec = timeout_us / 1000000;
      timeout.tv_usec = timeout_us % 1000000;

      while (bytes_left) {
        if ((retval = select(_fd + 1, &watched_fds, NULL, NULL, &timeout)) < 1) {	//block until at least 1 char is available or timeout
          if (retval < 0) {																												//error reading chars
						#ifdef SERIAL_DEVICE_DEBUG
            perror("SerialDevice::ReadChars");
						#endif
          }
          else {																																	//timeout
						#ifdef SERIAL_DEVICE_DEBUG
	          std::cerr << "SerialDevice::ReadChars: Error: timeout. The terminating sequence has not been read"<< std::endl;
						#endif
          }
          return -1;
        }
        bytes_read = read(_fd, &(data[bytes_read_total]), 1);
        
        if (bytes_read==1){
          if (data[bytes_read_total]==_termSequence[charsMatched]){
            charsMatched++;
          }
          else {
            charsMatched=0;
          }
          
          //std::cerr<<data[bytes_read_total];
          bytes_read_total += bytes_read;
          bytes_left       -= bytes_read;
          
          if (charsMatched==_numTermChars){
            if (_retTermSequence){
              return bytes_read_total;
            }
            else {
              return bytes_read_total-_numTermChars;
            }
          }
        }
      }
      #ifdef SERIAL_DEVICE_DEBUG
      std::cerr << "SerialDevice::ReadChars: Error: Read too much data. The terminating sequence has not been read"<< std::endl;
      #endif
      return -1;

    default:
			#ifdef SERIAL_DEVICE_DEBUG
      std::cerr << "SerialDevice::ReadChars: Error: Bad io mode " << std::endl;
			#endif
      return -1;

  }
}

int SerialDevice::WriteChars(char * data, int byte_count, int delay_us){
  int bytes_written_total=0;
  int bytes_written;
  int bytes_left=byte_count;
    
  
  //check whether we are connected to the device  
  if (!_connected){
		#ifdef SERIAL_DEVICE_DEBUG
    std::cerr << "SerialDevice::ReadChars: Error: not connected to the device" << std::endl;
		#endif
    return -1;
  }
  
  if (delay_us==0){
    bytes_written_total=write(_fd,data,byte_count);
  }
  else{
    while (bytes_left){
      bytes_written=write(_fd,&(data[bytes_written_total]),1);
      if (bytes_written < 0){
				#ifdef SERIAL_DEVICE_DEBUG
        perror("SerialDevice::ReadChars");
				#endif
      }
      if (bytes_written < 1){
				#ifdef SERIAL_DEVICE_DEBUG
        std::cerr << "SerialDevice::WriteChars: Error: Could not write a char. #chars written= "<<bytes_written_total <<", requested= "<< byte_count << std::endl;
				#endif
        return bytes_written_total;
      }

      bytes_written_total += bytes_written;
      bytes_left       -= bytes_written;
      usleep(delay_us);
    }
  }  
  
  tcdrain(_fd);   //wait till all the data written to the file descriptor is transmitted

  return bytes_written_total;
}



#endif //SERIAL_DEVICE_HPP
