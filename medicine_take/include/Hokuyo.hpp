
#ifndef HOKUYO_HPP
#define HOKUYO_HPP

/* Implementation dependencies */
#include <c++/5.4.0/iostream>
#include <c++/5.4.0/string>
#include <c++/5.4.0/sstream>
#include <termios.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <time.h>
#include <errno.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
#include <netdb.h>
#include <ctype.h>
#include <pthread.h>
#include <x86_64-linux-gnu/sys/ioctl.h>
#include <x86_64-linux-gnu/sys/types.h>
#include <x86_64-linux-gnu/sys/stat.h>
#include <x86_64-linux-gnu/sys/socket.h>

#include "SerialDevice.hpp"


/** The default number of tries to communicate with the Hokuyo before giving up. */
#define DEFAULT_NUM_TRIES 5
#define SWITCH_MODE_TRIES 10
#define SEND_STOP_TRIES 10

/** Defines the initial baud of the Hokuyo */
#define HOKUYO_INIT_BAUD_RATE 115200

#define HOKUYO_READ_LINE_TIMEOUT 0.2
#define HOKUYO_READ_PACKET_TIMEOUT 0.050
#define HOKUYO_NUM_TEST_BAUD_RETRIES 3
#define HOKUYO_MAX_NUM_POINTS 3000

//actually 769 is the max number of points for range only scan for URG-04LX, 
//but when range+intensity+agc is requested, it goes up to 771
#define HOKUYO_MAX_NUM_POINTS_URG_04LX 771 
#define HOKUYO_MAX_NUM_POINTS_UTM_30LX 1081*2

#define HOKUYO_MAX_PACKET_LENGTH 15000 
#define HOKUYO_MAX_DATA_LENGTH 10000
#define HOKUYO_MAX_LINE_LENGTH 100

#define HOKUYO_TYPE_URG_04LX 0 
#define HOKUYO_TYPE_UTM_30LX 1 
#define HOKUYO_TYPE_URG_04LX_STRING "SOKUIKI Sensor URG-04LX"
//#define HOKUYO_TYPE_UTM_X001S_STRING "SOKUIKI Sensor TOP-URG UTM-X001S"
#define HOKUYO_TYPE_UTM_30LX_STRING "SOKUIKI Sensor TOP-URG UTM-30LX"

#define HOKUYO_AGC_1 219
#define HOKUYO_AGC_0 220
#define HOKUYO_RANGE_INTENSITY_1_AGC_1 221
#define HOKUYO_RANGE_INTENSITY_0_AGC_0 222
#define HOKUYO_RANGE_INTENSITY_AV_AGC_AV 223

#define HOKUYO_INTENSITY_0 237
#define HOKUYO_INTENSITY_1 238
#define HOKUYO_INTENSITY_AV 239

#define HOKUYO_RANGE_INTENSITY_0 253
#define HOKUYO_RANGE_INTENSITY_1 254
#define HOKUYO_RANGE_INTENSITY_AV 255

#define HOKUYO_TOP_URG_RANGE_INTENSITY 256

//these are the allowed scan names
#define HOKUYO_RANGE_STRING "range" //for 04LX and 30LX

//for 04LX only:
#define HOKUYO_RANGE_INTENSITY_AV_STRING "range+intensityAv"
#define HOKUYO_RANGE_INTENSITY_0_STRING "range+intensity0"
#define HOKUYO_RANGE_INTENSITY_1_STRING "range+intensity1"
#define HOKUYO_INTENSITY_AV_STRING "intensityAv"
#define HOKUYO_INTENSITY_0_STRING "intensity0"
#define HOKUYO_INTENSITY_1_STRING "intensity1"
#define HOKUYO_RANGE_INTENSITY_AV_AGC_AV_STRING "range+intensityAv+AGCAv"
#define HOKUYO_RANGE_INTENSITY_0_AGC_0_STRING "range+intensity0+AGC0"
#define HOKUYO_RANGE_INTENSITY_1_AGC_1_STRING "range+intensity1+AGC1"
#define HOKUYO_AGC_0_STRING "AGC0"
#define HOKUYO_AGC_1_STRING "AGC1"

//for 30LX only:
#define HOKUYO_TOP_URG_RANGE_INTENSITY_STRING "top_urg_range+intensity"



#define HOKUYO_SCAN_REGULAR 0
#define HOKUYO_SCAN_SPECIAL_ME 1

//character encoding
#define HOKUYO_2DIGITS 2
#define HOKUYO_3DIGITS 3

#define HOKUYO_SCIP1 0
#define HOKUYO_SCIP20 1

#define HOKUYO_SERIAL 0
#define HOKUYO_TCP 1

#define HOKUYO_SDS_HTTP_PORT 80
#define HOKUYO_SDS_START_DEVICE_PORT 8000
#define HOKUYO_SDS_CONFIG_CHANGE_CONFIRM_MAX_SIZE 512

//when serial device server returns an html document as a confirmation of accepted changes, the size is exactly 396
//if the size is different, then something went wrong
#define HOKUYO_SDS_CONFIG_CHANGE_CONFIRM_SIZE 396

#define HOKUYO_TURN_LASER_OFF false
#define HOKUYO_TURN_LASER_ON true
#define HOKUYO_NUM_STOP_LASER_RETRIES 5
#define HOKUYO_LASER_STOP_DELAY_US 50000
#define HOKUYO_TEST_BAUD_RATE_DELAY_US 100000


/*!
 * \brief A general class for interfacing w/ Hokuyo urg 04-lx laser range finders
 *
 */
class Hokuyo
{

public:

  // mutex for writing / reading data
  
  pthread_mutex_t m_HokuyoDataMutex;
  pthread_cond_t m_HokuyoDataCond;

  // Device type (e.g URG-04LX or UTM-X001S)
  int _type; 

  // Constructor
  Hokuyo();

  // Destructor
  ~Hokuyo();
  

  // Initializes the Hokuyo
  int Connect(std::string dev_str, const int baud_rate, const int mode);
  
  // send a specific command to hokuyo and count number of bytes in response
  //int sendCmdAndCountResponseBytes(char * command, int num);
  
  // Disconnect from Hokuyo
  int Disconnect();
  
  // send the command to turn on/off the laser
  int LaserOn();
  int LaserOff();

  // Get a scan from Hokuyo
  int GetScan(unsigned int * range, int & n_range, int scanStart, int scanEnd, int scanSkip, int encoding, int scanType, int numScans, bool * scanReady=NULL);

  //int TurnOnContStreaming(int start, int stop, int skip);
  
  int CreateScanRequest(int scanStart, int scanEnd, int scanSkip, int encoding, int scanType, int numScans, char * req, bool & needToGetConfirm);
  int ConfirmScan(char * req, int timeout_us);
  int RequestScan(char * req);
  
  int FindPacketStart();
  
  // SerialDevice object that's used for communication
  SerialDevice sd;

public:

  // A path to the device at which the sick can be accessed.
  std::string _device;
  
  int _readPacket(char * data, int & packet_length, int timeout_us);
  int _measurePacketLength();
  int _extractPacket(char * full_packet, char * extracted_packet, int packet_length);
  int _decodePacket(char * extracted_packet, unsigned int * extracted_data, int extracted_length, int encoding);
  int _checkLineChecksum(char * line, int max_length);
  void _printStrings(char * exp, char * got, int num);
  void _printLine(char * line, int line_len);
  int _laserOnOff(bool turnOn);
  
    
  // Hokuyo mode (SCIP1 or SCIP2.0)
  int _mode;
  
  // is the scanner streaming data?
  bool _streaming;
  
  // Communication protocol for the device: serial port or TCP (serial device server)
  int _comProtocol;

  // The baud rate at which to communicate with the Hokuyo
  int _baud;

  // Sets the baud rate for communication with the Hokuyo.
  int _setBaudRate(const int baud_rate);

  /** Tests communication with the Hokuyo at a particular baud rate. */
  int _testBaudRate(const int baud_rate);

  /** Gets the status of the Hokuyo. */
  int _getStatusData();
  int _getStatusData(int mode);
  
  /** Reads a line and returns number of chars read */
  int _readLine(char * line, int max_chars, int timeout_us, bool checkSum);
  
  /** Set the SCIP2.0 protocol on hokuyo */
  int _setSCIP20();
 };

#ifdef MATLAB_MEX_FILE
#include "mex.h"
void createEmptyOutputMatrices(int nlhs, mxArray * plhs[]);
int createOutput(int sensorType, char * scanType, unsigned int * data, uint16_t n_points, int nlhs, mxArray * plhs[]);
int parseScanParams(int sensorType,char * scanType,const mxArray * prhs[],int indexStart, int * start, int * stop, int * skip, int * encoding, int * specialScan);
#endif //MATLAB_MEX_FILE

int getScanTypeAndSkipFromName(int sensorType, char * scanTypeName, int * skip, int * scanType);
int getNumOutputArgs(int sensorType, char * scanTypeName);



// Constructor

//connect using serial or USB connection
Hokuyo::Hokuyo(){
  _type=HOKUYO_TYPE_URG_04LX;
  _baud=0;
  _comProtocol=HOKUYO_SERIAL;
  _device=std::string("Unknown");
  _mode=HOKUYO_SCIP20;
  _streaming=false;
}


Hokuyo::~Hokuyo() {

	if(sd.Disconnect())
		std::cerr << "Hokuyo::~Hokuyo - _closeTerm() failed to close terminal" <<std::endl;
}

int Hokuyo::Connect(std::string dev_str, const int baud_rate, const int mode) {

  _device=dev_str;
  //connect to the device
  if(sd.Connect((char *)(_device.c_str()),baud_rate)) {
		std::cerr << "Hokuyo::Connect - Connect() failed!" << std::endl;
		return -1;
	}
  
  //find out what rate the sensor is currently running at. _baud will be set to the current baud rate.
	if(!_testBaudRate(115200)) {
		std::cout << "Hokuyo baud rate is 115200bps..." << std::endl;
	} 
	else if(!_testBaudRate(19200) == 0) {
		std::cout << "Hokuyo baud rate is 19200bps..." << std::endl;
	}
  else if(!_testBaudRate(38400) == 0) {
		std::cout << "Hokuyo baud rate is 38400bps..." << std::endl;
	}
	else {
		std::cerr << "Hokuyo::Connect - failed to detect baud rate!" << std::endl;
    sd.Disconnect();
		return -1;
	}
  
	fflush(stdout);
	
	// Make sure requested baud isn't already set
	if(_baud == baud_rate) {
		std::cout << "Hokuyo::Connect - Hokuyo is already operating @ " << baud_rate << std::endl;
	} else {
		std::cout << "Hokuyo::Connect: Attempting to set requested baud rate..." << std::endl;
		if (!_setBaudRate(baud_rate)) {
			std::cout << "Hokuyo::Connect: Operating @ " <<  _baud << std::endl;
		} else {
			std::cerr << "Hokuyo::Connect - Unable to set the requested Hokuyo baud rate of " <<  baud_rate << std::endl;
      sd.Disconnect();    
			return -1;
		}
	}
	
  // get the status data from sensor to determine its type
  if(_getStatusData()){
    std::cerr << "Hokuyo::Connect - could not get status data from sensor" << std::endl;
    sd.Disconnect();   
		return -1;
  }
  
	std::cout << "Hokuyo::Connect: Turning the laser on" <<std::endl;
	if (LaserOn()){
		std::cerr << "Hokuyo::Connect: was not able to turn the laser on" << std::endl;
    sd.Disconnect();
		return -1;
	}
	
  pthread_mutex_init(&m_HokuyoDataMutex,NULL);
  pthread_cond_init(&m_HokuyoDataCond,NULL);

	std::cout << "Hokuyo::Connect: Initialization Complete." <<std::endl <<"-----------------------------------------"<<std::endl<<std::endl;
	return 0;
}

int Hokuyo::Disconnect() {
	
  if (!sd.IsConnected()){
    return 0;
  }
  
  if (!LaserOff()){
    std::cout<<"Hokuyo::Disconnect: Laser has been shut off"<<std::endl;
  }
  else {
    std::cerr<<"Hokuyo::Disconnect: ERROR: Wasn't able to shut off the laser"<<std::endl;
  }

  pthread_mutex_destroy(&m_HokuyoDataMutex);
  pthread_cond_destroy(&m_HokuyoDataCond);

  sd.Disconnect();
	return 0;
}

int Hokuyo::FindPacketStart(){
  static char data[HOKUYO_MAX_PACKET_LENGTH];
  char seq[2]={0x0A,0x0A};   //two LFs
  
  if (!sd.IsConnected()){
		std::cerr << "Hokuyo::_findPacketStart: not connected to the sensor!" << std::endl;
		return -1;
	}
  
  sd.Set_IO_BLOCK_W_TIMEOUT_W_TERM_SEQUENCE(seq,2,true);
  int num_chars=sd.ReadChars(data,HOKUYO_MAX_PACKET_LENGTH);
  
  if ( num_chars< 1 ){
    std::cerr << "Hokuyo::_findPacketStart: Error: terminating character was not read"<<std::endl;
    return -1;
  }
  return 0;
}

int Hokuyo::_measurePacketLength(){
  static char dataIn[HOKUYO_MAX_PACKET_LENGTH];
  char seq[2]={0x0A,0x0A};   //two LFs
  
  if (!sd.IsConnected()){
		std::cerr << "Hokuyo::_readWholeDataPacket: not connected to the sensor!" << std::endl;
		return -1;
	}
  
  sd.Set_IO_BLOCK_W_TIMEOUT_W_TERM_SEQUENCE(seq,2,true);
  
  int num_chars=sd.ReadChars(dataIn,HOKUYO_MAX_PACKET_LENGTH);
  
  if ( num_chars< 1){
    std::cerr << "Hokuyo::_measurePacketLength: Error: terminating character was not read"<<std::endl;
    return -1;
  }
  return num_chars;
}

int Hokuyo::_readPacket(char * data, int & packet_length, int timeout_us){
  if (!sd.IsConnected()){
		std::cerr << "Hokuyo::_readPacket: not connected to the sensor!" << std::endl;
		return -1;
	}

  if (packet_length > 0){
    //std::cout <<"Hokuyo::_readPacket: using previous packet length" <<std::endl;
    sd.Set_IO_BLOCK_W_TIMEOUT();  
    if (sd.ReadChars(data,packet_length,timeout_us) != packet_length){
      std::cerr << "Hokuyo::_readPacket: could not read the number of expected characters!" << std::endl;
      return -1;
    }
  }
  else {
    //std::cout <<"Hokuyo::_readPacket: need to measure packet length" <<std::endl;
    char term_sequence[2]={0x0A,0x0A};
    
    sd.Set_IO_BLOCK_W_TIMEOUT_W_TERM_SEQUENCE(term_sequence, 2, true);
    int chars_read=sd.ReadChars(data,HOKUYO_MAX_PACKET_LENGTH,timeout_us);
    if (chars_read < 0){
      std::cerr << "Hokuyo::_readPacket: could not read the terminating character while trying to determine the packet length!" << std::endl;
      return -1;
    }
    packet_length=chars_read;
  }
  
  return 0;
}

int Hokuyo::_extractPacket(char * full_packet, char * extracted_packet, int packet_length){
  int lenNLFC=0;     //length of data without LFs and checksum
  int lineLength;
  int lineCount=0;
  
  
  if (packet_length < 1){
    std::cerr<<"Hokuyo::_extractPacket: bad length!!!"<<std::endl;
    return -1;
  }
  
  lineLength=_checkLineChecksum(full_packet, HOKUYO_MAX_LINE_LENGTH);
  
  //line length will be zero at the end of packet, when the last LF is read
  while (lineLength > 0){
    lineCount++;
  
    //copy the data
    memcpy(extracted_packet,full_packet,lineLength);
    
    extracted_packet+=lineLength;
    full_packet+=(lineLength+2);     //advance the pointer 2 chars ahead to skip the checksum and endline char
    lenNLFC+=lineLength;
    
    //check the next line
    lineLength=_checkLineChecksum(full_packet, HOKUYO_MAX_LINE_LENGTH);
  }
  if (lineLength < 0){
    std::cerr<<"Hokuyo::_extractPacket: error extracting a line!!! line count="<<lineCount<<std::endl;
    return -1;
  }
  
  //std::cerr<<"Hokuyo::_extractPacket: line count="<<lineCount<<std::endl;
  return lenNLFC;
}

int Hokuyo::_decodePacket(char * extracted_packet, unsigned int * extracted_data, int extracted_length, int encoding){
  int data_length=0;
  
  if (extracted_length < 1){
    std::cerr<<"Hokuyo::_decodePacket: bad length!!!"<<std::endl;
    return -1;
  }
  
  switch (encoding){
  
    case HOKUYO_2DIGITS:
      data_length = 0;
      for(int i=0;i<extracted_length-1;i++){   //FIXME: do we really need -1 here?
        *extracted_data=((extracted_packet[i] - 0x30)<<6) + extracted_packet[i+1] - 0x30;
        i++;
        data_length++;
        if (data_length > HOKUYO_MAX_NUM_POINTS){
          std::cerr << "Hokuyo::_decodePacket: returned too many data points" << std::endl;
          return -1;
        }
      }
      break;
    
    case HOKUYO_3DIGITS:
      data_length = 0;
      for(int i=0;i<extracted_length-1;i++){   //FIXME: do we really need -1 here?
        *extracted_data=((extracted_packet[i] - 0x30)<<12) + ((extracted_packet[i+1] - 0x30)<<6) + extracted_packet[i+2] - 0x30;
        i+=2;
        extracted_data++;
        data_length++;
        if (data_length > HOKUYO_MAX_NUM_POINTS){
          std::cerr << "Hokuyo::_decodePacket: returned too many data points" << std::endl;
          return -1;
        }
      }
      break;
    default:
      std::cerr << "Hokuyo::_decodePacket: bad encoding" << std::endl;
      return -1;
  }
  
  return data_length;
}


//verify the checksum of the line, which is the last character of the line
int Hokuyo::_checkLineChecksum(char * line, int max_length){
  
  char * end_line;
  int line_length;
  char sum=0;
  
  end_line=(char *)memchr((void *)line,0x0A, max_length);
  if (end_line == NULL){
    std::cerr<< "Hokuyo::_checkLineChecksum: Error: end of line is not found"<<std::endl;
    return -1;
  }
  
  line_length=end_line-line;
  
  //sanity check
  if ( (line_length < 0) || (line_length > HOKUYO_MAX_LINE_LENGTH)){
    std::cerr<< "Hokuyo::_checkLineChecksum: Error: bad line length"<<std::endl;
    return -1;
  }
  
  //empty line
  if (line_length==0){
    return 0;
  }
  
  //compute the checksum up to the checksum character
  for (int j=0;j<line_length-1;j++){
    sum+=line[j];
  }
  
  //encode the sum
  sum = (sum & 0x3f) + 0x30;
  
  if (sum!=line[line_length-1]){
    std::cerr<<"Hokuyo::_checkLineChecksum:Checksum ERROR!!!"<<std::endl;
    //std::cerr<<"line_len="<<line_length<<std::endl;
    //std::cerr<<"line: "<<line<<std::endl;
    //std::cerr<<"prev line: "<<std::endl;
    //std::cerr<<(line-100)<<std::endl;
    
    return -1;
  }
  
  return (line_length-1);
}

//read one line from the sensor and optionally verify the checksum
//max specifies the maximum expected length of the line
int Hokuyo::_readLine(char * line,int max_chars, int timeout_us,bool checkSum){
	char seq[1]={0x0A};   //LF
  if (!sd.IsConnected()){
		std::cerr << "Hokuyo::_readLine: not connected to the sensor!" << std::endl;
		return -1;
	}
  
  sd.Set_IO_BLOCK_W_TIMEOUT_W_TERM_SEQUENCE(seq,1,true);
  
  int num_chars=sd.ReadChars(line,max_chars, timeout_us);
  
  if ( num_chars< 1){
    std::cerr << "Hokuyo::_readLine: Error: terminating character was not read"<<std::endl;
    return -1;
  }
  
   //empty line, therefore return
	if (num_chars==1){
    if (line[0]==0x0A){
      return 0;
    }
    else{
      std::cerr << "Hokuyo::_readLine: Error: read one char and it was not an endl"<<std::endl;
      return -1;
    }
  }
  
  if (!checkSum){
    return num_chars-1;
  }
  
  
  if (_checkLineChecksum(line, num_chars) != num_chars-2){
    std::cerr << "Hokuyo::_readLine: Error: unknown error"<<std::endl;
    return -1;
  }
  
  return (num_chars-2);
}

int Hokuyo::LaserOn(){
  if (LaserOff()){
    return -1;
  }
  
  if (_laserOnOff(HOKUYO_TURN_LASER_ON)){
    std::cerr<<"Hokuyo::LaserOn: ERROR: Wasn't able to turn on the laser"<<std::endl;
    return -1;
  }
  std::cout<<"Hokuyo::LaserOn: Laser has been turned on"<<std::endl;
  return 0;
}

int Hokuyo::LaserOff(){
  for (int i=0;i<HOKUYO_NUM_STOP_LASER_RETRIES; i++){
    if (!_laserOnOff(HOKUYO_TURN_LASER_OFF)){
      std::cout<<"Hokuyo::LaserOff: Laser has been shut off"<<std::endl;
      return 0;
    }
    else if (i==HOKUYO_NUM_STOP_LASER_RETRIES-1){
      std::cerr<<"Hokuyo::LaserOff: ERROR: Wasn't able to shut off the laser"<<std::endl;
    }
    usleep(HOKUYO_LASER_STOP_DELAY_US);
  }
  return -1;
}


//turn on/off the laser
int Hokuyo::_laserOnOff(bool turnOn){
  char line[HOKUYO_MAX_LINE_LENGTH];
		
  if (!sd.IsConnected()){
		std::cerr << "Hokuyo::_laserOnOff: not connected to the sensor!" << std::endl;
		return -1;
	}
  
  char cmd[3];
  char resp1[2];
  char resp2[2];
  
  if (turnOn){
    memcpy(cmd,"BM\n",3);
    memcpy(resp1,"00",2);
    memcpy(resp2,"02",2);
  }
  else {
    memcpy(cmd,"QT\n",3);
    memcpy(resp1,"00",2);
    memcpy(resp2,"00",2);
  }
  
  sd.FlushInputBuffer();
  
  //turn the laser on or off
  if(sd.WriteChars(cmd,3) != 3){
    std::cerr << "Hokuyo::_laserOnOff: could not write command" << std::endl;
    return -1;
  }
		
  //read back the echo of the command
  int timeout_us=200000;
  int line_len=_readLine(line,HOKUYO_MAX_LINE_LENGTH,timeout_us,false);
  if(line_len < 0){
    std::cerr << "Hokuyo::_laserOnOff: could not read line 1" << std::endl;
    return -1;
  }
  
  if(strncmp(cmd,line,line_len)){
    std::cerr << "Hokuyo::_laserOnOff: response does not match (line 1):" << std::endl;
    _printStrings(cmd,line,line_len);
    return -1;
  }
		
  //read back the status response
  line_len=_readLine(line,HOKUYO_MAX_LINE_LENGTH,timeout_us,true);
  if(line_len < 0){
    std::cerr << "Hokuyo::LaserOnOff: could not read line 2" << std::endl;
    return -1;
  }
 
  if( (strncmp(resp1,line,line_len)!=0) && (strncmp(resp2,line,line_len)!=0)){
    std::cerr << "Hokuyo::LaserOnOff: response does not match (line 2):" << std::endl;
    _printStrings(resp1,line,line_len);
    _printStrings(resp2,line,line_len);
    return -1;
  }
  
  //read off the LF
  line_len=_readLine(line,HOKUYO_MAX_LINE_LENGTH,timeout_us,false);
  if(line_len < 0){
    std::cerr << "Hokuyo::LaserOnOff: could not read line 3" << std::endl;
    return -1;
  }
  
  if (!turnOn){
    _streaming=false;
  }

  
  return 0;
}

void Hokuyo::_printStrings(char * exp, char * got, int num){
  std::cerr << "Expected: ";
  for (int i=0;i<num;i++) {
    if (exp[i]==0x0A){
      break;
    }
    std::cerr<<exp[i];
  }
  
  std::cerr <<std::endl<< "Got: ";
  for (int i=0;i<num;i++) std::cerr<<got[i];
  std::cerr <<std::endl;
}

int Hokuyo::CreateScanRequest(int scanStart, int scanEnd, int scanSkip, int encoding, int scanType, int numScans, char * req, bool & needToReadOffCmdAndStatus){


  //error checking
  if (scanStart < 0){
    std::cerr << "Hokuyo::CreateRequest: ERROR: bad scanStart" <<std::endl;
    return -1;
  }
  
  
  if (scanEnd < 0){
    std::cerr << "Hokuyo::CreateRequest: ERROR: bad scanEnd" <<std::endl;
    return -1;
  }
  
  if ((_type == HOKUYO_TYPE_UTM_30LX) && (scanEnd > HOKUYO_MAX_NUM_POINTS_UTM_30LX)){
    std::cerr << "Hokuyo::CreateRequest: ERROR: scanEnd is greater than maximum number of points for UTM-30LX" <<std::endl;
    std::cerr << "Maximum= "<<HOKUYO_MAX_NUM_POINTS_UTM_30LX<<", requested= "<<scanEnd<<std::endl;
    return -1;
  }
  
  if ((_type == HOKUYO_TYPE_URG_04LX) && (scanEnd > HOKUYO_MAX_NUM_POINTS_URG_04LX)){
    std::cerr << "Hokuyo::CreateRequest: ERROR: scanEnd is greater than maximum number of points for URG-04LX" <<std::endl;
    std::cerr << "Maximum= "<<HOKUYO_MAX_NUM_POINTS_URG_04LX<<", requested= "<<scanEnd<<std::endl;
    return -1;
  }
  
  if (scanEnd <= scanStart){
    std::cerr << "Hokuyo::CreateRequest: ERROR: scanEnd < scanStart" <<std::endl;
    return -1;
  }
  
  if ((numScans < 0) || (numScans > 99)){
    std::cerr << "Hokuyo::CreateRequest: ERROR: bad numScans" <<std::endl;
    return -1;
  }
  
  
  if ((_type == HOKUYO_TYPE_UTM_30LX) && (encoding == HOKUYO_2DIGITS)){
    std::cerr << "Hokuyo::CreateRequest: ERROR: UTM-30LX does not support 2-digit data mode." <<std::endl;
    return -1;
  }

  if (numScans==1){
    if (scanType==HOKUYO_SCAN_REGULAR){  
      if (encoding == HOKUYO_3DIGITS){
        sprintf(req,"GD%04d%04d%02x\n",scanStart,scanEnd,scanSkip);
        needToReadOffCmdAndStatus=false;
      }
      else if (encoding == HOKUYO_2DIGITS){
        sprintf(req,"GS%04d%04d%02x\n",scanStart,scanEnd,scanSkip);
        needToReadOffCmdAndStatus=false;
      }
      else {
        std::cerr << "Hokuyo::CreateScanRequest: ERROR: invalid selection of character encoding:"<<encoding << std::endl;
        return -1;
      }
    } else if ((scanType==HOKUYO_SCAN_SPECIAL_ME) && (_type == HOKUYO_TYPE_UTM_30LX) ) {
      if (encoding == HOKUYO_3DIGITS){
        sprintf(req,"ME%04d%04d%02x0%02d\n",scanStart,scanEnd,scanSkip,numScans);
        needToReadOffCmdAndStatus=true;
      }
      else {
        std::cerr << "Hokuyo::CreateScanRequest: ERROR: invalid selection of character encoding :"<<encoding <<" not supported by UTM_30LX"  << std::endl;
        return -1;
      }
    }
    else {
      std::cerr << "Hokuyo::CreateScanRequest: invalid scan type :"<<scanType <<" not supported by the sensor" << std::endl;
      return -1;
    }
  }
  else if (numScans==0){
    needToReadOffCmdAndStatus=true;
    if (scanType==HOKUYO_SCAN_REGULAR){  
      if (encoding == HOKUYO_3DIGITS){
        sprintf(req,"MD%04d%04d%02x0%02d\n",scanStart,scanEnd,scanSkip,numScans);
      }
      else if (encoding == HOKUYO_2DIGITS){
        sprintf(req,"MS%04d%04d%02x0%02d\n",scanStart,scanEnd,scanSkip,numScans);
      }
      else {
        std::cerr << "Hokuyo::CreateScanRequest: ERROR: invalid selection of character encoding:"<<encoding << std::endl;
        return -1;
      }
    } else if ((scanType==HOKUYO_SCAN_SPECIAL_ME) && (_type == HOKUYO_TYPE_UTM_30LX) ) {
      if (encoding == HOKUYO_3DIGITS){
        sprintf(req,"ME%04d%04d%02x0%02d\n",scanStart,scanEnd,scanSkip,numScans);
      }
      else {
        std::cerr << "Hokuyo::CreateScanRequest: ERROR: invalid selection of character encoding :"<<encoding <<" not supported by UTM_30LX"  << std::endl;
        return -1;
      }
    }
    else {
      std::cerr << "Hokuyo::CreateScanRequest: invalid scan type :"<<scanType <<" not supported by the sensor" << std::endl;
      return -1;
    }
  }
  else {
    std::cerr << "Hokuyo::CreateScanRequest: invalid number of scans :"<< numScans<< std::endl;
    return -1;
  }
  
  return 0;
}


int Hokuyo::RequestScan(char * req){
    
  if (!sd.IsConnected()){
		std::cerr << "Hokuyo::RequestScan: ERROR: not connected to the sensor!" << std::endl;
		return -1;
	}  
  
  sd.FlushInputBuffer();

  //write the data request command
  int reqLen = strlen(req);
  if(sd.WriteChars(req,reqLen) != reqLen){
    std::cerr << "Hokuyo::RequestScan: ERROR: could not write command" << std::endl;
    return -1;
  }
  
  return 0;
}


int Hokuyo::ConfirmScan(char * req, int timeout_us){
  char line[HOKUYO_MAX_LINE_LENGTH];
  int line_len;    
  //read back the echo of the command    
  line_len=_readLine(line,HOKUYO_MAX_LINE_LENGTH,timeout_us,false);
  if(line_len < 0){
    std::cerr << "Hokuyo::ConfirmScan: ERROR: could not read echo of the command (line 1)" << std::endl;
    return -1;
  }
  
  if(strncmp(req,line,line_len)){
    std::cerr << "Hokuyo::ConfirmScan: ERROR: response does not match the echo of the command (line 1):"<<std::endl;
    _printStrings(req,line,line_len);
    return -1;
  }
  
  //read back the status response
  line_len=_readLine(line,HOKUYO_MAX_LINE_LENGTH,timeout_us,true);
  if(line_len < 0){
    std::cerr << "Hokuyo::ConfirmScan: ERROR: could not read status response(line 2)" << std::endl;
    return -1;
  }
  
  if(strncmp("00",line,2)){
    std::cerr << "Hokuyo::ConfirmScan: ERROR: status response does not match (line 2):"<<std::endl;
    _printStrings("00",line,line_len);
      
    if(strncmp("10",line,2)==0){
      std::cerr << "Hokuyo::ConfirmScan: ERROR: (response from scanner)it looks like we need to turn the laser back on" << std::endl;
      LaserOn();
    }
    else if(strncmp("01",line,2)==0){
      std::cerr << "Hokuyo::ConfirmScan: ERROR: (response from scanner) Starting step has non-numeric value" << std::endl;
    }
    else if(strncmp("02",line,2)==0){
      std::cerr << "Hokuyo::ConfirmScan: ERROR: (response from scanner) End step has non-numeric value" << std::endl;
    }
    else if(strncmp("03",line,2)==0){
      std::cerr << "Hokuyo::ConfirmScan: ERROR: (response from scanner) Cluster count (skip) has non-numeric value" << std::endl;
    }
    else if(strncmp("04",line,2)==0){
      std::cerr << "Hokuyo::ConfirmScan: ERROR: (response from scanner)End step is out of range" << std::endl;
    }
    else if(strncmp("05",line,2)==0){
      std::cerr << "Hokuyo::ConfirmScan: ERROR: (response from scanner)End step is smaller than starting step" << std::endl;
    }
    else {
      std::cerr << "Hokuyo::ConfirmScan: ERROR: (response from scanner)Looks like hardware trouble" << std::endl;
    }
    return -1;
  }
  
  return 0;
}

int Hokuyo::GetScan(unsigned int * range, int & n_range,int scanStart, int scanEnd, int scanSkip, int encoding, int scanType, int numScans, bool * scanReady){
	static int _scanStart=-1;
  static int _scanEnd=-1;
  static int _scanSkip=-1;
  static int _encoding=-1;
  static int _scanType=-1;
  static int packet_length=-1;
  
  char req[128];
  char goodStatus[2];
	char line[HOKUYO_MAX_LINE_LENGTH];
  char full_packet[HOKUYO_MAX_PACKET_LENGTH];
  char extracted_packet[HOKUYO_MAX_PACKET_LENGTH];
  int line_len;
  bool needToRequestScan;

  if (!sd.IsConnected()){
		std::cerr << "Hokuyo::GetScan: ERROR: not connected to the sensor!" << std::endl;
		return -1;
	}
  
  if ( (_scanStart!=scanStart) || (_scanEnd!=scanEnd) || (_scanSkip!=scanSkip) || (_encoding!=encoding) || (_scanType!=scanType)  ){
    packet_length=-1;
    //std::cout<<"need to check packet size!!!!" <<std::endl;
  }
  else{
    //std::cout<<"reusing old packet size" <<std::endl;
  }
  
  memcpy(goodStatus,"99",2*sizeof(char));
  bool needToReadOffCmdAndStatus;
  if (CreateScanRequest(scanStart,scanEnd,scanSkip,encoding,scanType,numScans,req,needToReadOffCmdAndStatus)){
    std::cerr << "Hokuyo::GetScan: ERROR: could not create scan request string!" << std::endl;
		return -1;
  }
  
  if (numScans==1){
    needToRequestScan=true;
  }
  else if (numScans==0){
    if (!_streaming) {
      needToRequestScan=true;
    }
    else {
      needToRequestScan=false;
    }
  }
  else {
    std::cerr << "Hokuyo::GetScan: ERROR: invalid number of scans:"<< numScans << std::endl;
    return -1;
  }
  
  
  int timeout_us=0;
  switch (_type){
    case HOKUYO_TYPE_URG_04LX:
      timeout_us=150000;
      break;
      
    case HOKUYO_TYPE_UTM_30LX:
      timeout_us=50000;
      break;
  }
  
  
  //only send the request command if we need to
  //this means only when we request a single scan or to start streaming
  if (needToRequestScan){
    //std::cout<<"Requesting: "<<req<<std::endl;
    if (RequestScan(req)){
      std::cerr << "Hokuyo::GetScan: ERROR: could not request scan!" << std::endl;
      return -1;
    }
  
    if (ConfirmScan(req,timeout_us)){
      std::cerr << "Hokuyo::GetScan: ERROR: could not confirm scan!" << std::endl;
      return -1;
    }
    //the read was successful, so it means that if we requested streaming, we got it
    if (numScans==0){
      _streaming=true;
    }
    
    if ((numScans==0) || (scanType==HOKUYO_SCAN_SPECIAL_ME)){
      //read off the LF character
      line_len=_readLine(line,HOKUYO_MAX_LINE_LENGTH,timeout_us,false);
      if(line_len < 0){
        std::cerr << "Hokuyo::ConfirmScan: ERROR: could not read LF (line 3) "<< std::endl;
        return -1;
      }

      if(line_len > 0){
        std::cerr << "Hokuyo::ConfirmScan: ERROR: response does not match LF (line 3):" << std::endl;
        _printStrings("\n",line,line_len);
        return -1;
      }
    }
  }
  
  if (needToReadOffCmdAndStatus){
  
    //read back the echo of the command
    line_len=_readLine(line,HOKUYO_MAX_LINE_LENGTH,timeout_us,false);
    if(line_len < 0){
      std::cerr << "Hokuyo::GetScan: could not read echo of the command (line @1)" << std::endl;
      return -1;
    }
    
    if(strncmp(req,line,line_len-2)){     //FIXME: not checking the number of remaining scans
      std::cerr << "Hokuyo::GetScan: echo of the command does not match (line @1):"<< std::endl;
      _printStrings(req,line,line_len);
      return -1;
    }

    //read back the status response
    line_len=_readLine(line,HOKUYO_MAX_LINE_LENGTH,timeout_us,true);
    if(line_len < 0){
      std::cerr << "Hokuyo::GetScan: could not read line @2" << std::endl;
      return -1;
    }
    
    if(strncmp(goodStatus,line,2)){
      std::cerr << "Hokuyo::GetScan: response does not match (line @2):"<< std::endl;
      _printStrings(goodStatus,line,line_len);
      return -1;
    }
    //std::cerr << "Hokuyo::GetScan: Read off cmd and status"<< std::endl;
  }
    

  //read the timestamp
  line_len=_readLine(line,HOKUYO_MAX_LINE_LENGTH,timeout_us,true);
  if(line_len < 0){
    std::cerr << "Hokuyo::GetScan: could not read timestamp (line @3)" << std::endl;
    return -1;
  }
  
  //read the whole packet
  if (_readPacket(full_packet,packet_length,timeout_us)){
    std::cerr << "Hokuyo::GetScan: could not read data packet" << std::endl;
    return -1;
  }
  
  //verify the checksum, and remove LF and checksum chars from data
  int extracted_length=_extractPacket(full_packet, extracted_packet, packet_length);
  if (extracted_length < 0){
    std::cerr << "Hokuyo::GetScan: could not extract data from packet" << std::endl;
    return -1;
  }
  
  //decode the packet into the provided buffer
  pthread_mutex_lock(&m_HokuyoDataMutex);
  n_range=_decodePacket(extracted_packet, range, extracted_length, encoding);
  if ( (n_range < 0) || (n_range>HOKUYO_MAX_NUM_POINTS)){
    std::cerr << "Hokuyo::GetScan: returned too many data points" << std::endl;
    if (scanReady != NULL) *scanReady=false;
    pthread_mutex_unlock(&m_HokuyoDataMutex);        
    return -1;
  }
  if (scanReady != NULL) *scanReady=true;
  pthread_cond_signal(&m_HokuyoDataCond);
  pthread_mutex_unlock(&m_HokuyoDataMutex);
  
  //update the scan params
  _scanStart=scanStart;
  _scanEnd=scanEnd;
  _scanSkip=scanSkip;
  _encoding=encoding;
  _scanType=scanType;
  
  //the read was successful, so it means that if we requested streaming, we got it
  if (numScans==0){
    _streaming=true;
  }
  
  return 0;
}

//set the desired baud rate of the sensor by sending the appropriate command
int Hokuyo::_setBaudRate(const int baud_rate) {
	char request[16];
	char resp1[10];
	char resp2[10];
  char resp3[10];
	char line[HOKUYO_MAX_LINE_LENGTH];
	int numBytes;
	bool check;
	
	if (!sd.IsConnected()){
		std::cerr << "Hokuyo::_setBaudRate: ERROR: not connected to the sensor!" << std::endl;
		return -1;
	}
	
  switch (baud_rate) {
    case 19200:
      strcpy(request,"SS019200\n");
      break;
    case 115200:
      strcpy(request,"SS115200\n");
      break;
    default:
      std::cerr << "Hokuyo::_setBaudRate: invalid baudrate" << std::endl;
      return -1;
  }
  numBytes=9;
  check=true;
  strcpy(resp1,"00\n");
  strcpy(resp2,"03\n");
  strcpy(resp3,"04\n");
	
	// Send the command to Hokuyo
	
	sd.FlushInputBuffer();
	
	if (sd.WriteChars(request,numBytes) != numBytes){
		std::cerr << "Hokuyo::_setBaudRate: Unable to send command to hokuyo" << std::endl;
		return -1;
	}
	
  int timeout_us=100000;
  
	// Receive the confirmation that the command was accepted
  int line_len=_readLine(line,HOKUYO_MAX_LINE_LENGTH,timeout_us,false);
	if( line_len < 0){
		std::cerr << "Hokuyo::_setBaudRate: could not read a line" << std::endl;
		return -1;
	}
	if(strncmp(request,line,line_len)){
		std::cerr << "Hokuyo::_setBaudRate: response does not match:"<< std::endl;
    _printStrings(request,line,line_len);
		return -1;
    }
	
	// Receive the confirmation that the baud rate changed
  line_len=_readLine(line,HOKUYO_MAX_LINE_LENGTH,timeout_us,true);
	if(line_len < 0){
		std::cerr << "Hokuyo::_setBaudRate: could not read a line" << std::endl;
		return -1;
	}
	if( (strncmp(resp1,line,line_len)!=0) && (strncmp(resp2,line,line_len)!=0) && (strncmp(resp3,line,line_len)!=0) ){
		std::cerr << "Hokuyo::_setBaudRate: response does not match:" << std::endl;
    _printStrings(resp1,line,line_len);
    _printStrings(resp2,line,line_len);
    _printStrings(resp3,line,line_len);
		return -1;
  }
	
	// Set the host terminal baud rate to the test speed
	if(sd.SetBaudRate(baud_rate)) {
		std::cerr << "Hokuyo::_setBaudRate: could not set terminal baud rate" << std::endl;
		return -1;
	}
	
  _baud=baud_rate;
	return 0;
}

//try to get status information from the sensor, using a given baud rate
int Hokuyo::_testBaudRate(const int baud_rate) {
	
	if (!sd.IsConnected()){
		std::cerr << "Hokuyo::_testBaudRate: ERROR: not connected to the sensor!" << std::endl;
		return -1;
	}

  if (LaserOff()){
    std::cerr << "Hokuyo::_testBaudRate: ERROR: could not turn off the laser!" << std::endl;
		return -1;
  }
	
	for (int i=0;i<HOKUYO_NUM_TEST_BAUD_RETRIES;i++){
		// Attempt to get status information at the current baud 
		std::cout << "Testing " << baud_rate << "..." << std::endl;
		
		// Set the host terminal baud rate to the test speed 
		if(sd.SetBaudRate(baud_rate)==0) {
			// Check if the Hokuyo replies to setting SCIP2.0 
      if (!_setSCIP20()){
			  std::cout << "Hokuyo::_testBaudRate - protocol set to SCIP2.0 " << std::endl;
        _baud=baud_rate;  
			  return 0;
		  }			

      /* if(!_getStatusData()) {
        _baud=baud_rate;
				return 0;
			} */
			else {
				std::cerr << "Hokuyo::_testBaudRate: _setSCIP20() failed! (possibly a timeout)"  << std::endl;
			}
		}
		else {
			std::cerr << "Hokuyo::_testBaudRate: sd.SetBaudRate() failed!" << std::endl;
		}
		usleep(HOKUYO_TEST_BAUD_RATE_DELAY_US);
	}
	
	return -1;
}

int Hokuyo::_getStatusData(){
	
	if (!sd.IsConnected()){
		std::cerr << "Hokuyo::_getStatusData: ERROR: not connected to the sensor!" << std::endl;
		return -1;
	}
	
	if	(!_getStatusData(_mode)){
		return 0;
	} else {
		std::cerr << "Hokuyo::_getStatusData: Unable to get status!" << std::endl;
    return -1;
  }
}

void Hokuyo::_printLine(char * line, int line_len){
  for (int i=0;i<line_len;i++){
    std::cout<<line[i];
  }
  std::cout<<std::endl;
}

int Hokuyo::_getStatusData(int mode){
	char line[HOKUYO_MAX_LINE_LENGTH];
	char req[10];
	char resp[10];
	int numBytes;
	
	if (!sd.IsConnected()){
		std::cerr << "Hokuyo::_getStatusData: ERROR: not connected to the sensor!" << std::endl;
		return -1;
	}
	
	sd.FlushInputBuffer();
	
  strcpy(req,"VV\n");
  strcpy(resp,"00\n");
  numBytes=3;
  std::cout << "Hokuyo::_getStatusData: Trying SCIP2.0" << std::endl;
	
	if (sd.WriteChars(req,numBytes) != numBytes){
		std::cerr << "Hokuyo::_getStatusData: could not write request" << std::endl;
		return -1;
	}
	
  int timeout_us=100000;
  int line_len=_readLine(line,HOKUYO_MAX_LINE_LENGTH,timeout_us,false);
	if(line_len < 0){
		std::cerr << "Hokuyo::_getStatusData: could not read a line" << std::endl;
		return -1;
	}
	
	if(strncmp(req,line,line_len)){
		std::cerr << "Hokuyo::_getStatusData: response does not match:"<<std::endl;
    _printStrings(req,line,line_len);
		return -1;
  }
	
  line_len=_readLine(line,HOKUYO_MAX_LINE_LENGTH,timeout_us,true);
	if(line_len < 0){
		std::cerr << "Hokuyo::_getStatusData: could not read a line" << std::endl;
		return -1;
	}
	
	if(strncmp(resp,line,line_len)){
		std::cerr << "Hokuyo::_getStatusData: response does not match:" << std::endl;
    _printStrings(resp,line,line_len);
		return -1;
  }
  
  std::cout << std::endl<<"Sensor Information:" <<std::endl;
  std::cout <<"#########################################" <<std::endl;
  
  //Vendor
  line_len=_readLine(line,HOKUYO_MAX_LINE_LENGTH,timeout_us,false);
  if(line_len < 0){
		std::cerr << "Hokuyo::_getStatusData: could not read vendor line" << std::endl;
		return -1;
	}
  _printLine(line,line_len);
  
  //Product
  line_len=_readLine(line,HOKUYO_MAX_LINE_LENGTH,timeout_us,false);
  if(line_len < 0){
		std::cerr << "Hokuyo::_getStatusData: could not read product line" << std::endl;
		return -1;
	}
  
  _printLine(line,line_len);
  line[line_len-2]=0;  //last two characters are weird so just ignore them
  if (strncmp(line+5,HOKUYO_TYPE_UTM_30LX_STRING,line_len-7)==0){
    _type = HOKUYO_TYPE_UTM_30LX;
    std::cout<<"*** Sensor identified as HOKUYO TOP-URG UTM_30LX"<<std::endl;
  }
  else if (strncmp(line+5,HOKUYO_TYPE_URG_04LX_STRING,line_len-7)==0){
    _type = HOKUYO_TYPE_URG_04LX;
    std::cout<<"*** Sensor identified as HOKUYO URG 04LX"<<std::endl;
  }
  else {
    std::cout<<"<<<WARNING>>> Sensor could not be identified! Seeting sensor type to URG-04LX"<<std::endl;
    _type = HOKUYO_TYPE_URG_04LX;
  }
  
  //Firmware
  line_len=_readLine(line,HOKUYO_MAX_LINE_LENGTH,timeout_us,false);
  if(line_len < 0){
		std::cerr << "Hokuyo::_getStatusData: could not read firmware line" << std::endl;
		return -1;
	}
  _printLine(line,line_len);
  
  //Protocol
  line_len=_readLine(line,HOKUYO_MAX_LINE_LENGTH,timeout_us,false);
  if(line_len < 0){
		std::cerr << "Hokuyo::_getStatusData: could not read a line" << std::endl;
		return -1;
	}
  _printLine(line,line_len);
  
  //Serial
  line_len=_readLine(line,HOKUYO_MAX_LINE_LENGTH,timeout_us,false);
  if(line_len < 0){
		std::cerr << "Hokuyo::_getStatusData: could not read a line" << std::endl;
		return -1;
	}
  _printLine(line,line_len);
  
  std::cout <<"#########################################" <<std::endl<<std::endl;
  
	// read off the rest of the info from the buffer
	usleep(100000);
	sd.FlushInputBuffer();
	
	return 0;
}




int Hokuyo::_setSCIP20(){
	
	if (!sd.IsConnected()){
		std::cerr << "Hokuyo::_setSCIP20: ERROR: not connected to the sensor!" << std::endl;
		return -1;
	}
	
  //removing this check so that the sensor is always switched to SCIP2.0
  /*
	if (_mode == HOKUYO_SCIP20){
		std::cout << "Hokuyo::_setSCIP20: SCIP2.0 is already set!" << std::endl;
		return 0;
	}
  */
	
	if (sd.WriteChars("SCIP2.0\n",8) != 8) {
		std::cerr << "Hokuyo::_setSCIP20: could not write request" << std::endl;
		return -1;
	}
	
  char line[HOKUYO_MAX_LINE_LENGTH];
  int timeout_us = 200000;
  int line_len=_readLine(line,HOKUYO_MAX_LINE_LENGTH,timeout_us,false);
  
	if(line_len < 0){
		std::cerr << "Hokuyo::_setSCIP20: could not read a line" << std::endl;
		return -1;
	}
	
	if(strncmp("SCIP2.0",line,line_len)){
		std::cerr << "Hokuyo::_setSCIP20: response does not match:" << std::endl;
    _printStrings("SCIP2.0",line,line_len);
		return -1;
  }
	
	usleep(100000);
	sd.FlushInputBuffer();
	_mode=HOKUYO_SCIP20;
	return 0;
}



#ifdef MATLAB_MEX_FILE
  void createEmptyOutputMatrices(int nlhs, mxArray * plhs[]){
  for (int i=0; i<nlhs;i++){
    plhs[i] = mxCreateDoubleMatrix(0, 0, mxREAL);
  }
}

int createOutput(int sensorType, char * scanType, unsigned int * data, uint16_t n_points, int nlhs, mxArray * plhs[]){
  int numOutArgs=getNumOutputArgs(sensorType,scanType);
  if (numOutArgs <= 0){
    createEmptyOutputMatrices(nlhs,plhs);
    return -1;
  }
  
  int maxNumPoints;
  switch(sensorType){
    case HOKUYO_TYPE_UTM_30LX:
      maxNumPoints=HOKUYO_MAX_NUM_POINTS_UTM_30LX;
      break;
    case HOKUYO_TYPE_URG_04LX:
      maxNumPoints=HOKUYO_MAX_NUM_POINTS_URG_04LX;
      break;
    default:
      std::cerr << "createOutput: ERROR: invalid sensor type = " <<n_points << std::endl;
      createEmptyOutputMatrices(nlhs,plhs);
      return -1;
  }
  if (n_points > maxNumPoints){
    std::cerr << "createOutput: ERROR:too many points received = " <<n_points << std::endl;
    createEmptyOutputMatrices(nlhs,plhs);
    return -1;
  }
  
  if (n_points % numOutArgs != 0){
    std::cerr << "createOutput: ERROR: (number of data points) mod (number of output vars) is not zero!! n_points="<<n_points<<" n_vars="<< numOutArgs <<std::endl;
    createEmptyOutputMatrices(nlhs,plhs);
    return -1;
  }
 
  mxArray *out0, *out1, *out2;
  switch (numOutArgs){
    case 1:
      out0=mxCreateDoubleMatrix(n_points,1,mxREAL);
      for (int i=0;i<n_points;i++){
        mxGetPr(out0)[i]=(double)(data[i]);
      }
      plhs[0]=out0;
      break;
      
    case 2:
      out0=mxCreateDoubleMatrix(n_points/2,1,mxREAL);
      out1=mxCreateDoubleMatrix(n_points/2,1,mxREAL);
      
      for (int i=0;i<n_points;i+=2){
        mxGetPr(out0)[i/2]=(double)(data[i]);
        mxGetPr(out1)[i/2]=(double)(data[i+1]);
      }
      
      plhs[0]=out0;
      plhs[1]=out1;
      break;
      
    case 3:
      out0=mxCreateDoubleMatrix(n_points/3,1,mxREAL);
      out1=mxCreateDoubleMatrix(n_points/3,1,mxREAL);
      out2=mxCreateDoubleMatrix(n_points/3,1,mxREAL);
      
      for (int i=0;i<n_points;i+=3){
        mxGetPr(out0)[i/3]=(double)(data[i]);
        mxGetPr(out1)[i/3]=(double)(data[i+1]);
        mxGetPr(out2)[i/3]=(double)(data[i+2]);
      }
      
      plhs[0]=out0;
      plhs[1]=out1;
      plhs[2]=out2;
      break;
      
    default:
      createEmptyOutputMatrices(nlhs,plhs);
      return -1;
  }
  
  if (nlhs > numOutArgs){
    for (int i=numOutArgs;i<nlhs;i++){
      plhs[i]=mxCreateDoubleMatrix(0, 0, mxREAL);
    }
  }
  
  return 0;
}


int parseScanParams(int sensorType,char * scanTypeName,const mxArray * prhs[],int indexStart, int * start, int * stop, int * skip, int * encoding, int * scanType){
  *start = (int)mxGetPr(prhs[indexStart])[0];
	*stop = (int)mxGetPr(prhs[indexStart+1])[0];
  *encoding = (int)mxGetPr(prhs[indexStart+3])[0];

  if ((*encoding != 2) && (*encoding != 3)){
    std::cerr<<"Please make sure that encoding is valid"<<std::endl;
    return -1;
  }
  
  if ((sensorType==HOKUYO_TYPE_UTM_30LX) && (*encoding==2)){
    std::cerr<<"parseScanParams: WARNING: 30LX does not support 2-char encoding, setting it to 3 chars"<<std::endl;
    *encoding=3;
  }
      
  *encoding = *encoding==2 ? HOKUYO_2DIGITS : HOKUYO_3DIGITS;

  switch(sensorType){
  
    case HOKUYO_TYPE_URG_04LX:

      *scanType=HOKUYO_SCAN_REGULAR;      

      if ( (*start >= HOKUYO_MAX_NUM_POINTS_URG_04LX) || (*stop >= HOKUYO_MAX_NUM_POINTS_URG_04LX) || (*start>*stop) || (*start < 0) ){
		    std::cerr<<"Please make sure that start and stop params are valid"<<std::endl;
        return -1;
	    }

      if (strcmp(scanTypeName, HOKUYO_RANGE_STRING) == 0){
        *skip = (int)mxGetPr(prhs[indexStart+2])[0];
        if (*skip <= 0){
          std::cerr <<"Bad skip value: "<<*skip<<std::endl;
          return -1;
        }        
        break;
      }
      if (strcmp(scanTypeName, HOKUYO_RANGE_INTENSITY_AV_STRING) == 0){
        *skip=HOKUYO_RANGE_INTENSITY_AV;        
        break;
      }
      if (strcmp(scanTypeName, HOKUYO_RANGE_INTENSITY_0_STRING) == 0){
        *skip=HOKUYO_RANGE_INTENSITY_0;        
        break;
      }
      if (strcmp(scanTypeName, HOKUYO_RANGE_INTENSITY_1_STRING) == 0){
        *skip=HOKUYO_RANGE_INTENSITY_1;        
        break;
      }
      if (strcmp(scanTypeName, HOKUYO_INTENSITY_AV_STRING) == 0){
        *skip=HOKUYO_INTENSITY_AV;        
        break;
      }
      if (strcmp(scanTypeName, HOKUYO_INTENSITY_0_STRING) == 0){
        *skip=HOKUYO_INTENSITY_0;        
        break;
      }
      if (strcmp(scanTypeName, HOKUYO_INTENSITY_1_STRING) == 0){
        *skip=HOKUYO_INTENSITY_1;        
        break;
      }
      if (strcmp(scanTypeName, HOKUYO_RANGE_INTENSITY_AV_AGC_AV_STRING) == 0){
        *skip=HOKUYO_RANGE_INTENSITY_AV_AGC_AV;        
        break;      
      }
      if (strcmp(scanTypeName, HOKUYO_RANGE_INTENSITY_0_AGC_0_STRING) == 0){
        *skip=HOKUYO_RANGE_INTENSITY_0_AGC_0;        
        break;
      }
      if (strcmp(scanTypeName, HOKUYO_RANGE_INTENSITY_1_AGC_1_STRING) == 0){
        *skip=HOKUYO_RANGE_INTENSITY_1_AGC_1;        
        break;
      }
      if (strcmp(scanTypeName, HOKUYO_AGC_0_STRING) == 0){
        *skip=HOKUYO_AGC_0;        
        break;
      }
      if (strcmp(scanTypeName, HOKUYO_AGC_1_STRING) == 0){
        *skip=HOKUYO_AGC_0;        
        break;
      }
      
      std::cerr << "Error: URG_04-LX does not support this scan mode: " <<scanTypeName<<std::endl;
      return -1;
      
    case HOKUYO_TYPE_UTM_30LX:
      
      if ( (*start >= HOKUYO_MAX_NUM_POINTS_UTM_30LX) || (*stop >= HOKUYO_MAX_NUM_POINTS_UTM_30LX) || (*start>*stop) || (*start < 0) ){
		    std::cerr<<"Please make sure that start and stop params are valid"<<std::endl;
        return -1;
	    }

      if (strcmp(scanTypeName, HOKUYO_RANGE_STRING) == 0){
        *scanType=HOKUYO_SCAN_REGULAR;  
        *skip = (int)mxGetPr(prhs[indexStart+2])[0];
        if (*skip <= 0){
          std::cerr <<"Bad skip value: "<<*skip<<std::endl;
          return -1;
        }        
        break;
      }

      if (strcmp(scanTypeName, HOKUYO_TOP_URG_RANGE_INTENSITY_STRING) == 0){
        *scanType=HOKUYO_SCAN_SPECIAL_ME;  
        *skip = (int)mxGetPr(prhs[indexStart+2])[0];
        if (*skip <= 0){
          std::cerr <<"Bad skip value: "<<*skip<<std::endl;
          return -1;
        }    
        break;
      }
      
      std::cerr << "Error: UTM_30LX does not support this scan mode: " <<scanTypeName<<std::endl;
      return -1;
      
    default:
      std::cerr << "Error: unknown sensor type: "<<std::endl;
      return -1;
  }
  //std::cout <<"New scan parameters are: start=" <<*start <<" stop=" <<*stop <<" skip="<< *skip<<" encoding="<<*encoding<<std::endl;
  return 0;
}

#endif //MATLAB_MEX_FILE

int getScanTypeAndSkipFromName(int sensorType, char * scanTypeName, int * skip, int * scanType){
  
  switch(sensorType){
  
    case HOKUYO_TYPE_URG_04LX:

      *scanType=HOKUYO_SCAN_REGULAR;

      if (strcmp(scanTypeName, HOKUYO_RANGE_STRING) == 0){
        *skip = 1;
        break;
      }
      if (strcmp(scanTypeName, HOKUYO_RANGE_INTENSITY_AV_STRING) == 0){
        *skip=HOKUYO_RANGE_INTENSITY_AV;        
        break;
      }
      if (strcmp(scanTypeName, HOKUYO_RANGE_INTENSITY_0_STRING) == 0){
        *skip=HOKUYO_RANGE_INTENSITY_0;        
        break;
      }
      if (strcmp(scanTypeName, HOKUYO_RANGE_INTENSITY_1_STRING) == 0){
        *skip=HOKUYO_RANGE_INTENSITY_1;        
        break;
      }
      if (strcmp(scanTypeName, HOKUYO_INTENSITY_AV_STRING) == 0){
        *skip=HOKUYO_INTENSITY_AV;        
        break;
      }
      if (strcmp(scanTypeName, HOKUYO_INTENSITY_0_STRING) == 0){
        *skip=HOKUYO_INTENSITY_0;        
        break;
      }
      if (strcmp(scanTypeName, HOKUYO_INTENSITY_1_STRING) == 0){
        *skip=HOKUYO_INTENSITY_1;        
        break;
      }
      if (strcmp(scanTypeName, HOKUYO_RANGE_INTENSITY_AV_AGC_AV_STRING) == 0){
        *skip=HOKUYO_RANGE_INTENSITY_AV_AGC_AV;        
        break;      
      }
      if (strcmp(scanTypeName, HOKUYO_RANGE_INTENSITY_0_AGC_0_STRING) == 0){
        *skip=HOKUYO_RANGE_INTENSITY_0_AGC_0;        
        break;
      }
      if (strcmp(scanTypeName, HOKUYO_RANGE_INTENSITY_1_AGC_1_STRING) == 0){
        *skip=HOKUYO_RANGE_INTENSITY_1_AGC_1;        
        break;
      }
      if (strcmp(scanTypeName, HOKUYO_AGC_0_STRING) == 0){
        *skip=HOKUYO_AGC_0;        
        break;
      }
      if (strcmp(scanTypeName, HOKUYO_AGC_1_STRING) == 0){
        *skip=HOKUYO_AGC_0;        
        break;
      }
      
      std::cerr << "getScanTypeAndSkipFromName: Error: URG_04-LX does not support this scan mode: " <<scanTypeName<<std::endl;
      return -1;
      
    case HOKUYO_TYPE_UTM_30LX:

      if (strcmp(scanTypeName, HOKUYO_RANGE_STRING) == 0){
        *scanType=HOKUYO_SCAN_REGULAR;  
        *skip = 1;      
        break;
      }

      if (strcmp(scanTypeName, HOKUYO_TOP_URG_RANGE_INTENSITY_STRING) == 0){
        *scanType=HOKUYO_SCAN_SPECIAL_ME;  
        *skip = 1;    
        break;
      }
      
      std::cerr << "getScanTypeAndSkipFromName: Error: UTM_30LX does not support this scan mode: " <<scanTypeName<<std::endl;
      return -1;
      
    default:
      std::cerr << "getScanTypeAndSkipFromName: Error: unknown sensor type: "<<std::endl;
      return -1;
  }
  return 0;
}

int getNumOutputArgs(int sensorType, char * scanTypeName){
  switch(sensorType){
  
    case HOKUYO_TYPE_URG_04LX:
      if (strcmp(scanTypeName, HOKUYO_RANGE_STRING) == 0)
        return 1;
      if (strcmp(scanTypeName, HOKUYO_RANGE_INTENSITY_AV_STRING) == 0)
        return 2;
      if (strcmp(scanTypeName, HOKUYO_RANGE_INTENSITY_0_STRING) == 0)
        return 2;
      if (strcmp(scanTypeName, HOKUYO_RANGE_INTENSITY_1_STRING) == 0)
        return 2;
      if (strcmp(scanTypeName, HOKUYO_INTENSITY_AV_STRING) == 0)
        return 1;
      if (strcmp(scanTypeName, HOKUYO_INTENSITY_0_STRING) == 0)
        return 1;
      if (strcmp(scanTypeName, HOKUYO_INTENSITY_1_STRING) == 0)
        return 1;
      if (strcmp(scanTypeName, HOKUYO_RANGE_INTENSITY_AV_AGC_AV_STRING) == 0)
        return 3;
      if (strcmp(scanTypeName, HOKUYO_RANGE_INTENSITY_0_AGC_0_STRING) == 0)
        return 3;
      if (strcmp(scanTypeName, HOKUYO_RANGE_INTENSITY_1_AGC_1_STRING) == 0)
        return 3;
      if (strcmp(scanTypeName, HOKUYO_RANGE_INTENSITY_1_AGC_1_STRING) == 0)
        return 3;
      if (strcmp(scanTypeName, HOKUYO_AGC_0_STRING) == 0)
        return 1;
      if (strcmp(scanTypeName, HOKUYO_AGC_1_STRING) == 0)
        return 1;
      
      std::cerr << "Error: URG_04-LX does not support this scan type: " <<scanTypeName<<std::endl;
      return -1;
      
    case HOKUYO_TYPE_UTM_30LX:
      if (strcmp(scanTypeName, HOKUYO_RANGE_STRING) == 0)
        return 1;
      if (strcmp(scanTypeName, HOKUYO_TOP_URG_RANGE_INTENSITY_STRING) == 0)
        return 2;
      
      std::cerr << "Error: UTM_30LX does not support this scan type: " <<scanTypeName<<std::endl;
      return -1;
      
    default:
      std::cerr << "Error: unknown sensor type: "<<std::endl;
      return -1;
  }
}
	


#endif //HOKUYO_HPP
