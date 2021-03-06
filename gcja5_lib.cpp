/****************************************************************
 * 
 * The SN-GCJA5 measures particles and provides mass density (PM1.0, PM2.5 and PM10) 
 * as well as particle count (PM0.5m, PM1, PM2.5, PM5, PM7.5 and Pm10)
 * 
 * Parts of the definition is based on Sparkfun's original library for Arduino-variants
 * SparkFun sells these at its website: www.sparkfun.com
 * Help support SparkFun. Buy a board! https://www.sparkfun.com/products/17123
 * 
 * version 2.0 / March 2021 / paulvha
 * - added SetI2cDevice(), SetI2cExit()
 * - added option to use kernel device (/dev/i2c-x)
 * - change SetI2cClock() call 
 * - change begin() and all I2C-calls to support kernel device
 * - change INCLUDE_SERIAL, INCLUDE_BCM2835_I2C, INCLUDE_KERNEL_I2C
 * - update to documentation
 * - update to makefile
 * 
 * version 1.0 / november 2020 /  
 * by Paul van Haastrecht (paulvha@hotmail.com)
 * 
 * - Initial Raspberry Pi
 * 
 * Resources / dependencies:
 * BCM2835 library (http://www.airspayce.com/mikem/bcm2835/) in case of I2C
 * 
 * For more details see sngcja5_on_raspberry.odt
 * 
 * BCM2835 library (http://www.airspayce.com/mikem/bcm2835/)
 * 
 * *****************************************************************
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 **********************************************************************/

#include "gcja5_lib.h"
#include <sys/ioctl.h> 

/*
 * 0 : no debug message
 * 1 : sending and receiving data
 * 2 : 1 + I2c protocol progress
 */
int _GCJA5_Debug = 0;

/* used as part of p_printf() */
bool NoColor=false;

/* buffer to store data read */
uint8_t read_buf[MAXBUF];

// check during compile
#if !defined(INCLUDE_SERIAL) && !defined(INCLUDE_BCM2835_I2C) && !defined(INCLUDE_KERNEL_I2C)
#error  Select I2C and/or Serial communication
#endif

/******************************************* 
 * @brief Constructor 
 *******************************************/
GCJA5::GCJA5(void){}

/************************************************************** 
 * @brief Initialize the port and check for GCJA5 
 * @param device. if supplied serial communication is expected
 * @return  true = OK, false is error 
 **************************************************************/
bool GCJA5::begin(char *device) {
    bool ret = false;
  
    if (device != NULL) {       // so we expect to communicate serial
#if defined(INCLUDE_SERIAL) 

        if (Ser_init(device) != GCJA5_ERR_OK) goto end;
#else
        if (_GCJA5_Debug) printf("No Serial support enabled\n");
        goto end;
#endif
    }
    else {                  // so must be I2C now
    
        if (I2C_init() != GCJA5_ERR_OK) goto end;
    }
    
    /* check for the GCJA5 */
    if (readMeasurement() != GCJA5_ERR_OK) goto end;

    ret = true;
end: 
    if (!ret) I2C_device_end();
    return(ret);
}

/**
 * @brief : Read all the registers to buffer
 */
uint8_t GCJA5::readMeasurement()
{
    // reset buffer
    memset (read_buf,0x0, sizeof(read_buf));
    
#if defined(INCLUDE_SERIAL)    
    // if connected over serial/uart
    if (_UseSerial) {
        if ( Ser_read(5) != GCJA5_ERR_OK) {
            if (_GCJA5_Debug == 2) printf("Error during reading serial\n");
            return(GCJA5_ERR_OUTOFRANGE);
        }
    }
    else 
#endif
    {
        // read all the PM and PC in one go
        if ( I2C_Read(SNGCJA5_PM1_0,40) != GCJA5_ERR_OK) {
            if (_GCJA5_Debug == 2) printf("Error during reading I2C\n");
            return(GCJA5_ERR_OUTOFRANGE);
        }
    }
    
    Pm1HasBeenReported = false;        
    Pm25HasBeenReported = false;    
    Pm10HasBeenReported = false;
        
    Pc05HasBeenReported = false;        
    Pc1HasBeenReported = false;
    Pc25HasBeenReported = false;
    Pc5HasBeenReported = false;
    Pc75HasBeenReported = false;
    Pc10HasBeenReported = false;
    
    StateHasBeenReported = false;

    StSenHasBeenReported = false;
    StPdHasBeenReported = false;
    StLdHasBeenReported = false;
    StFanHasBeenReported = false;
    
    return(GCJA5_ERR_OK);
}

/**
 * @brief : Read the mass density PM register + conversion 
 */
float GCJA5::getPM(uint8_t pmRegister)
{
  uint32_t count = readRegister32(pmRegister);
#if defined(INCLUDE_SERIAL)  
  // only on I2C behind the decimal point
  if ( _UseSerial)  return (count);
#endif
  return (count / 1000.0);
}

float GCJA5::getPM1_0()
{
  /* trigger new read if needed */  
  if (Pm1HasBeenReported)  
        readMeasurement(); //Pull in new data from sensor

  Pm1HasBeenReported = true;
  
  return (getPM(SNGCJA5_PM1_0));
}

float GCJA5::getPM2_5()
{
  /* trigger new read if needed */  
  if (Pm25HasBeenReported == true) 
        readMeasurement(); //Pull in new data from sensor
  
  Pc25HasBeenReported = true;
  return (getPM(SNGCJA5_PM2_5));
}

float GCJA5::getPM10()
{
  /* trigger new read if needed */  
  if (Pm10HasBeenReported == true) 
        readMeasurement(); //Pull in new data from sensor
  
  Pm10HasBeenReported = true;
  return (getPM(SNGCJA5_PM10));
}

/**
 * @brief Particle count functions (TOTAL count for each PM)
 * @param check : true will perform a check on reported earlier
 */
uint16_t GCJA5::getPC0_5(bool check) {
  
  if (check) {
      /* trigger new read if needed */  
      if (Pc05HasBeenReported == true) 
        readMeasurement(); //Pull in new data from sensor
  }
  Pc05HasBeenReported = true;

  return(readRegister16(SNGCJA5_PCOUNT_0_5));
}

uint16_t GCJA5::getPC1_0(bool check) {
  uint16_t val;
  
  if (check) {
    /* trigger new read if needed */  
    if (Pc1HasBeenReported == true) 
        readMeasurement(); //Pull in new data from sensor
  }
 
  Pc1HasBeenReported = true;

  val = getPC0_5(false);
  val += readRegister16(SNGCJA5_PCOUNT_1_0);
  return(val);
}

uint16_t GCJA5::getPC2_5(bool check) {
  uint16_t val;
  if (check) {
    /* trigger new read if needed */  
    if (Pc25HasBeenReported == true) 
        readMeasurement(); //Pull in new data from sensor
  }
  Pc25HasBeenReported = true;

  val = getPC1_0(false);
  val += readRegister16(SNGCJA5_PCOUNT_2_5);
  return(val);
}

uint16_t GCJA5::getPC5_0(bool check) {
  uint16_t val;
  if (check) {
    /* trigger new read if needed */  
    if (Pc5HasBeenReported == true) 
        readMeasurement(); //Pull in new data from sensor
  }
  Pc5HasBeenReported = true;

  val = getPC2_5(false);
  val += readRegister16(SNGCJA5_PCOUNT_5_0);
  return(val);
}

uint16_t GCJA5::getPC7_5(bool check)
{
  uint16_t val;
  if (check) {
    /* trigger new read if needed */  
    if (Pc75HasBeenReported == true) 
        readMeasurement(); //Pull in new data from sensor
  }
  Pc75HasBeenReported = true;

  val = getPC5_0(false);
  val += readRegister16(SNGCJA5_PCOUNT_7_5);
  return(val);
}

uint16_t GCJA5::getPC10(bool check)
{
  uint16_t val;
  if (check) {
    /* trigger new read if needed */  
    if (Pc10HasBeenReported == true) 
        readMeasurement(); //Pull in new data from sensor
  }
  Pc10HasBeenReported = true;

  val = getPC7_5(false);
  val += readRegister16(SNGCJA5_PCOUNT_10);
  return(val);
}

/**
 * @brief Get status for Sensors, Photo-Diode, Laser Diode or Fan
 */
uint8_t GCJA5::getState()
{
  /* trigger new read if needed */  
  if (StateHasBeenReported == true) 
        readMeasurement(); //Pull in new data from sensor
  
  StateHasBeenReported = true;
  
  return(readRegister8(SNGCJA5_STATE));
}

uint8_t GCJA5::getStatusSensors()
{
  /* trigger new read if needed */  
  if (StSenHasBeenReported == true) 
        readMeasurement(); //Pull in new data from sensor
  
  StSenHasBeenReported = true;
  
  return((readRegister8(SNGCJA5_STATE) >> 6) & 0b11);
}

uint8_t GCJA5::getStatusPD()
{
  /* trigger new read if needed */  
  if (StPdHasBeenReported == true) 
        readMeasurement(); //Pull in new data from sensor
  
  StPdHasBeenReported = true;
  
  return ((readRegister8(SNGCJA5_STATE) >> 4) & 0b11);
}

uint8_t GCJA5::getStatusLD()
{
  /* trigger new read if needed */  
  if (StLdHasBeenReported == true) 
        readMeasurement(); //Pull in new data from sensor
  
  StLdHasBeenReported = true;
  
  return((readRegister8(SNGCJA5_STATE) >> 2) & 0b11);
}

uint8_t GCJA5::getStatusFan()
{
  /* trigger new read if needed */  
  if (StFanHasBeenReported == true) 
        readMeasurement(); //Pull in new data from sensor
  
  StFanHasBeenReported = true;
  
  return((readRegister8(SNGCJA5_STATE) >> 0) & 0b11);
}

/**
 * @brief close hardware correctly on the Raspberry Pi
 * 
 */
void GCJA5::ConnClose(void) {

#if defined(INCLUDE_SERIAL)
    if (_UseSerial) 
        Ser_close();
    else 
#endif
        I2C_close();
}

/************************************************************
 * @brief Set for debugging the driver
 *
 * @param val : action to be performed
 * 0 = disable debug messages
 * 1 = sent/receive messages
 * 2 = like 1 + protocol errors
 *
 * This can be called BEFORE performing the begin() call.
 ************************************************************/
void GCJA5::setDebug(int val) {
    _GCJA5_Debug = val;
}

////////////////////////////////////////////////////////////////////
///// low level routines ///////////////////////////////////////////
////////////////////////////////////////////////////////////////////
#if defined(INCLUDE_BCM2835_I2C) || defined(INCLUDE_KERNEL_I2C)

/**
 * @brief : Start I2C communication
 * 
 * @return
 * All good : GCJA5_ERR_OK
 * else error
 */
uint8_t GCJA5::I2C_init()
{
     
#if defined(INCLUDE_KERNEL_I2C) 
    if (_i2cfile > 0) {

        // set slave address
        if (ioctl(_i2cfile, I2C_SLAVE, GCJA5_ADDRESS) < 0){
            printf("Can't set Slave address!\n");
            return(GCJA5_ERR_PROTOCOL);
        }
 
        return(GCJA5_ERR_OK);
    }
#endif

#if defined(INCLUDE_BCM2835_I2C)    
    if (!bcm2835_init()) {
        printf("Can't init bcm2835!\n");
        return(GCJA5_ERR_PROTOCOL);
    }

    // will select I2C channel 0 or 1 depending on board version.
    if (!bcm2835_i2c_begin()) {
        printf("Can't setup I2c pin!\n");
        
        // release BCM2835 library
        bcm2835_close();
        return(GCJA5_ERR_PROTOCOL);
    }
    
    /* set speed to default speed*/
    I2C_SetClock(GCJA5_SPEED);

    /* set BCM2835 slaveaddress */
    bcm2835_i2c_setSlaveAddress(GCJA5_ADDRESS);
    
    return(GCJA5_ERR_OK);
#endif

    if (_GCJA5_Debug) printf("INCLUDE_BCM2835_I2C not defined!\n");
    return(GCJA5_ERR_UNKNOWNCMD);
}

/**
 * @brief : close library and reset pins.
 */
void GCJA5::I2C_close()
{
    
#if defined(INCLUDE_KERNEL_I2C) 
    if (_i2cfile > 0) {
        I2C_device_end();
        return;
    }
#endif

#if defined(INCLUDE_BCM2835_I2C)     
    // reset pins ? 
    // some are using BOTH BCM2835 and kernel drivers
    if (ExitSpeed > 0)        
        I2C_SetClock(ExitSpeed);
    else 
        bcm2835_i2c_end();  
    
    // release BCM2835 library
    bcm2835_close();
#endif

}

/**
 * @brief : will set exit condition
 * speed : the exit speed for SCL
 */
bool GCJA5::SetI2cExit(uint16_t speed)
{
#if defined(INCLUDE_BCM2835_I2C)  
    ExitSpeed = speed;
    return(true);
#else
   if (_GCJA5_Debug) printf("BCM2835 I2C is NOT enabled\n");
   I2C_device_end();
   return(false);
#endif
}

/**
 * @brief : set I2C speed.
 * Can not be changed on the fly for kernel drivers.
 * Clock divided is based on nominal base clock rate defined in gcja5.h (depending on the board)
 */
void GCJA5::I2C_SetClock(uint16_t clock)
{
#if defined(INCLUDE_KERNEL_I2C) 
    
    if (_i2cfile > 0) {
        
        if (_GCJA5_Debug){
             printf("I2C for kernel can only be changed in/boot/config.txt\n");
             printf("Add a line dtparam = i2c_baudrate = 50000 (for 50Khz)\n");
        }
        return;
    }
#endif

#if defined(INCLUDE_BCM2835_I2C)    
   uint16_t baudrate = PI_CORE_CLOCK / clock;
   bcm2835_i2c_setClockDivider(baudrate);
#endif
}

/**
 * @brief : read with I2C communication
 * @param cnt: number of data bytes to get
 * @param address : starting register to read
 */
uint8_t GCJA5::I2C_Read(uint8_t reg, uint8_t cnt)
{
    uint8_t i;
#if defined(INCLUDE_KERNEL_I2C) 

    // this code is taken from 
    // https://gist.github.com/JamesDunne/9b7fbedb74c22ccc833059623f47beb7
    uint8_t outbuf[1];
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset[1];
    
    if (_i2cfile > 0) {

        msgs[0].addr = GCJA5_ADDRESS;
        msgs[0].flags = 0;
        msgs[0].len = 1;
        msgs[0].buf = outbuf;
    
        msgs[1].addr = GCJA5_ADDRESS;
        msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
        msgs[1].len = cnt;
        msgs[1].buf = read_buf;
    
        msgset[0].msgs = msgs;
        msgset[0].nmsgs = 2;
    
        outbuf[0] = reg;

        if (ioctl(_i2cfile, I2C_RDWR, &msgset) < 0) {
            perror("ioctl(I2C_RDWR) in i2c_read");
            return(GCJA5_ERR_PROTOCOL);
        }
        
        goto next;
    }
#endif

#if defined(INCLUDE_BCM2835_I2C)  
    // read from Sensor (with restart !!)
    switch(bcm2835_i2c_read_register_rs((char *) &reg, (char *) read_buf, cnt))
    {
        case BCM2835_I2C_REASON_ERROR_NACK :
            if(_GCJA5_Debug == 2) printf(REDSTR,"DEBUG: Read NACK error\n");
            return(GCJA5_ERR_PROTOCOL);
            break;

        case BCM2835_I2C_REASON_ERROR_CLKT :
            if(_GCJA5_Debug == 2) printf(REDSTR,"DEBUG: Read Clock stretch error\n");
            return(GCJA5_ERR_PROTOCOL);
            break;

        case BCM2835_I2C_REASON_ERROR_DATA :
            if(_GCJA5_Debug ==2) printf(REDSTR,"DEBUG: not all data has been read\n");
            return(GCJA5_ERR_PROTOCOL);
            break;
    }
    
    goto next; // else the compiler will complain and fail if INCLUDE_KERNEL_I2C not defined
#endif

next:
    if (_GCJA5_Debug == 2) {
        printf("received:\n");
        for(i = 0; i < cnt; i++) printf("0x%02x ",read_buf[i]);
        printf("\n");
    }
    
    // The sensor needs delay between I2C calls, 
    // or it will just bail out..
    
    usleep(300000);      // 300 mS looks save 
    
    return(GCJA5_ERR_OK);
}

/**********************************
 *  close out I2C kernel
 **********************************/
void GCJA5::I2C_device_end()
{
#ifdef INCLUDE_KERNEL_I2C
    if(_i2cfile > 0 ) {
        close(_i2cfile);
    }
#endif
}

/************************************************************** 
 * @brief indicate to use the I2C kernel device 
 * @param adapter. Together will form /dev/i2c-x
 * @return  true = OK, false if error 
 **************************************************************/
bool GCJA5::SetI2cDevice(uint8_t adapter)
{
#if defined(INCLUDE_KERNEL_I2C)   

    snprintf(_i2cdevice,19,"/dev/i2c-%d",adapter);
    
    _i2cfile = open(_i2cdevice, O_RDWR);
    
    if (_i2cfile < 0) {
        if (_GCJA5_Debug) printf("Can not open %s\n", _i2cdevice);
        return(false);
    }
    
    return(true);
#else
    if (_GCJA5_Debug) printf("No I2C Kernel driver support enabled\n");
    return(false);
#endif
}

/**
 * @brief : Set the I2C clock speed
 */
void GCJA5::SetI2cClock(uint16_t clock) 
{
    I2C_SetClock(clock);
}

#else
// placeholders
uint8_t GCJA5::I2C_init(){printf("No I2C support\n"); return(GCJA5_ERR_PROTOCOL);}
uint8_t GCJA5::I2C_Read(uint8_t reg, uint8_t cnt){printf("No I2C support\n");return(GCJA5_ERR_PROTOCOL);}
void GCJA5::I2C_SetClock(uint16_t clock){printf("No I2C support\n");}
void GCJA5::I2C_close(){printf("No I2C support\n");}
void GCJA5::I2C_device_end() {printf("No I2C support\n");}
void GCJA5::SetI2cClock(uint16_t clock){printf("No I2C support\n"); return;} 
bool GCJA5::SetI2cDevice(uint8_t adapter){printf("No I2C support\n"); return(false);} 
bool GCJA5::SetI2cExit(uint16_t speed){printf("No I2C support\n"); return(false);} 

#endif //defined(INCLUDE_BCM2835_I2C) || defined(INCLUDE_KERNEL_I2C)

/**
 * @brief : read registers and return value
 * @param addr : starting register to read
 **/
uint8_t GCJA5::readRegister8(uint8_t addr){

    return(read_buf[addr]);
}

uint16_t GCJA5::readRegister16(uint8_t addr){
   
    return ((uint16_t)read_buf[addr+1] << 8 | read_buf[addr]);
}

uint32_t GCJA5::readRegister32(uint8_t addr){

    return (((uint32_t)read_buf[addr+3] << 24) | ((uint32_t)read_buf[addr+2] << 16) | \
    ((uint32_t)read_buf[addr+1] << 8) | ((uint32_t)read_buf[addr] << 0));
}

//////////////////////////////////////////////////////////////////////
//     Serial communication
/////////////////////////////////////////////////////////////////////

/**
 * open connection to sensor
 */
#if defined(INCLUDE_SERIAL)
 
uint8_t GCJA5::Ser_init(char * device)
{
    // if already initialised
    if (_connected)	return(GCJA5_ERR_OK);
           
    _SerialDevice[0] = 0x0;
    strncpy(_SerialDevice, device, sizeof(_SerialDevice));
   
    // open device
    if (_SerialDevice == NULL) return(GCJA5_ERR_PARAMETER);
    
    _fd = open(_SerialDevice, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
    
    if (_fd < 0)
    {
		printf("Unable to open device %s.\n",_SerialDevice);
		return(GCJA5_ERR_PARAMETER);
    }
    
    // flush any pending input or output data
    tcflush(_fd,TCIOFLUSH);
    
    // configure
    if (Ser_configure() != GCJA5_ERR_OK)	return(GCJA5_ERR_CMDSTATE);
    
    // set as opened / initialised
    _connected = true;
    _UseSerial = true;

    return(GCJA5_ERR_OK);
}

/**
 *  close the connection correctly 
 */
void GCJA5::Ser_close()
{
    if (!_connected) return;
    
    if (_fd)
    {
		// restore old port settings
		if (tcsetattr(_fd, TCSANOW, &_old_options) < 0)
			printf("Unable to restore serial setting on device.\n");
	
        close(_fd);
	
		_fd=0;
    }
    
    _connected = false;
}

/**
 * @brief set the serial configuration correct to read
 * 
 * Baudrate9600 1 start, 8 data, 1 even parity, 1 stop
 * returns Ok(GCJA5_ERR_OK) else error
 */
 
uint8_t GCJA5::Ser_configure()
{
    struct termios options;
    
    tcgetattr(_fd, &options);
    tcgetattr(_fd, &_old_options);	// restore later
    
    cfsetispeed(&options, B9600);	// set input speed
    cfsetospeed(&options, B9600);	// set output speed

    options.c_iflag |=  IGNBRK;     // ignore break condition
    options.c_iflag |=  INPCK;      // enable parity checking
    options.c_iflag &=  ~PARMRK;    // disable marking parity or framing error checking   
    options.c_iflag &=  ~IUTF8;     // disable character erase
    options.c_iflag &=  ~IUCLC;     // disable upper case to lower case
    options.c_iflag &= ~(IXON|IXOFF);	// no flow control
    options.c_iflag &= ~(ISTRIP | IGNCR | INLCR | ICRNL);
            
    options.c_cflag &=  ~CSIZE;
    options.c_cflag |= CS8;			// 8 bit
    options.c_cflag &= ~CSTOPB;		// 1 stopbit
    options.c_cflag &= ~CRTSCTS; 	// no flow control
    options.c_cflag |= PARENB;		// parity
    options.c_cflag &= ~PARODD;		// even 
  
    options.c_cc[VMIN] = 30;		// need at least 1 character
    options.c_cc[VTIME] = 0;		// no timeout
    
    options.c_cflag |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    options.c_oflag &= ~(OPOST);
    
    if (tcsetattr(_fd, TCSANOW, &options) < 0) 
    {
		if(_GCJA5_Debug) printf("Unable to configure Serial port.\n");
		return(GCJA5_ERR_CMDSTATE);
    }
	
    return(GCJA5_ERR_OK);
}

/**
 * @brief read from sensor on serial
 * 
 * @param wait : max time in seconds before returning
 * 
 * if wait = 0 the read will block until received data
 * 
 * returns number of characters read into buffer or 0 
 */
uint8_t GCJA5::Ser_read(uint8_t wait)
{
    int		num,x;
    time_t 	time_start;
    char    tempbuf[50];
    uint8_t FCC = 0, offset = 0;
	
    // check that init was done
    if (!_connected) return(GCJA5_ERR_UNKNOWNCMD);
    
    // flush any pending input or output data
    tcflush(_fd,TCIOFLUSH);
    
    // get start time
    time_start=time(NULL);
	
    // wait for data
    do {
        
        // try to read
        num = read(_fd, tempbuf, 32);

		// if nothing, sleep second
		if (num == -1)	sleep(1);
		
        else {
/*            
  // for debug only
  for (int x =0; x < num;x++){
    printf("0x%02x", tempbuf[x]);
    printf(" ");
  }
  printf("\n");           
*/  
            // check for start of message
            if (tempbuf[0] == STX) {
            
                for (x = 1 ; x < 32 ; x++){
                    
                    if (x == 30) {
              
                        if ( FCC != tempbuf[x]  || tempbuf[x+1] != ETX) {
                            return(GCJA5_ERR_OUTOFRANGE);
                        }
                        else {
                            read_buf[SNGCJA5_STATE] = tempbuf[29];
                            return(GCJA5_ERR_OK);
                        }
                    }
                    else {    
                        FCC = FCC ^ tempbuf[x];    // calculate check sum
                        
                        // skip empty positions in serial input
                        if (x != 21 &&  x != 22)
                            read_buf[offset++] = tempbuf[x];// store read_buf
                   }
                } // for all the daa
            } //start of message
        } //we have data
			
		// if wait time was requested, check whether elapsed		
		if (wait > 0)
		{
		    if (wait < time(NULL) - time_start)
			return(GCJA5_ERR_TIMEOUT);
		}
    } while(1);
}
#endif // INCLUDE_SERIAL

/*********************************************************************
 * @brief Display in color
 * @param format : Message to display and optional arguments
 *                 same as printf
 * @param level :  1 = RED, 2 = GREEN, 3 = YELLOW 4 = BLUE 5 = WHITE
 * 
 * if NoColor was set, output is always WHITE.
 *********************************************************************/
void p_printf(int level, char *format, ...) {
    
    char    *col;
    int     coll=level;
    va_list arg;
    
    //allocate memory
    col = (char *) malloc(strlen(format) + 20);
    
    if (NoColor) coll = WHITE;
                
    switch(coll)
    {
    case RED:
        sprintf(col,REDSTR, format);
        break;
    case GREEN:
        sprintf(col,GRNSTR, format);
        break;      
    case YELLOW:
        sprintf(col,YLWSTR, format);
        break;      
    case BLUE:
        sprintf(col,BLUSTR, format);
        break;
    default:
        sprintf(col,"%s",format);
    }

    va_start (arg, format);
    vfprintf (stdout, col, arg);
    va_end (arg);

    fflush(stdout);

    // release memory
    free(col);
}
