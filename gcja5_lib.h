/****************************************************************
 *
 * The SN-GCJA5 measures particles and provides mass density (PM1.0, PM2.5 and PM10)
 * as well as particle count ( PM0.5m, PM1, PM2.5, PM5, PM7.5 and Pm10)
 *
 * Parts of the definition is based on Sparkfun's original library for Arduino-variants
 * SparkFun sells these at its website: www.sparkfun.com
 * Help support SparkFun. Buy a board! https://www.sparkfun.com/products/17123
 *
 * version 1.0 / november 2020 /
 * by Paul van Haastrecht (paulvha@hotmail.com)
 *
 * initial Raspberry Pi
 *
 * Resources / dependencies:
 * BCM2835 library (http://www.airspayce.com/mikem/bcm2835/) in case of I2C
 *
 * For more details see sngcja5_on_raspberry.odt
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
 *
 ********************************************************************/

#ifndef __GCJA5_H__
#define __GCJA5_H__

# include <getopt.h>
# include <signal.h>
# include <stdint.h>
# include <stdarg.h>
# include <stdio.h>
# include <stdlib.h>
# include <string.h>
# include <unistd.h>
# include <time.h>
# include <math.h>
# include <termios.h>
# include <fcntl.h>

/////////////////////////////////////////////////////////////////
// IF YOU WANT TO USE SERIAL ONLY DEFINE THAT HERE
// THE I2C RELATED STUFF WILL BE EXCLUDED FROM THE FINAL CODE
//////////////////////////////////////////////////////////////////
//#define SERIAL_ONLY_NO_I2C 1


/////////////////////////////////////////////////////////////////
// IF YOU WANT TO USE I2C ONLY DEFINE THAT HERE
// THE SERIAL RELATED STUFF WILL BE EXCLUDED FROM THE FINAL CODE
//////////////////////////////////////////////////////////////////
//#define I2C_ONLY_NO_SERIAL 1

/* set version number */
# define VERSIONMAJOR 1
# define VERSIONMINOR 0

#if !defined(SERIAL_ONLY_NO_I2C)

// library for I2c
# include <bcm2835.h>

/* The default I2C address for the GCJA5 is 0x33 */
# define GCJA5_ADDRESS 0x33

/* default speed 50 Khz*/
# define GCJA5_SPEED 50

// For setting correct I2C speed, set your Raspberry Pi ( 2 ,3 or 4 )
#define RASPBERRY_PI_BOARD 4

#if RASPBERRY_PI_BOARD == 4
#define PI_CORE_CLOCK 400000

#elif RASPBERRY_PI_BOARD == 3
#define PI_CORE_CLOCK 300000

#else  // older boards
#define PI_CORE_CLOCK 250000
#endif // RASPBERRY_PI_BOARD

#endif //SERIAL_ONLY_NO_I2C

// size of different buffers
#define MAXBUF 100

/* error codes */
#define GCJA5_ERR_OK          0x00
#define GCJA5_ERR_DATALENGTH  0X01
#define GCJA5_ERR_UNKNOWNCMD  0x02
#define GCJA5_ERR_ACCESSRIGHT 0x03
#define GCJA5_ERR_PARAMETER   0x04
#define GCJA5_ERR_OUTOFRANGE  0x28
#define GCJA5_ERR_CMDSTATE    0x43
#define GCJA5_ERR_TIMEOUT     0x50
#define GCJA5_ERR_PROTOCOL    0x51


enum SNGCJA5_REGISTERS {
        SNGCJA5_PM1_0 = 0x00,
        SNGCJA5_PM2_5 = 0x04,
        SNGCJA5_PM10 = 0x08,
        SNGCJA5_PCOUNT_0_5 = 0x0C,
        SNGCJA5_PCOUNT_1_0 = 0x0E,
        SNGCJA5_PCOUNT_2_5 = 0x10,
        SNGCJA5_PCOUNT_5_0 = 0x14,
        SNGCJA5_PCOUNT_7_5 = 0x16,
        SNGCJA5_PCOUNT_10 = 0x18,
        SNGCJA5_STATE = 0x26,
};

class GCJA5
{
  public:

        /*! constructor */
        GCJA5(void);

        /*! Initialize the hardware library and check fro SN-GCJA5
         *
         * @return  true = OK, false is error
         */
        bool begin(char *device=NULL);

        /*! set clock speed */
        void SetClock(uint16_t clock);

        /*! get the PM density numbers */
        float getPM1_0();
        float getPM2_5();
        float getPM10();

        /*! get the particle counts (TOTAL) */
        uint16_t getPC0_5(bool check=true);  // 0.3 - 0.5
        uint16_t getPC1_0(bool check=true);  // 0.3 - 1.0
        uint16_t getPC2_5(bool check=true);  // 0.3 - 2.5
        uint16_t getPC5_0(bool check=true);  // 0.3 - 5
        uint16_t getPC7_5(bool check=true);  // 0.3 - 7.5
        uint16_t getPC10(bool check=true);   // 0.3 - 10

        /*! get the status information */
        uint8_t getState();
        uint8_t getStatusSensors();
        uint8_t getStatusPD();
        uint8_t getStatusLD();
        uint8_t getStatusFan();

        /*! close driver and release memory */
        void ConnClose(void);

        /*! enable debug messages */
        void setDebug(int val);

  private:
        /*! Central call for PM values */
        float getPM(uint8_t pmRegister);

        /*! display debug messages */
        void debug_cmd(uint16_t command);

        /*! low level communication routines */
        uint8_t readMeasurement();
        uint8_t readRegister8(uint8_t addr);
        uint16_t readRegister16(uint8_t addr);
        uint32_t readRegister32(uint8_t addr);

        /*! keeps track whether to read from Sensor */
        bool Pm1HasBeenReported = true;
        bool Pm25HasBeenReported = true;
        bool Pm10HasBeenReported = true;

        bool Pc05HasBeenReported = true;
        bool Pc1HasBeenReported = true;
        bool Pc25HasBeenReported = true;
        bool Pc5HasBeenReported = true;
        bool Pc75HasBeenReported = true;
        bool Pc10HasBeenReported = true;

        bool StateHasBeenReported = true;
        bool StSenHasBeenReported = true;
        bool StPdHasBeenReported = true;
        bool StLdHasBeenReported = true;
        bool StFanHasBeenReported = true;

        /*! I2C routines  */
        uint8_t I2C_init();
        void I2C_close();
        void I2C_SetClock(uint16_t clock);
        uint8_t I2C_Read(uint8_t address, uint8_t cnt);

#if !defined(I2C_ONLY_NO_SERIAL)

        /*! variables for Serial communication */
        bool _UseSerial = false;    // indicate to use serial
        char _SerialDevice[MAXBUF]; // holds a serial device to open
        int _fd = 0;                // file descriptor to device
        bool _connected = false;    // indicate connected & initialised
        struct termios _old_options;// save & restore current port setting
        #define STX 0x02
        #define ETX 0x03

        /*! Serial routines  */
        uint8_t Ser_init();
        uint8_t Ser_configure();
        void Ser_close();
        uint8_t Ser_read(uint8_t wait);
#endif
};

/*! to display in color  */
void p_printf (int level, char *format, ...);

// color display enable
#define RED     1
#define GREEN   2
#define YELLOW  3
#define BLUE    4
#define WHITE   5

#define REDSTR "\e[1;31m%s\e[00m"
#define GRNSTR "\e[1;92m%s\e[00m"
#define YLWSTR "\e[1;93m%s\e[00m"
#define BLUSTR "\e[1;34m%s\e[00m"

// disable color output
extern bool NoColor;

#endif  // End of definition check
