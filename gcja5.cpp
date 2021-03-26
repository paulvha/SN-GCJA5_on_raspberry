/****************************************************************
 * 
 * The SN-GCJA5 measures particles and provides mass density (PM1.0, PM2.5 and PM10) 
 * as well as particle count (PM0.5m, PM1, PM2.5, PM5, PM7.5 and Pm10)
 * 
 * SparkFun sells these at its website: www.sparkfun.com
 * Help support SparkFun. Buy a board! https://www.sparkfun.com/products/17123
 *
 * version 2.0 / March 2021 / paulvha
 * - added option -k to keep SDA/SCL at certain speed on exit
 * - added option -A to use kernel device (/dev/i2c-#) instead of BCM2835
 * - update documentation
 * 
 * version 1.0 / november 2020 /  
 * by Paul van Haastrecht (paulvha@hotmail.com)
 * 
 * -initial Raspberry Pi
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
 **********************************************************************/

#include "gcja5_lib.h"

#ifdef SDS011	// set in makefile

#include "sds011/sdsmon.h"
SDSmon SDSm;

typedef struct sds
{
    char    port[MAXBUF];   // connected port (like /dev/ttyUSB0)
    bool    include;        // true = include
    float   value_pm25;     // measured value sds
    float   value_pm10;     // measured value sds
} sds;

#endif //SDS011

/* global constructor */ 
GCJA5 MySensor;

char progname[20];

struct gcja5_par
{
    /* option program variables */
    uint16_t loop_count;        // number of measurement
    uint16_t loop_delay;        // loop delay in between measurements
    bool timestamp;             // include timestamp in output
    int verbose;                // verbose level
    uint8_t baudrate;		// I2C speed
    bool keepPins;		// keep or reset GPIO 3 and 5
    uint16_t ExitSpeed;		// set I2C speed on exit
    uint8_t adapter;		// use /dev/i2c-x
    bool include_particle;      // include particle count in output
    bool relation;              // include correlation calc (SDS)
    bool condition;		// display status info
    char SerDevice[MAXBUF];	// holds the name of the serial device to use
    bool UseSerial;		// true is use serial communication	

#ifdef SDS011                   // SDS monitor option
    /* include SDS info */
    struct sds sds;
#endif

};

/*********************************************
 * @brief generate timestamp
 * 
 * @param buf : returned the timestamp
 *********************************************/  
void get_time_stamp(char *buf)
{
    time_t ltime;
    struct tm *tm ;
    
    ltime = time(NULL);
    tm = localtime(&ltime);
    
    static const char mon_name[][4] = {
    "Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

    sprintf(buf, "%.3s %3d %d %.2d:%.2d:%.2d",
    mon_name[tm->tm_mon], tm->tm_mday, 1900 + tm->tm_year, tm->tm_hour, 
    tm->tm_min, tm->tm_sec);
}

/**********************************************************
 * @brief display usage 
 * @param par : pointer to SN-GCJA5 parameters
 *********************************************************/
void usage(struct gcja5_par *par)
{
    printf(    "%s [options]  (version %d.%d) \n"
            
    "\nprogram options:\n"
#if defined(INCLUDE_KERNEL_I2C)
    "-A #       use kernel /dev/i2c-x not BCM2835       (default BCM2835)\n"
#endif
    "-B         do not display output in color\n"
    "-l #       number of measurements (0 = endless)    (default %d)\n"
    "-w #       waittime (seconds) between measurements (default %d)\n"
    "-v #       verbose/ debug level (0 - 2)            (default %d)\n"
    "-t         add timestamp to output                 (default %s)\n"
    "-p         include particle count                  (default %s)\n"
    "-c         add condition/status of the sensor      (default %s)\n"
    
#if defined(INCLUDE_SERIAL)      
    "-s #       enable serial communication port        (No default)\n"
#endif

#if defined(INCLUDE_BCM2835_I2C)
    "-i #       set I2C speed                           (default is %dkhz)\n" 
    "-k #       on exit keep GPIO SDA/SCL at speed #    (default %s)\n" 
#endif

#ifdef SDS011
    "\n\nSDS011 options:\n"
    "-S port    enable SDS011 input from port           (No default)\n"
    "-C         add correlation calculation             (default %s)\n"
#else
	"\n"
#endif
       
   ,progname, VERSIONMAJOR, VERSIONMINOR, par->loop_count, par->loop_delay, par->verbose,
    par->timestamp?"Timestamp":"No TimeStamp", par->include_particle?"added":"excluded",
    par->condition?"added":"excluded"

#if defined(INCLUDE_BCM2835_I2C)
    ,GCJA5_SPEED
    ,par->keepPins?"keep as SDA/SCL":"reset to input"
#endif

#ifdef SDS011
   ,par->relation?"added":"excluded");
#else
   );
#endif
}

/*********************************************************************
*  @brief close hardware and program correctly
**********************************************************************/
void closeout()
{
   /* reset pins in Raspberry Pi */
   MySensor.ConnClose();

#ifdef SDS011       // SDS011 monitor
    SDSm.close_sds();
#endif   

   exit(EXIT_SUCCESS);
}

/**********************************************************
 * @brief initialise the Raspberry PI and check for SN-GCJA5 
 * @param par : pointer to SN-GCJA5 parameters
 *********************************************************/
void init_hw(struct gcja5_par *par)
{
    uint8_t ret;
    
    if (geteuid() != 0)
    {
	p_printf(RED,(char *)"You must be super user\n");
	exit(EXIT_FAILURE);
    }
    
    /* progress & debug messages tell driver */
    MySensor.setDebug(par->verbose);

    /* use I2C kernel drivers ? */
    if (par->adapter > 0) {
	
	if (!MySensor.SetI2cDevice(par->adapter)){
	    p_printf(RED,(char *) "Error during init kernel device\n");
	    exit(-1);
	}
	
	printf("You need to set speed manual in /boot/config.txt\n");
	printf("Add a line dtparam=i2c_baudrate=50000  (50Khz)\n");
    }
    else
    {	// leave GPIO 5 and 3 on exit as SDA/SCL with certain speed
	if (par->keepPins){
	    
	    if (!MySensor.SetI2cExit(par->ExitSpeed)){
	    	p_printf(RED,(char *) "Can not set GPIO on exit and speed\n");
		exit(-1);
	    }
	}
    }
    
    /* start hardware and SN-GCJA5. Serial or I2C*/
    if (par->UseSerial) ret = MySensor.begin(par->SerDevice);
	else ret = MySensor.begin();
    
    if (! ret)
    {
        p_printf(RED,(char *) "Error during init communication\n");
        exit(-1);
    }
    
    // set I2C baudrate (ignored if initialized as serial comms)
    if (par->baudrate > 0) {
	/* set for requested baudrate */
	MySensor.SetI2cClock(par->baudrate);
    }
	
#ifdef SDS011  // SDS011 monitor
    if (par->sds.include) {
    
        if (par->verbose) p_printf (YELLOW, (char *) "initialize SDS011\n");
        
        if (SDSm.open_sds(par->sds.port, par->verbose) != 0) closeout();
        
        if (par->verbose) p_printf (YELLOW, (char *) "connected to SDS011\n");
    }
#endif // SDS011
}

/*********************************************************************
 * Parse parameter input 
 * @param par : pointer to SN-GCJA5 parameters
 *
 *********************************************************************/ 
void parse_cmdline(int opt, char *option, struct gcja5_par *par)
{
    switch (opt) {
    
    case 'A':   // use I2C with kernel drivers
	par->adapter = (uint8_t) strtod(option, NULL);
	
	if (par->adapter < 1) {
	    p_printf (RED, (char *) "Incorrect number of /dev/i2c-x.\n");
            exit(EXIT_FAILURE);
	}
	break;
	    
    case 'B':   // set for no color output
        NoColor = true;
        break;
	 
     case 'k':  // Keep GPIO as SDA/SCL with speed
        par->ExitSpeed = (uint16_t) strtod(option, NULL);
        
	if (par->ExitSpeed < 1 || par->ExitSpeed > 400) 
	    par->ExitSpeed = 100;
        
	par->keepPins = true;
        break;
	             
    case 'l':   // loop count
        par->loop_count = (uint16_t) strtod(option, NULL);
	if (par->loop_count < 1) par->loop_count = 1;
        break;
        
     case 'p':   // include particle count
        par->include_particle = true;
        break;
                 
    case 'w':   // loop delay in between measurements
        par->loop_delay = (uint16_t) strtod(option, NULL);
        if (par->loop_delay < 1) par->loop_delay = 1;
        break;
    
    case 't':  // Add timestamp to output
        par->timestamp = ! par->timestamp;
        break;
                
    case 'v':   // set verbose / debug level
        par->verbose = (int) strtod(option, NULL);

        // must be between 0 and 2
        if (par->verbose < 0 || par->verbose > 2)
        {
            p_printf (RED, (char *) "Incorrect verbose/debug. Must be  0,1, 2 \n");
            exit(EXIT_FAILURE);
        }
        break;

    case 'i':   // i2C Speed
        par->baudrate = (uint32_t) strtod(option, NULL);
        
        if (par->UseSerial){
            p_printf(RED,(char *) "Can not set i2C speed with Serial\n");
            exit(EXIT_FAILURE);
        }
        			 
        if (par->baudrate  < 1 || par->baudrate  > 400)
        {
            p_printf(RED,(char *) "Invalid i2C speed option %dKhz\n",par->baudrate);
            exit(EXIT_FAILURE);
        }
        break;
    
    case 's':   // use Serial communication
        strncpy(par->SerDevice, option, MAXBUF);
        par->UseSerial = true;
        break;
             
    case 'c':   // toggle status conditions
	par->condition = ! par->condition;        
	break;
		
    case 'S':   // include SDS011 read
#ifdef SDS011   
        strncpy(par->sds.port, option, MAXBUF);
        par->sds.include = true;
#else
        p_printf(RED, (char *) "SDS011 is not supported in this build\n");
#endif
        break;
    
    case 'C':   // toggle correlation calculation
#ifdef SDS011        
	par->relation = ! par->relation;
#else
        p_printf(RED, (char *) "SDS011 is not supported in this build\n");
#endif
        break; 
          
    case 'h':   // help  (No break)
    default: /* '?' */
        usage(par);
        exit(EXIT_FAILURE);
    }
}

/*****************************************************************
 * @brief Here the main of the program 
 * @param par : pointer to SN-GCJA5 parameters
 ****************************************************************/
void main_loop(struct gcja5_par *par)
{
    uint16_t  loop_set;
    bool      first=true;
    float     PM1, PM2, PM10;
    uint16_t  PC1, PC2, PC10;
    char	  timebuf[50];
    uint8_t   retry;
    
    /*  check for endless loop */
    if (par->loop_count > 0 ) loop_set = par->loop_count;
    else loop_set = 1;
   
    /* loop requested */
    while (loop_set > 0)  {
		
	// header here
	if (first){
	    if (par->timestamp) printf("\t\t\t");	
	    printf("== SN-GCJA5 MASS ===");

#ifdef SDS011			
	    if (par->sds.include)
		    printf("\t=== SDS011 ===");
#endif		
	    if (par->include_particle)
		    printf("\t== PARTICLE COUNT ==");
#ifdef SDS011
	    if (par->relation) 
		    printf("\t=Correlation=");
#endif
	    if (par->condition)
		    printf("\t===== Status =====");
	    
	    printf("\n");
	    
	    if (par->timestamp) printf("\t\t\t");
	    printf("PM1.0\tPM2.5\tPM10");
	    
#ifdef SDS011			
	    if (par->sds.include)
		    printf("\tPM2.5\tPM10");
#endif				
	    if (par->include_particle)
		    printf("\tPM1.0\tPM2.5\tPM10");
	    
#ifdef SDS011
	    if (par->relation) 
		    printf("\tPM2.5%%\tPM10%%");
#endif
	    if (par->condition)
		    printf("\tLD\tPD\tFan");

	    printf("\n");
	    
	    first = false;
	}
	
	// Get data. Sometimes the SN-GCJA5 sensor is out of sync and results
	// very high numbers. This is a retry oppertunity to get
	// corrected numbers
	
	retry = 0;
	while(retry++ < 3) {

	    PM1 = MySensor.getPM1_0();
	    PM2 = MySensor.getPM2_5();
	    PM10 = MySensor.getPM10();		
	    
	    if (PM1 > 2000|| PM2 > 2000 || PM10 > 2000) {
		    if (par->verbose >0) printf("retry SN-GCJA5 PM\n");
		    continue;
	    }
	    
	    if (par->include_particle){
		    
		PC1 = MySensor.getPC1_0();
		PC2 = MySensor.getPC2_5();
		PC10 = MySensor.getPC10();
		
		if (PC1 > 1000 || PC2 > 1000 || PC10 > 1000) {
		    if (par->verbose >0) printf("retry SN-GCJA5 PC\n");
		    continue;	
		}		
	    }
#ifdef SDS011			
	    if (par->sds.include) {
		    
		// read values from SDS
		if (SDSm.read_sds(&par->sds.value_pm25, &par->sds.value_pm10) != 0)
		{
		    if (par->verbose >0) printf("retry SDS011\n");
		    par->sds.value_pm25 = 0;
		    par->sds.value_pm10 = 0;
		    continue;
		}	
	    }
#endif				
	    break;
	}
	
	// add timestamp if requested
	if (par->timestamp) {
	    get_time_stamp(timebuf);
	    printf("%s\t", timebuf);
	}
	
	// display SN-GCJA5 PM numbers
	p_printf(YELLOW,(char *) "%2.2f\t%2.2f\t%2.2f",PM1, PM2, PM10 );
	
#ifdef SDS011			
	if (par->sds.include) {
	    p_printf(GREEN, (char *)"\t%2.2f\t%2.2f",
	    par->sds.value_pm25, par->sds.value_pm10 );
	}
#endif

	if (par->include_particle){
	    p_printf(YELLOW,(char *) "\t%d\t%d\t%d",PC1, PC2, PC10 );
	}
	
#ifdef SDS011
	float temp;
    
	// if relation is requested
	if(par->relation && par->sds.value_pm10  > 0) {
    
	    temp = par->sds.value_pm25 / PM2 -1 ;
	    p_printf(GREEN, (char *)"\t%2.2f%%", temp *100);
	    
	    temp = par->sds.value_pm10 / PM10 -1 ;
	    p_printf(GREEN, (char *)"\t%2.2f%%", temp *100);
	}
#endif

	if (par->condition) {
	    p_printf(YELLOW, (char *) "\t%x\t%x\t%x", MySensor.getStatusLD(), MySensor.getStatusPD(), MySensor.getStatusFan());
	}
	
	printf("\n");
	
	sleep(par->loop_delay);
	
	/* check for endless loop */
	if (par->loop_count > 0) loop_set--;
    }
    
    printf("Reached the loop count of %d.\nclosing down\n", par->loop_count);
}

/*********************************************************************
* @brief catch signals to close out correctly 
* @param  sig_num : signal that was raised
* 
**********************************************************************/
void signal_handler(int sig_num)
{
    switch(sig_num)
    {
        case SIGINT:
        case SIGKILL:
        case SIGABRT:
        case SIGTERM:
        default:
            printf("\nStopping SN-GCJA5 monitor\n");
            closeout();
            break;
    }
}

/*****************************************
 * @brief setup signals 
 *****************************************/
void set_signals()
{
    struct sigaction act;
    
    memset(&act, 0x0,sizeof(act));
    act.sa_handler = &signal_handler;
    sigemptyset(&act.sa_mask);
    
    sigaction(SIGTERM,&act, NULL);
    sigaction(SIGINT,&act, NULL);
    sigaction(SIGABRT,&act, NULL);
    sigaction(SIGSEGV,&act, NULL);
    sigaction(SIGKILL,&act, NULL);
}

/************************************************
 * @brief  initialise the variables 
 * @param par : pointer to SN-GCJA5 parameters
 ************************************************/
void init_variables(struct gcja5_par *par)
{
    par->loop_count = 10;          // number of measurement
    par->loop_delay = 5;           // loop delay in between measurements
    par->timestamp = false;        // NOT include timestamp in output
    par->verbose = 0;              // No verbose level
    par->include_particle=false;   // do not include particle count
    par->relation = false;         // display corelation SDS
    par->condition = false;	   // include status or not
    par->UseSerial = false;	   // use serial communication
    par->adapter = 0;		   // use I2C kernel drivers	
    par->keepPins = false;	   // reset GPIO 3 and 5 on exit
    par->ExitSpeed = 100;	   // reset I2C speed to this speed.	
    
#if defined(INCLUDE_SERIAL)  
    par->baudrate = GCJA5_SPEED;   // default 50Khz (only for BCM2835)
#endif

#ifdef SDS011
    /* SDS values */
    par->sds.include = false;
    par->sds.value_pm25 = 0;
    par->sds.value_pm10 = 0;
#endif
}

int main(int argc, char *argv[])
{
    int opt;
    struct gcja5_par par; 	// parameters
    
    /* set signals */
    set_signals(); 
 
    /* save name for (potential) usage display */
    strncpy(progname,argv[0],20);
    
    /* set the initial values */
    init_variables(&par);

    /* parse commandline */
    while ((opt = getopt(argc, argv, "A:Bl:k:v:w:ti:s:pS:Chc")) != -1)
    {
        parse_cmdline(opt, optarg, &par);
    }

    /* initialise hardware */
    init_hw(&par);
  
    /* main loop to read SN-GCJA5 results */
    main_loop(&par);
    
    closeout();
}
