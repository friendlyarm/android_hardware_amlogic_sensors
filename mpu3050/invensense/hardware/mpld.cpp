/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */

/**
 *  MPLd server code main entry
 */

#include "Impld.h"
#include <binder/IServiceManager.h>
#include <binder/IPCThreadState.h>
#include <cutils/native_handle.h>
#include <pthread.h>
#include <utils/Vector.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stddef.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/poll.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <linux/input.h>
#include <sys/select.h>

/*  *****************************
 *  includes for MPL              */
#include "sm_version.h"
#include "math.h"
#include "ml.h"
#include "mlFIFO.h"
#include "mlsl.h"
#include "mlos.h"
#include "mlsupervisor_9axis.h"
#include "kernel/mpuirq.h"
#ifndef MPL_3_1
#include "mlsetinterrupts.h"
#endif

#define MPLD_SEN_GYRO    0
#define MPLD_SEN_ACCEL   1
#define MPLD_SEN_COMPASS 2
#define MPLD_SEN_ROT_VEC 3
#define MPLD_SEN_LIN_ACC 4
#define MPLD_SEN_GRAVITY 5
#define MPLD_SEN_ORIENTATION 6
#define MPLD_NUM_SENSORS 7

#define SENSOR_STATUS_UNRELIABLE        0
#define SENSOR_STATUS_ACCURACY_LOW      1
#define SENSOR_STATUS_ACCURACY_MEDIUM   2
#define SENSOR_STATUS_ACCURACY_HIGH     3

typedef struct ts_inc t_sensor;

void gyro_handler(t_sensor* data);
void accel_handler(t_sensor* data);
void compass_handler(t_sensor* data);
void rv_handler(t_sensor* data);
void la_handler(t_sensor* data);
void grav_handler(t_sensor* data);
void orien_handler(t_sensor* data);

typedef struct {
    int token;
    int enabled_sensors;
    int socket;
} t_client_data;

static android::Vector< t_client_data > client_list;

typedef void (*t_data_handler)(t_sensor*);

struct ts_inc {              //incomplete type for ouput sensor data
    int enabled;             // enable/disable flag
    bool valid;              // data valid flag
    t_data_handler handler;  // function to convert mpl data to this sensor type
    int accuracy;            // accuracy indicator
    float data[3];           // sensor data
};

/* list of sensors exported to the java layer
 *   these correspond to the Java/HAL sensors
 */
static t_sensor sensor_list[] = {
    {0, false, gyro_handler, 0, {0,0,0}},
    {0, false, accel_handler, 0, {0,0,0}},
    {0, false, compass_handler, 0, {0,0,0}},
    {0, false, rv_handler, 0, {0,0,0}},
    {0, false, la_handler, 0, {0,0,0}},
    {0, false, grav_handler, 0, {0,0,0}},
    {0, false, orien_handler, 0, {0,0,0}}
};

/* threads and mutexes:
 *   there are three threads
 *     main thread -- runs the mpl UpdateData function and distributes data to clients
 *     socket thread -- listens for new client attachment
 *     server thread -- handles RPC requests (may not be strictly necessary)
 */
static pthread_mutex_t mpld_mutex      = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t idle_mutex      = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  mpld_idle_cond  = PTHREAD_COND_INITIALIZER;
static pthread_t socket_thread;
static pthread_t service_thread;

static bool exit_socket_thread = false;  //flag to tell the socket thread to exit
static bool new_sensor_active = false;   //flag to alert the main thread that a new sensor activated
static int mpu_accuracy;                 //global storage for the current accuracy status
static bool global_all_disabled_flag = true;  //flag to alert the main thread that all sensors are disabled
static int wake_socks[2];                //local socket pair used for the wake function
static bool wake_pending = false;        //flag to alert the main thread that a wake ipc call was made

struct pollfd *cli_fds = NULL;           //dynamic array used to track connected client sockets

/*
 * Create a UNIX-domain socket address in the Linux "abstract namespace".
 *
 * The socket code doesn't require null termination on the filename, but
 * we do it anyway so string functions work.
 */
int makeAddr(const char* name, struct sockaddr_un* pAddr, socklen_t* pSockLen)
{
    int nameLen = strlen(name);
    if (nameLen >= (int) sizeof(pAddr->sun_path) -1)  /* too long? */
        return -1;
    pAddr->sun_path[0] = '\0';  /* abstract namespace */
    strcpy(pAddr->sun_path+1, name);
    pAddr->sun_family = AF_LOCAL;
    *pSockLen = 1 + nameLen + offsetof(struct sockaddr_un, sun_path);
    return 0;
}

/**
 * keep the cli_fds up to date so we can monitor the socket connections
 */
void update_cli_fds() {

   // must be called with mpld mutex held
   uint32_t num_s_clients = client_list.size();
   uint32_t i;

   if(cli_fds)
       free(cli_fds);
   if(num_s_clients == 0) {
       cli_fds = NULL;
       return;
   }

   cli_fds = (struct pollfd*)malloc((num_s_clients)*sizeof(pollfd));
   memset(cli_fds, 0, (num_s_clients)*sizeof(pollfd));
   for(i=0; i < client_list.size(); i++) {
       cli_fds[i].fd = client_list[i].socket;
       cli_fds[i].events = POLLIN;
   }
}

/* thread used to service incoming socket connections */
void* socket_thread_func(void *ptr)
{
    struct sockaddr_un sockAddr;
    socklen_t sockLen;
    int result = 1;
    char buf[64];
    int clientSock;
    struct pollfd sockfds[1];
    int fd=0;

    memset(sockfds, 0, sizeof(struct pollfd));

    //set up sensor socket
    if (makeAddr("mpld1", &sockAddr, &sockLen) < 0) {
        LOGD("Error: cant make server address");
        return NULL;
    }
    fd = socket(AF_LOCAL, SOCK_STREAM, PF_UNIX);
    if (fd < 0) {
        LOGD("Error: call to socket() failed");
        return NULL;
    }

    LOGD("MPLD SERVER %s\n", sockAddr.sun_path+1);
    
    if (bind(fd, (const struct sockaddr*) &sockAddr, sockLen) < 0) {
        LOGD("Error: server thread bind() failed");
        goto bail;
    }
    
    if (listen(fd, 5) < 0) {
        LOGD("Error: server thread listen() failed");
        goto bail;
    }
    
    //set up for poll()

    sockfds[0].fd = fd;
    sockfds[0].events = POLLIN;

    while(!exit_socket_thread) {
        poll(sockfds, 1, -1);
        if (sockfds[0].revents & POLLIN) {
            sockfds[0].revents &= ~POLLIN;

            clientSock = accept(fd, NULL, NULL);
            if (clientSock < 0) {
                LOGD("Error: server thread accept() failed");
                goto bail;
            }
            LOGD("accepted sensor client");
            pthread_mutex_lock(&mpld_mutex);
            {
                int ci = client_list.add();
                t_client_data &cli = client_list.editItemAt(ci);
                cli.token = ci;
                cli.enabled_sensors = 0;
                cli.socket = clientSock;
                write(clientSock, &ci, sizeof(ci)); //send the client its token
                fcntl(cli.socket, F_SETFL, O_NONBLOCK);  //make the client socket nonblocking so he cant hang us
                update_cli_fds();
                LOGD("mpld server thread: new client %d", ci);
            }
            pthread_mutex_unlock(&mpld_mutex);
        }
    }

bail:
    close(fd);

    return NULL;        
}

/* check for the condition that all sensors are disabled
 *   when the condition is met, the mpld can go into its idle state
 */
int all_disabled() {
    uint32_t i;
    bool found = false;
    for(i=0;i<MPLD_NUM_SENSORS;i++) {
       if(sensor_list[i].enabled) {
           global_all_disabled_flag=false;
           found = true;
           break;
       }
    }

    if(!found) {
        global_all_disabled_flag = true;
    }

    return global_all_disabled_flag;
}

/**
 * enable or disable a single sensor.  used by the RPC side
 */
int activate_sensor(int client, int sensorType, int activate) {
    int rv = 0;
    int sen_active;
    int idle = all_disabled();
    
    sen_active = sensor_list[sensorType].enabled > 0;
    
    LOGD("mpld activate sensor sen %d  (curr %d) %d", sensorType, sen_active, activate);
    
    if(activate && !sen_active) {
        new_sensor_active = true;
        sensor_list[sensorType].enabled = 1;
    }    
    
    if(!activate && sen_active) {
        sensor_list[sensorType].enabled = 0;
        new_sensor_active = true;
    }
    
    all_disabled();

    if(idle) {
        pthread_cond_signal(&mpld_idle_cond);
    }

    return rv;
}

/* skeleton function for controlling power states at a finer granularity.
 *  this function is currently unused and will not work until power control
 *  apis are added to the MPL.
 */
void set_power_states () {

    if(sensor_list[3].enabled || sensor_list[4].enabled || sensor_list[5].enabled) {
        //everything on
        
        return;
    }

    if(!sensor_list[0].enabled && !sensor_list[1].enabled && !sensor_list[2].enabled) {
        //everything off
        
        return;
    } 
    
    if(sensor_list[0].enabled) {
        //gyro on
    } else {
        //sleep gyro
    }
    
    if(sensor_list[1].enabled) {
        //accel on
    } else {
        //sleep accel
    }

    if(sensor_list[2].enabled) {
        //compass on
    } else {
        //sleep compass   
    }
}
        
namespace android {
/**
 * implementation of the Impld interface (server side).
 */
class mpldService : public Bnmpld {
public:
  static void instantiate();
  mpldService();
  virtual ~mpldService();
  virtual int getSocket();
  virtual int activateSensor(int, int, int);
  virtual int wake();
private:
  int dummy;
};

int mpldService::getSocket() 
{
  return dummy;
}

int mpldService::activateSensor(int client, int sensorType, int activate)
{
    int rv = 0;
  
    pthread_mutex_lock(&mpld_mutex);
    
    rv = activate_sensor(client, sensorType, activate);
    //set_power_states();
    pthread_mutex_unlock(&mpld_mutex);
    return rv;
}

/** force a client (which may be blocked waiting for data) to wake up.
 * android uses this functionality to ensure that deadlocks don't happen.
 */
int mpldService::wake() {
    char m = 0;
    pthread_mutex_lock(&mpld_mutex);
    write(wake_socks[1], &m, 1);
    wake_pending = true;
    pthread_mutex_unlock(&mpld_mutex);
    return 1;
}

/** hook into the android RPC mechanism
 * */
void mpldService::instantiate()
{
  status_t status;
  status = defaultServiceManager()->addService(String16("vendor.invensense.mpld"), new mpldService()); 
}

mpldService::mpldService()
{
    /* initialization needed for IPC interface */
    dummy = 42;
}

mpldService::~mpldService()
{

}

} //namespace android

/** thread used for android RPC service
 * */
void* service_thread_func(void *ptr)
{
    // Binder based IPC
    android::mpldService::instantiate();
    
    //Create binder threads for this "server"
    android::ProcessState::self()->startThreadPool();
    LOGD("Server is up and running");
    android::IPCThreadState::self()->joinThreadPool(false);
    return NULL;

}


/**
 * utility function for (more or less) cleanly exiting the mpld.
 */
void serverExit() {
    /* clean up -- close file handles, etc */
    perror(NULL);
    exit(-1);
}

/* ********************** 
 * MPL interface functions
 */

extern "C" { 
    void initMPL();
    void setupInterrupts();
    void setupCallbacks();
    void setupFIFO();
    void cb_onMotion(uint16_t);
    void mpld_idle();
    void mpld_active();
}

/**
 * container function for all the calls we make once to set up the MPL.
 */
void initMPL()
{
    char *port = NULL;
    tMLError result;
        
    if( MLSerialOpen(port) != ML_SUCCESS ) {
        serverExit();
    }

    if( MLDmpOpen() != ML_SUCCESS ) {
        LOGD("Fatal Error : could not open DMP correctly.\n");
        serverExit();
    }
    
    if( MLEnable9axisFusion() != ML_SUCCESS ) {
        LOGD("Warning : 9 axis sensor fusion not available - No compass detected.\n");
    }
    
    if( MLSetBiasUpdateFunc(ML_ALL) != ML_SUCCESS ) {
        LOGD("Error : Bias update function could not be set.\n");
        serverExit();
    }
    
    if( MLSetInterrupts(ML_INT_MOTION | ML_INT_FIFO) != ML_SUCCESS) {
        LOGD("Error : could not set interrupts 0 ");
        if( MLSetInterrupts(ML_INT_MOTION | ML_INT_FIFO) != ML_SUCCESS) {
            LOGD("Error : could not set interrupts 1");
            serverExit();
        }
    }
    
    // @todo any reason to have variable FIFO rate?
	result = MLSetFIFORate(6);
    if (result != ML_SUCCESS) {
        LOGD("Fatal error: MLSetFIFORate returned %d\n",result);
        serverExit();
    }
    
    result = MLSetDataMode(ML_DATA_FIFO);
    if (result != ML_SUCCESS) {
        printf("Fatal error: MLSetDataMode returned %d\n",result);
        serverExit();
    }
    
    mpu_accuracy = SENSOR_STATUS_ACCURACY_MEDIUM;
}

/** setup the fifo contents based on the active sensors.
 * this function can only be called when the mpld thread holds the mpld_mutex
 */
void setupFIFO()
{
    static int32_t q_act=0, g_act=0, a_act=0;
    tMLError result;
    
    if(sensor_list[MPLD_SEN_ROT_VEC].enabled || 
       sensor_list[MPLD_SEN_LIN_ACC].enabled ||
       sensor_list[MPLD_SEN_GRAVITY].enabled ||
       sensor_list[MPLD_SEN_ORIENTATION].enabled) {
       
	    result = FIFOSendQuaternion(ML_32_BIT);
	    q_act++;
        if (result != ML_SUCCESS) {
            LOGD("Fatal error: FIFOSendQuaternion returned %d\n",result);
            serverExit();
        }
    } else if(q_act>0){
       
	    result = FIFOSendQuaternion(0);
	    q_act--;
        if (result != ML_SUCCESS) {
            LOGD("Fatal error: FIFOSendQuaternion returned %d\n",result);
            serverExit();
        }
    }
    
    if(sensor_list[MPLD_SEN_GYRO].enabled) {
        
    	result = FIFOSendGyro(ML_ALL, ML_32_BIT);
    	g_act++;
        if (result != ML_SUCCESS) {
            LOGD("Fatal error: FIFOSendGyro returned %d\n",result);
            serverExit();
        }
    } else if (g_act>0){
        
    	result = FIFOSendGyro(ML_ALL, 0);
    	g_act--;
        if (result != ML_SUCCESS) {
            LOGD("Fatal error: FIFOSendGyro returned %d\n",result);
            serverExit();
        }
    }   
    
    if(sensor_list[MPLD_SEN_ACCEL].enabled) {
    
    	result = FIFOSendAccel(ML_ALL, ML_32_BIT);
    	a_act++;
        if (result != ML_SUCCESS) {
            LOGD("Fatal error: FIFOSendAccel returned %d\n",result);
            serverExit();
        }
    } else if(a_act>0) {
    
    	result = FIFOSendAccel(ML_ALL, 0);
        if (result != ML_SUCCESS) {
            LOGD("Fatal error: FIFOSendAccel returned %d\n",result);
            serverExit();
        }
    }

}

/**
 *  set up the callbacks that we use in all cases
 */
void setupCallbacks()
{
    if( MLSetMotionCallback(cb_onMotion) != ML_SUCCESS) {
        LOGD("Error : Motion callback could not be set.\n");
        serverExit();
    }
}

/**
 * handle the motion/no motion output from the MPL.
 */
void cb_onMotion(uint16_t val) {
    //after the first no motion, the gyro should be calibrated well
    if(!val) {
        mpu_accuracy = SENSOR_STATUS_ACCURACY_HIGH;
    }

    return;
}

/**
 * configure the mpu interrupts.
 */
void setupInterrupts()
{
    tMLError result;
    result = MLSetInterrupts(ML_INT_FIFO | ML_INT_MOTION);
    if(result != ML_SUCCESS) {
        LOGD("Fatal error: MLSetInterrupts failed.");
    }
}

/**
 * put the mpl into idle state and wait for someone to signal the condition
 * variable (mpld_idle_cond).
 */
void mpld_idle()
{
    int rc;
    //must be called with mpld mutex lock held
    
    while(all_disabled()) {
        LOGD("mpld idle");
        rc = pthread_cond_wait(&mpld_idle_cond, &mpld_mutex);
    }
    //we still hold the mutex at exit
}

/**
 * This function implements the data distribution part of the mpld.
 */
void mpld_active()
{
    int32_t rc;
    uint32_t i, j;
    int abort = 0;

    //must be called with mpld mutex held
    
    //get the data for each enabled sensor
    for(i=0; i < MPLD_NUM_SENSORS; i++) {
        if(sensor_list[i].enabled) {
            sensor_list[i].handler(&sensor_list[i]);
        }
    }
    
    //send the sensor data to interested clients
    for(j=0; j < client_list.size(); j++) {
        t_client_data cli = client_list[j];
        for(i=0;i<MPLD_NUM_SENSORS; i++) {
            if(sensor_list[i].enabled && sensor_list[i].valid) {
                uint8_t packet[20];
                uint8_t *pp=packet;
                ssize_t wr;
                memcpy(pp, &i, sizeof(i)); pp += sizeof(i);
                memcpy(pp, &(sensor_list[i].accuracy), sizeof(int)); pp += sizeof(int);
                memcpy(pp, &(sensor_list[i].data[0]),  sizeof(float)); pp+= sizeof(float);
                memcpy(pp, &(sensor_list[i].data[1]),  sizeof(float)); pp+= sizeof(float);
                memcpy(pp, &(sensor_list[i].data[2]),  sizeof(float));
                wr = write(cli.socket, packet, 20);
                if(wr < 20) {
                    LOGE("problem writing sensor client %d", j);
                    close(cli.socket);
                    client_list.removeAt(j);
                    update_cli_fds();
                    abort = 1;
                    break;
                }
            }
            if(abort) break;
        }
    }
    abort = 0;
    //mark the sensor data invalid so it is not sent twice
    for(i=0; i < MPLD_NUM_SENSORS; i++) {
        if(sensor_list[i].enabled) {
            sensor_list[i].valid = false;
        }
    }
}

//these handlers transform mpl data into one of the Android sensor types
//  scaling and coordinate transforms should be done in the handlers

void gyro_handler(t_sensor* s) 
{
    tMLError res;
    res = MLGetFloatArray(ML_GYROS, s->data);
    s->data[0] = s->data[0] * M_PI/180.0;
    s->data[1] = s->data[1] * M_PI/180.0;
    s->data[2] = s->data[2] * M_PI/180.0;
    s->accuracy = mpu_accuracy;
    if(res == ML_SUCCESS)
        s->valid = true;
}

void accel_handler(t_sensor* s)
{
    tMLError res;
    res = MLGetFloatArray(ML_ACCELS, s->data);
    s->data[0] = s->data[0] * 9.81;
    s->data[1] = s->data[1] * 9.81;
    s->data[2] = s->data[2] * 9.81;
    s->accuracy = SENSOR_STATUS_ACCURACY_HIGH;
    if(res == ML_SUCCESS)
        s->valid = true;
}

void compass_handler(t_sensor* s)
{
    tMLError res, res2;
    float bias_error[3];
    float total_be;
    static int bias_error_settled=0;
    
    res = MLGetFloatArray(ML_MAGNETOMETER, s->data);
    
    if(res != ML_SUCCESS) {
        LOGD("compass_handler MLGetFloatArray(ML_MAGNETOMETER) returned %d", res);
    }

    if(!bias_error_settled) {
        res2 = MLGetFloatArray(ML_MAG_BIAS_ERROR, bias_error);
        
        if(res2 == ML_SUCCESS) {
            //use total of bias errors to estimate sensor accuracy
            total_be = bias_error[0] + bias_error[1] + bias_error[2];
            if(total_be > 2700.0)
                s->accuracy = SENSOR_STATUS_UNRELIABLE;
            else if(total_be > 1500.0)
                s->accuracy = SENSOR_STATUS_ACCURACY_LOW;
            else if(total_be > 300.0)
                s->accuracy = SENSOR_STATUS_ACCURACY_MEDIUM;
            else {
                s->accuracy = SENSOR_STATUS_ACCURACY_HIGH;
                bias_error_settled=1;
            }            
        } else {
           LOGE("mpld could not get mag bias error");
           s->accuracy = SENSOR_STATUS_ACCURACY_HIGH;
        }
    } else {
        s->accuracy = SENSOR_STATUS_ACCURACY_HIGH;
    }
    
    if(res == ML_SUCCESS)
        s->valid = true;
}

void rv_handler(t_sensor* s)
{
    float quat[4];
    float norm = 0;
    float ang = 0;
    tMLError r;
    
    r = MLGetFloatArray(ML_QUATERNION, quat);
    
    if(r != ML_SUCCESS) {
        s->valid = false;
        return;
    } else {
        s->valid = true;
    }
        
    if(quat[0] < 0.0) {
      quat[1] = -quat[1];
      quat[2] = -quat[2];
      quat[3] = -quat[3];
    }
    
    s->data[0] = quat[1];
    s->data[1] = quat[2];
    s->data[2] = quat[3];
      
    s->accuracy = mpu_accuracy;
}

void la_handler(t_sensor* s)
{
    tMLError res;
    res = MLGetFloatArray(ML_LINEAR_ACCELERATION, s->data);
    s->data[0] *= 9.81;
    s->data[1] *= 9.81;
    s->data[2] *= 9.81;
    s->accuracy = mpu_accuracy;
    if(res == ML_SUCCESS)
        s->valid = true;    
}

void grav_handler(t_sensor* s)
{
    tMLError res;
    res = MLGetFloatArray(ML_GRAVITY, s->data);
    s->data[0] *= 9.81;
    s->data[1] *= 9.81;
    s->data[2] *= 9.81;
    s->accuracy = mpu_accuracy;
    if(res == ML_SUCCESS)
        s->valid = true;    
}

//pick up the ComputeAndOrientation function from the mlsdk
#include "mlsdk/mlutils/and_orient_helper.c"

void orien_handler(t_sensor* s) //note that this is the handler for the android 'orientation' sensor, not the mpl orientation output
{
    tMLError r1, r2;
    float euler[3];
    float heading[1];

    r1 = MLGetFloatArray(ML_EULER_ANGLES, euler);
    r2 = MLGetFloatArray(ML_HEADING, heading);

    ComputeAndOrientation(heading[0], euler, s->data);
    s->accuracy = mpu_accuracy;

    if((r1 == ML_SUCCESS) && (r2 == ML_SUCCESS))
        s->valid = true;
    else
        LOGD("orien_handler: data not valid (%d %d)", (int)r1, (int)r2);

}

using namespace android;

/**
 * check all the clients and see if one has disconnected.
 */
void check_client_disconnects() {
    //client management
    //must be called with mpld mutex lock
    bool changed = false;
    uint32_t num_s_clients = client_list.size();
    int32_t i;

    if(cli_fds == NULL)
        return; //nothing to do

    poll(cli_fds, num_s_clients, 0);

    for(i=num_s_clients-1;i>=0;i--) {  //go backwards -- otherwise the indexes don't match after the first remove
        if(cli_fds[i].revents & POLLHUP) {
            //client[i] disconnected
            LOGD("sensor client %d disconnect", i);
            close(client_list[i].socket);
            client_list.removeAt(i);
            //we rely on the Android java framework to maintain the enable/disable state of the sensors
            changed = true;
            cli_fds[i].revents &= ~POLLHUP;
        }
    }

    if(changed) {
        update_cli_fds();
    }
}

/** implements the main loop of the mpld.  it is a standalone function so that mpld can be
 *   made into a library.
 */
int mpld_main_thread(void)
{
    unsigned char *verStr;
    int32_t mpu_int_fd, pt_ret;
    tMLError rv;
    struct pollfd ufds[2];
    mpuirq_data irq_data;

    //put the ver strings in the debug log
    MLVersion(&verStr);
    LOGD(SM_VERSION);
    LOGD("%s\n", verStr);

    socketpair(AF_UNIX, SOCK_STREAM, 0, wake_socks);  //open local sockets used for the wake call
    
    //check if MPU interrupts are available
    mpu_int_fd = open("/dev/mpuirq", O_RDWR);
    if(mpu_int_fd == -1) {
        LOGD("mpld: could not open the mpu irq device node");
        //since the interrupts are not there, we'll use a polling mode later on
    }
    
    //set up the struct that is passed to poll
    ufds[0].fd = mpu_int_fd;   
    ufds[0].events = POLLIN;
    
    ufds[1].fd = wake_socks[0];
    ufds[1].events = POLLIN;

    //MPL setup
    initMPL();
    setupCallbacks();
    
    rv = MLDmpStart();
    if(rv != ML_SUCCESS) {
        LOGD("Fatal error: could not start the DMP correctly. (code = %d)\n", rv);
        serverExit();
    }
    
    //launch the other two threads
    pt_ret = pthread_create( &socket_thread, NULL, socket_thread_func, NULL);
    if(pt_ret != 0) {
        LOGD("Fatal error: could not start socket listener thread. (code = %d)", pt_ret);
        serverExit();
    }
    
    pt_ret = pthread_create( &service_thread, NULL, service_thread_func, NULL);
    if(pt_ret != 0) {
        LOGD("Fatal error: could not start service thread. (code = %d)", pt_ret);
        serverExit();
    }
    
    pt_ret = pthread_mutex_lock(&mpld_mutex);  //after this, we never voluntarily exit

    while(1)
    { 
        tMLError result;
        int rc, irq, nread, i;

        check_client_disconnects();
               
#if 0
        // here, we check to see if the daemon should go into its idle state       
        if(!wake_pending && global_all_disabled_flag) {
            if(MLDmpStop() != ML_SUCCESS) {
                LOGD("Error: could not stop the DMP correctly.\n");
            }    
            mpld_idle();
            if(MLDmpStart() != ML_SUCCESS) {
                LOGD("Error: could not start the DMP correctly.\n");
            }
            continue;
        }
#endif
        
        if( new_sensor_active ) {  //sensor state change
            setupFIFO();
            new_sensor_active = false;   
        }
        
        rc = pthread_mutex_unlock(&mpld_mutex); //release the mutex before calling poll
        
        //here is where we wait for data to be ready (or just wait if the irq is not in use)
        rc = poll(ufds + (mpu_int_fd == -1), 1+(mpu_int_fd >= 0), (mpu_int_fd == -1) ? 1000/30 : -1);

        pthread_mutex_lock(&mpld_mutex);
        check_client_disconnects();
        all_disabled();

        if(rc < 0) {
            char msg[80];
            strerror_r(errno, msg, 80);
            LOGE("poll for mpl data interrupted (%s)",msg);
            continue;
        }
        
        if((mpu_int_fd == -1) || (ufds[0].revents & POLLIN) ) {  //mpl data processing
            if(mpu_int_fd >= 0) {
                nread = read(mpu_int_fd, &irq_data, sizeof(irq_data));
             }   
            
            result = MLUpdateData();
            if(result != ML_SUCCESS) {
                LOGD("MLUpdateData return code %d", result);
            }     
            mpld_active();
        }

        if(ufds[1].revents) { //wake requested, force clients waiting for sensor data to wake up
            uint32_t j;
            int32_t msg[5] = {255, 0, 0, 0, 0};  //fake message for clients
            uint8_t m = 0;
            LOGD("mpld main thread -- wake received");
            nread = read(ufds[1].fd, &m, 1); //consume the wake request
            LOGD("wake marker consumed");
            for(j=0; j< client_list.size(); j++) {
                int k;
                //this needs to change if the data sent to clients changes (see mpld_active)
                k = write(client_list[j].socket, &msg, 20);
                LOGD("wrote wake packet to client (rv = %d)", k);
            }
            ufds[1].revents = 0;
            wake_pending = false;
        }

        all_disabled();
    }
    
    pt_ret = pthread_mutex_unlock(&mpld_mutex);
    return 0;
}

int main(int argc, char** argv)
{
    mpld_main_thread();
}
