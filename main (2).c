/******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2019 STMicroelectronics</center></h2>
  *
  * Licensed under ST MYLIBERTY SOFTWARE LICENSE AGREEMENT (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/myliberty
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied,
  * AND SPECIFICALLY DISCLAIMING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
******************************************************************************/

/*! \file Nordic.c
 *
 *  \author Joseph Getz/Haider Ali Siddiquee
 *
 *  \brief Implementation for UART3 communication.
 *
 */

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <dbus-1.0/dbus/dbus.h>
#include <math.h>
#include <pthread.h>
#include <syslog.h>
#include <signal.h>

#include "ndx_common.h"
#include "ndx_dbus.h"
#include "ndx_timer.h"

/*
******************************************************************************
                                PRIVATE DEFINE
******************************************************************************
*/

#define UART_ERR_NONE           0
#define UART_ERR_UNKNOWN        2
#define UART_ERR_IO             3
#define UART_ERR_FILE           4
#define UART_ERR_DEFAULT        5

#define NORDIC_COMMAND_ACK      0x01
#define NORDIC_COMMAND_NACK     0xff
#define NORDIC_COMMAND_DELAYED  0x02

#define NORDIC_COMMAND_ERROR    0x80
#define NORDIC_COMMAND_OK       0x40

#define LOOPSLEEP               10000
#define TX_SYNC_CHAR            0x3E
#define RX_SYNC_CHAR            0x3C

#define DBUS_NORDIC_BUS         "ndx.nordic.Bus"
#define DBUS_NORDIC_OBJECT      "/ndx/nordic/Object"
#define DBUS_NORDIC_TYPE        "ndx.nordic.Type"


#define SIM
/*
******************************************************************************
                                PRIVATE TYPES
******************************************************************************
*/

/*
******************************************************************************
                                PRIVATE VARIABLE
******************************************************************************
*/

typedef enum NORDIC_CMDS
{
    NORDIC_DEV_TEST = 0,
    NORDIC_WHEEL_MOTOR,
    NORDIC_ACTUATOR_MOTOR,
    NORDIC_FEED_MOTOR,
    NORDIC_OIL_PUMP,
    NORDIC_LYSIS_PUMP,
    NORDIC_LYSIS_CLUTCH,
    NORDIC_BLISTER_CLUTCH,
    NORDIC_AMP_CLUTCH,
    NORDIC_HEATER,
    NORDIC_UV_A_LED,
    NORDIC_UV_C_LYSIS,
    NORDIC_UV_C_AMP,
    NORDIC_VIS_LED,
    NORDIC_POTENTIOMETER_SENSE,
    NORDIC_CURRENT_SENSE,
    NORDIC_IR_SENSE,
    NORDIC_OIL_PRESSURE,
    NORDIC_LYSIS_PRESSURE,
    NORDIC_TEMP_STATUS,
    NORDIC_LAST_DEVICE
} NORDIC_DEVICE_T;

static const char               *device     = "/dev/ttymxc2"; //UART3
static int                      fd          = 0;
static int                      isUARTInit  = 0;
static uint8_t                  test_start;
static volatile sig_atomic_t    done        = 0;

/*
******************************************************************************
                                PRIVATE FUNCTIONS
******************************************************************************
*/

static int  nordic_init();
//static int nordic_deinit( void );
static int  nordic_Tx(const unsigned char *txData, int length);
//static int nordic_Rx(unsigned char *rxData, int length);
static void reply_to_dbus_method_call(DBusMessage* msg, DBusConnection* conn);
static int  set_interface_attribs(int fd, int speed);
static void signal_handler(int signal);
int         temprature_logging(char* temp_buf, FILE* log_file, time_t* ltime, struct tm* mytime);
static int  uart_init( void );
/*
******************************************************************************
                                PUBLIC VARIABLE
******************************************************************************
*/
/*
******************************************************************************
                                PUBLIC FUNCTION
******************************************************************************
*/
char  heater_data[10];
char  wheel_data[10];

uint32_t heater_sequence;
uint32_t wheel_sequence;

uint8_t  heater_flag = 0;
uint8_t  wheel_flag = 0;

/*
 ******************************************************************************
 *                              FUNCTION DEFINATION
 ******************************************************************************
 */

int main()
{
    DBusMessage* msg;
    DBusConnection* conn;
    DBusError err;

    int ret = 0;
    int rd_length;
    char rxData[50];
    char temp_buf[50];
    FILE*   log_file;
    time_t ltime;
    struct tm mytime;

    syslog(LOG_INFO, "starting nordic serice");

    signal(SIGHUP,  signal_handler);
    signal(SIGINT,  signal_handler);
    signal(SIGKILL, signal_handler); // can't catch?
    signal(SIGQUIT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGTSTP, signal_handler);

    nordic_init();
    printf("Listening for method calls\n");

    // initialise the error
    dbus_error_init(&err);

    // connect to the bus and check for errors
    conn = dbus_bus_get(DBUS_BUS_SYSTEM, &err);

    if (dbus_error_is_set(&err))
    {
        fprintf(stderr, "Connection Error (%s)\n", err.message);
        dbus_error_free(&err);
    }

    if (NULL == conn)
    {
        fprintf(stderr, "Connection Null\n");
        exit(1);
    }

    // request our name on the bus and check for errors
    // we are requesting dbus name two time one in dbus_init function
    dbus_bus_request_name(conn,"ndx.nordic.Bus", DBUS_NAME_FLAG_REPLACE_EXISTING , &err);

    if (dbus_error_is_set(&err))
    {
        fprintf(stderr, "Name Error (%s)\n", err.message);
        dbus_error_free(&err);
    }

    // loop, testing for new messages
    while (!done)
    {
        NDX_EXIT_LT_0(ndx_dbus_pump());
        if (ret == 1)
            continue;

        NDX_EXIT_NE_0(usleep(LOOPSLEEP));

        // non blocking read of the next available message
        dbus_connection_read_write(conn, 0);
        msg = dbus_connection_pop_message(conn);

        // loop again if we haven't got a message
        if (NULL == msg)
        {
            //check for UART3 DATA
            rd_length = read(fd, rxData, sizeof(rxData));

            if (rd_length > 0)
            {
                memcpy(temp_buf, rxData, rd_length);
                temp_buf[rd_length] = 0;
                printf("Recieve data from nRF %s", temp_buf);
                temprature_logging(temp_buf, log_file, &ltime, &mytime);
                if(heater_flag == 0x01)
                {
                    if(strstr(temp_buf, heater_data))
                    {
                        heater_flag = 0;
                        printf("Heater Response Recieved!\n");
                        sprintf(heater_data, "%08i", heater_sequence);
                        ndx_dbus_emit_signal_str(DBUS_NORDIC_OBJECT, DBUS_NORDIC_TYPE, "nordic_response", heater_data);
                    }
                }
                if(wheel_flag == 0x01)
                {
                    if(strstr(temp_buf, wheel_data))
                    {
                        wheel_flag = 0;
                        printf("Wheel Response Recieved!\n");
                        sprintf(wheel_data, "%08i", wheel_sequence);
                        ndx_dbus_emit_signal_str(DBUS_NORDIC_OBJECT, DBUS_NORDIC_TYPE, "nordic_response", wheel_data);
                    }
                }
                fflush(stdout);
            }
            continue;
        }
        else
        {
            if ( dbus_message_has_interface(msg, "ndx.nordic.Type") )
                {
                    reply_to_dbus_method_call( msg, conn );
                }
            //check for UART3 DATA
            rd_length = read(fd, rxData, sizeof(rxData));

            if (rd_length > 0)
            {
                memcpy(temp_buf, rxData, rd_length);
                temp_buf[rd_length] = 0;
                printf("Recieve data from nRF %s", temp_buf);
                temprature_logging(temp_buf, log_file, &ltime, &mytime);
                if(heater_flag == 0x01)
                {
                    if(strstr(temp_buf, heater_data))
                    {
                        heater_flag = 0;
                        printf("Heater Response Recieved!\n");
                        sprintf(heater_data, "%08i", heater_sequence);
                        ndx_dbus_emit_signal_str(DBUS_NORDIC_OBJECT, DBUS_NORDIC_TYPE, "nordic_response", heater_data);
                    }
                }
                if(wheel_flag == 0x01)
                {
                    if(strstr(temp_buf, wheel_data))
                    {
                        wheel_flag = 0;
                        printf("Wheel Response Recieved!\n");
                        sprintf(wheel_data, "%08i", wheel_sequence);
                        ndx_dbus_emit_signal_str(DBUS_NORDIC_OBJECT, DBUS_NORDIC_TYPE, "nordic_response", wheel_data);
                    }
                }
                fflush(stdout);
            }

            // free the message
            dbus_message_unref(msg);
        }
    }
    // close the connection
    //   dbus_connection_close(conn);
    Exit:
        syslog(LOG_INFO, "Terminating nordic service %d", done);
}

static int nordic_init()
{
    int ret = 0;
    NDX_EXIT_NE_0(ndx_dbus_init("ndx.nordic.Bus", 0));
    NDX_EXIT_NE_0(uart_init());

Exit:
    return ret;
}


static int uart_init(void)
{
    int ret = UART_ERR_DEFAULT;

    fd = open(device, O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
    if (fd < 0)
    {
        printf("Error: uart device open = %i\n", fd);
        ret = UART_ERR_FILE;
        goto error;
    }
    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    set_interface_attribs(fd, B115200);

    isUARTInit = 1;
    return UART_ERR_NONE;

error:
    return ret;
}

//static int nordic_deinit(void)
//{
//    int ret = UART_ERR_DEFAULT;
//
//    ret = close(fd); // close file as binary
//    if (fd < 0)
//    {
//        printf("Error: uart device close = 0\n");
//        ret = UART_ERR_FILE;
//        goto error;
//    }
//    isUARTInit = 0;
//    return UART_ERR_NONE;
//
//error:
//    return ret;
//}

static int nordic_Tx(const unsigned char *txData, int length)
{

    int ret = 0;
    int temp_len;

    /* check if SPI init is done */
    if (!isUARTInit)
    {
        printf(" error: uart is used for communication before its initialization\n");
        return ret;
    }
    /* simple output */

    temp_len = write(fd, txData, length);
    if (temp_len != length) {
        printf("Error: UART error in data transfer = %d\n", ret);
        return ret;
    }
    printf("Transfer Data value = %s \n", txData);
    tcdrain(fd);    /* delay for output */

    return 1;
}

//static int nordic_Rx(unsigned char *rxData, int length)
//{
//    int ret = 0;
//    int rd_length = 0;
//
//    /* check if SPI init is done */
//    if (!isUARTInit)
//    {
//        printf(" error: uart is used for communication before its initialization\n");
//        return UART_ERR_IO;
//    }
//
//    while(length > 0)
//    {
//        rd_length = read(fd, rxData, length); // read some number of bytes to uart
//        if(rd_length > 0)
//        {
//            length -= rd_length;
//            rxData += rd_length;
//        }
//    }
//
//    return ret;
//}

static void reply_to_dbus_method_call(DBusMessage* msg, DBusConnection* conn)
{
    // does these messages should be created on main/global, so does not create everytime we call this fucntion.
    DBusMessage* reply;
    DBusMessageIter rootIter;

    uint8_t       Device;
    uint16_t      Value;
    uint32_t      SequenceNumber;
    uint32_t      Timeout;
    uint32_t      Status;

    uint8_t       cmd_err = NORDIC_COMMAND_OK;
    uint8_t       length = 0;

    unsigned char  tx_data[10];
    unsigned char  rx_data[50];

    // read the arguments
    if (!dbus_message_iter_init(msg,&rootIter))
    {
        fprintf(stderr, "Message has no arguments!\n");
    }

    dbus_message_iter_get_basic(&rootIter, &Device);

    if(dbus_message_iter_has_next(&rootIter))
    {
        dbus_message_iter_next(&rootIter);
        dbus_message_iter_get_basic(&rootIter, &Value);
    }

    if(dbus_message_iter_has_next(&rootIter))
    {
        dbus_message_iter_next(&rootIter);
        dbus_message_iter_get_basic(&rootIter, &SequenceNumber);
    }

    if(dbus_message_iter_has_next(&rootIter))
    {
        dbus_message_iter_next(&rootIter);
        dbus_message_iter_get_basic(&rootIter, &Timeout);
    }


    syslog(LOG_INFO, "Received massage from test-runner with Device name = %i, Device Value = %i, Sequence Number = %i, Timeout = %i\n",
        Device, Value, SequenceNumber, Timeout);
    printf("Received massage from test-runner with Device name = %i, Device Value = %i, Sequence Number = %i, Timeout = %i\n",
        Device, Value, SequenceNumber, Timeout);

    if (dbus_message_is_method_call( msg, "ndx.nordic.Type", "Nordic_Command"))
    {
        tx_data[0] = TX_SYNC_CHAR;

        switch (Device)
        {
            case NORDIC_WHEEL_MOTOR:
                tx_data[1] = 'W';
                wheel_sequence = SequenceNumber;
                wheel_flag = 1;
                break;

            case NORDIC_HEATER:
                tx_data[1] = 'H';
                heater_sequence = SequenceNumber;
                heater_flag = 1;
                break;

            case NORDIC_LYSIS_PUMP:
                tx_data[1] = 'P';
                break;

            case NORDIC_UV_A_LED:
                tx_data[1] = 'U';
                break;

            case NORDIC_VIS_LED:
                tx_data[1] = 'C';
                break;

            case NORDIC_TEMP_STATUS:
                tx_data[1] = 'T';
                break;

            case NORDIC_DEV_TEST:
                test_start = 1;
                tx_data[1] = 'T';
                printf("Nordic task started\n");
                break;

            case NORDIC_LAST_DEVICE:
                test_start = 0;
                tx_data[1] = 'T';
                printf("Nordic task ended\n");
                break;

            default:
                cmd_err = NORDIC_COMMAND_ERROR;
                break;
        }

        if(tx_data[1] != 0)
        {
            sprintf((char*)&tx_data[2], "%04d", Value);
            tx_data[6] = 0;
//            printf("%s\n", tx_data);
        }

        fflush(stdout);

        if (cmd_err == NORDIC_COMMAND_OK)
        {

            if(Device == NORDIC_WHEEL_MOTOR)
            {
                memcpy(&wheel_data[1], &tx_data[1], 5);
                wheel_data[0] = '<';
                wheel_data[6] = '+';
                wheel_data[7] = 0x00;
            }

            if(Device == NORDIC_HEATER)
            {
                memcpy(&heater_data[1], &tx_data[1],5);
                heater_data[0] = '<';
                heater_data[6] = '+';
                heater_data[7] = 0x00;
            }

            nordic_Tx(tx_data, 6);          //Send to Nordic nRF52840
            // here length need to be according to re_data like sizeof(rx_data) not 4.
            length = read(fd, rx_data, 4);  // read response of nRF52840
            syslog(LOG_INFO, "Received data from nRF = %s", rx_data);
            printf("Received data from nRF = %s", rx_data);
            fflush(stdout);

            if((rx_data[1] == tx_data[1]) && (length == 4)) // length need to be sizeof(rx_data)
            {
                Status = NORDIC_COMMAND_ACK;
            }
            else
            {
                Status = NORDIC_COMMAND_NACK;
            }
        }
        else
        {
            Status = NORDIC_COMMAND_NACK;
        }
    }

    if((Device == NORDIC_WHEEL_MOTOR) || (Device == NORDIC_HEATER))
    {
        Status = NORDIC_COMMAND_DELAYED;
    }

    // create a reply from the message
    reply = dbus_message_new_method_return(msg);

    // add the arguments to the reply
    dbus_message_iter_init_append(reply, &rootIter);


    Status = Status << 16;
    Status += SequenceNumber;

    if (!dbus_message_iter_append_basic(&rootIter, DBUS_TYPE_UINT32, &Status))
    {
        fprintf(stderr, "Out Of Memory!\n");
        exit(1);
    }
    // send the reply && flush the connection
    if (!dbus_connection_send(conn, reply, &Status))
    {
        fprintf(stderr, "Out Of Memory!\n");
        exit(1);
    }

    dbus_connection_flush(conn);

    // free the reply
    dbus_message_unref(reply);

    // do we have to close connection.
}

static int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

//     setup for non-canonical mode
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    tty.c_oflag &= ~OPOST;


//     fetch bytes as they become available
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    cfmakeraw(&tty);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

static void signal_handler(int signal)
{
    done = signal;
}

int temprature_logging(char* temp_buf, FILE* log_file, time_t* ltime, struct tm* mytime)
{
    int ret = 0;
    char buff[50];
//    char temprature[60];

    time(ltime);
//    printf("Cordinate time is %s\n", asctime_r(gmtime_r(ltime, mytime), buff));

    log_file = fopen("/storage/data/temprature.log", "a+");

    if(log_file == NULL)
    {
        printf(" Not able to log the temprature file");
        ret = -1;
    }

    if(temp_buf[1] == 'T')
    {
//        sprintf(temprature, "%s --> %s", buff, temp_buf);
        fprintf(log_file, "%s%s", buff, temp_buf);
    }

    return ret;
}
