/*
 * An example SMR program.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include <sys/ioctl.h>
#include "rhd.h"
#include "componentserver.h"
#include "xmlio.h"

struct xml_in *xmldata;
struct xml_in *xmllaser;
struct
{
  double x, y, z, omega, phi, kappa, code, id, crc;
} gmk;
double visionpar[10];
double laserpar[10];
double angular_v;

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);

componentservertype lmssrv, camsrv;

symTableElement *
getinputref(const char *sym_name, symTableElement *tab)
{
  int i;
  for (i = 0; i < getSymbolTableSize('r'); i++)
    if (strcmp(tab[i].name, sym_name) == 0)
      return &tab[i];
  return 0;
}

symTableElement *
getoutputref(const char *sym_name, symTableElement *tab)
{
  int i;
  for (i = 0; i < getSymbolTableSize('w'); i++)
    if (strcmp(tab[i].name, sym_name) == 0)
      return &tab[i];
  return 0;
}

#define LOG_LENGTH 100
#define NOT_LAST_OUTPUT 0
#define LAST_OUTPUT 1
#define CROSSING_BLACK_LINE 0 // Mode 0 = crossingblackline
#define END_OF_BLACK_LINE 1
#define IR_DISTANCE 2 // Mode 2 = IR distance
#define IR_DISTANCE_LOST 9
#define LASER_DISTANCE 3 // Mode 3 = Laser distance
#define ODO_DISTANCE 4   // Mode 4 = odo distance
#define CROSSING_WHITE_LINE 5
#define END_OF_WHITE_LINE 6
#define BLACK 7
#define WHITE 8
// IR sensors
#define IR_LEFT 0
#define IR_FORWARD_LEFT 1
#define IR_FORWARD_MIDDLE 2
#define IR_FORMARD_RIGHT 3
#define IR_RIGHT 4
#define IR_NONE 5
// Laser scanner
#define LASER_LEFT 0
#define LASER_FRONT_m1 3
#define LASER_FRONT 4
#define LASER_FRONT_p1 5
#define LASER_RIGHT 8
// followline: right = 3.0, middle = 3.5, left = 4.0
#define FOLLOW_RIGHT 3.0f
#define FOLLOW_MIDDLE 3.5f
#define FOLLOW_LEFT 4.0f

#define T1 -0.371580802823595f
#define T2 1.40527367822561f
#define T3 0.0314219095354553f
#define T4 0.992371206299907f

double odoCoords[2], mobotCoords[2], coordDist = 0, coordAngle = 0;

typedef struct
{
  unsigned int log_missionTime[LOG_LENGTH];
  double log_speedL[LOG_LENGTH];
  double log_speedR[LOG_LENGTH];
  unsigned int log_currentDataEntry;
} logType;

logType log_data;

void logging(unsigned int time, double motorspeed_l, double motorspeed_r, int lastOutput)
{
  FILE *fp;
  fp = fopen("data.log", "a");

  if (!lastOutput)
  {
    log_data.log_missionTime[log_data.log_currentDataEntry] = time;
    log_data.log_speedL[log_data.log_currentDataEntry] = motorspeed_l;
    log_data.log_speedR[log_data.log_currentDataEntry++] = motorspeed_r;
    // printf("Log entry no: %d, time: %d\n", log_data.log_currentDataEntry, time);

    if (log_data.log_currentDataEntry == LOG_LENGTH)
    {
      log_data.log_currentDataEntry = 0;

      for (int i = 0; i < LOG_LENGTH; i++)
      {

        fprintf(fp, "%d, %0.3f, %0.3f,\n", log_data.log_missionTime[i], log_data.log_speedL[i], log_data.log_speedR[i]);
      }
    }
  }
  else if (lastOutput)
  {
    for (int i = 0; i < log_data.log_currentDataEntry; i++)
    {

      fprintf(fp, "%d, %0.3f, %0.3f,\n", log_data.log_missionTime[i], log_data.log_speedL[i], log_data.log_speedR[i]);
    }
  }

  fclose(fp);
}

typedef struct
{
  double log_theta[LOG_LENGTH];
  double log_x[LOG_LENGTH];
  double log_y[LOG_LENGTH];
  unsigned int log_currentDataEntry;
} odoLogType;

odoLogType odo_log_data;

void odoLogging(double theta, double x, double y, int lastOutput)
{
  FILE *fp;
  fp = fopen("odoData.log", "a");

  if (!lastOutput)
  {
    odo_log_data.log_theta[odo_log_data.log_currentDataEntry] = theta;
    odo_log_data.log_x[odo_log_data.log_currentDataEntry] = x;
    odo_log_data.log_y[odo_log_data.log_currentDataEntry++] = y;
    // printf("Log entry no: %d, time: %d\n", log_data.log_currentDataEntry, time);

    if (odo_log_data.log_currentDataEntry == LOG_LENGTH)
    {
      odo_log_data.log_currentDataEntry = 0;

      for (int i = 0; i < LOG_LENGTH; i++)
      {

        fprintf(fp, "%0.3f, %0.3f, %0.3f,\n", odo_log_data.log_theta[i], odo_log_data.log_x[i], odo_log_data.log_y[i]);
      }
    }
  }
  else if (lastOutput)
  {
    for (int i = 0; i < odo_log_data.log_currentDataEntry; i++)
    {

      fprintf(fp, "%0.3f, %0.3f, %0.3f,\n", odo_log_data.log_theta[i], odo_log_data.log_x[i], odo_log_data.log_y[i]);
    }
  }

  fclose(fp);
}

// for some reason there are 10 values in laserpar. The last one is always 0.
// and the laser only reads the distance in 9 zones.
#define NO_LASERS 9
typedef struct
{
  double log_laser[LOG_LENGTH][NO_LASERS];
  unsigned int log_currentDataEntry;
} laserLogType;

laserLogType laser_log_data;

void laserLogging(double laserReading[], int lastOutput)
{
  FILE *fp;
  fp = fopen("laserData.log", "a");

  if (!lastOutput)
  {
    for (int i = 0; i < NO_LASERS; i++)
    {
      laser_log_data.log_laser[laser_log_data.log_currentDataEntry][i] = laserReading[i];
    }
    laser_log_data.log_currentDataEntry++;

    if (laser_log_data.log_currentDataEntry == LOG_LENGTH)
    {
      laser_log_data.log_currentDataEntry = 0;

      for (int i = 0; i < LOG_LENGTH; i++)
      {
        for (int j = 0; j < NO_LASERS; j++)
          fprintf(fp, "%0.3f, ", laser_log_data.log_laser[i][j]);

        fprintf(fp, "\n");
      }
    }
  }
  else if (lastOutput)
  {
    for (int i = 0; i < laser_log_data.log_currentDataEntry; i++)
    {
      for (int j = 0; j < NO_LASERS; j++)
        fprintf(fp, "%0.3f, ", laser_log_data.log_laser[i][j]);

      fprintf(fp, "\n");
    }
  }

  fclose(fp);
}

void resetLogging(void)
{
  FILE *fp;
  log_data.log_currentDataEntry = 0;
  odo_log_data.log_currentDataEntry = 0;
  laser_log_data.log_currentDataEntry = 0;
  fp = fopen("data.log", "w");
  fprintf(fp, "\r");
  fclose(fp);
  fp = fopen("odoData.log", "w");
  fprintf(fp, "\r");
  fclose(fp);
  fp = fopen("laserData.log", "w");
  fprintf(fp, "\r");
  fclose(fp);
  fp = fopen("lineSensorCalibData.log", "w");
  fprintf(fp, "\r");
  fclose(fp);
}

/*****************************************
 * odometry
 */
#define WHEEL_DIAMETER 0.06522 /* m */
#define WHEEL_SEPARATION 0.26  /* m */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define ROBOTPORT 24902

typedef struct
{                          // input signals
  int left_enc, right_enc; // encoderticks
  // parameters
  double w;      // wheel separation
  double cr, cl; // meters per encodertick
                 // output signals
  double right_pos, left_pos;
  // internal variables
  int left_enc_old, right_enc_old;

  double x;
  double y;
  double theta;

} odotype;

void reset_odo(odotype *p);
void update_odo(odotype *p);

/********************************************
 * Motion control
 */

typedef struct
{ // input
  int cmd;
  int curcmd;
  int mode;
  int IR_mode;
  int IR_mode_stop;
  int colour;
  double IR_dist_stop;
  double line;
  double speedcmd;
  double dist;
  double angle;
  double radius;
  double ref_angle;
  double left_pos, right_pos;
  // parameters
  double w;
  // output
  double motorspeed_l, motorspeed_r;
  int finished;
  // internal variables
  double startpos;
} motiontype;

enum
{
  mot_stop = 1,
  mot_direction_control,
  mot_followline,
  mot_followwall,
  mot_move,
  mot_turn,
  mot_turnr
};

void update_motcon(motiontype *p);

void coord_odo2mobot(double x, double y);
void coord_mobot2odo(double x, double y);
double getAngleTo(double dest_x, double dest_y);
double getDistanceTo(double dest_x, double dest_y);
int followWall(double dist, double speed, int time, int IR_mode, int IR_mode_stop, double IR_dist_stop);
int followline(double line, double dist, double speed, int time, int mode, int IR_mode, int colour);
int direction_control(double dist, double speed, double ref_angle, int time);
int fwd(double dist, double speed, int time, int mode, int IR_mode);
int turn(double angle, double speed, int time);
int turnr(double angle, double radius, double speed, int time);
double deg2rad(double degree);

typedef struct
{
  int state, oldstate;
  int time;
} smtype;

void sm_update(smtype *p);

// SMR input/output data

symTableElement *inputtable, *outputtable;
symTableElement *lenc, *renc, *linesensor, *irsensor, *speedl, *speedr, *resetmotorr, *resetmotorl;

odotype odo;
smtype mission;
motiontype mot;

enum
{
  ms_init,
  ms_fwd,
  ms_turn,
  ms_end,
  ms_direction_control,
  ms_followline_middle,
  ms_followline_left,
  ms_followline_right,
  ms_measureLine
};

// prototypes for functions at the bottom of script
void lineCalibLogging(int time);
int lineSensorCalibrated(int rawValue);
int lineMinInt(int input[]);
double IRcalibrated(int rawValue);

int main()
{
  // reset the files to prepare it for use.
  resetLogging();

  int running, n = 0, arg, time = 0;
  double dist = 0, angle = 0, speed = 0, ref_angle = 0;

  /* Establish connection to robot sensors and actuators.
   */
  if (rhdConnect('w', "localhost", ROBOTPORT) != 'w')
  {
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE);
  }

  printf("connected to robot \n");
  if ((inputtable = getSymbolTable('r')) == NULL)
  {
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE);
  }
  if ((outputtable = getSymbolTable('w')) == NULL)
  {
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE);
  }
  // connect to robot I/O variables
  lenc = getinputref("encl", inputtable);
  renc = getinputref("encr", inputtable);
  linesensor = getinputref("linesensor", inputtable);
  irsensor = getinputref("irsensor", inputtable);

  speedl = getoutputref("speedl", outputtable);
  speedr = getoutputref("speedr", outputtable);
  resetmotorr = getoutputref("resetmotorr", outputtable);
  resetmotorl = getoutputref("resetmotorl", outputtable);
  // **************************************************
  //  Camera server code initialization
  //

  /* Create endpoint */
  lmssrv.port = 24919;
  strcpy(lmssrv.host, "127.0.0.1");
  strcpy(lmssrv.name, "laserserver");
  lmssrv.status = 1;
  camsrv.port = 24920;
  strcpy(camsrv.host, "127.0.0.1");
  camsrv.config = 1;
  strcpy(camsrv.name, "cameraserver");
  camsrv.status = 1;

  if (camsrv.config)
  {
    int errno = 0;
    camsrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (camsrv.sockfd < 0)
    {
      perror(strerror(errno));
      fprintf(stderr, " Can not make  socket\n");
      exit(errno);
    }

    serverconnect(&camsrv);

    xmldata = xml_in_init(4096, 32);
    printf(" camera server xml initialized \n");
  }

  // **************************************************
  //  LMS server code initialization
  //

  /* Create endpoint */
  lmssrv.config = 1;
  if (lmssrv.config)
  {
    char buf[256];
    int errno = 0, len;
    lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (lmssrv.sockfd < 0)
    {
      perror(strerror(errno));
      fprintf(stderr, " Can not make  socket\n");
      exit(errno);
    }

    serverconnect(&lmssrv);
    if (lmssrv.connected)
    {
      xmllaser = xml_in_init(4096, 32);
      printf(" laserserver xml initialized \n");
      len = sprintf(buf, "scanpush cmd='zoneobst'\n");
      send(lmssrv.sockfd, buf, len, 0);
    }
  }

  /* Read sensors and zero our position.
   */
  rhdSync();

  odo.w = 0.256;
  odo.cr = DELTA_M;
  odo.cl = odo.cr;
  odo.left_enc = lenc->data[0];
  odo.right_enc = renc->data[0];
  reset_odo(&odo);
  printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
  mot.w = odo.w;
  running = 1;
  mission.state = ms_init;
  mission.oldstate = -1;
  while (running)
  {
    if (lmssrv.config && lmssrv.status && lmssrv.connected)
    {
      while ((xml_in_fd(xmllaser, lmssrv.sockfd) > 0))
        xml_proca(xmllaser);
    }

    if (camsrv.config && camsrv.status && camsrv.connected)
    {
      while ((xml_in_fd(xmldata, camsrv.sockfd) > 0))
        xml_proc(xmldata);
    }

    rhdSync();
    odo.left_enc = lenc->data[0];
    odo.right_enc = renc->data[0];
    update_odo(&odo);

    /****************************************
    / mission statemachine
    */

    // our homemade logging function
    logging(mission.time, mot.motorspeed_l, mot.motorspeed_r, NOT_LAST_OUTPUT);
    laserLogging(laserpar, NOT_LAST_OUTPUT);

    
    // prints the linesensor values for
    for(int i = 0; i<8; i++)
      printf("%d ", linesensor->data[i]);

    printf("  <-r|c->  ");

    for(int i = 0; i<8; i++)
      printf("%d ", lineSensorCalibrated(linesensor->data[i]));

    printf("\n");
    

    sm_update(&mission);

    switch (mission.state)
    {
    case ms_init:
      n = 2;
      dist = 2;
      speed = 0.15;
      angle = deg2rad(90);
      mission.state = 41;
      printf("Entering Case %d\n", mission.state);
      break;

      /* case 101:
         if (turn(deg2rad(90), speed, mission.time))
         {
           ++mission.state;
         }
         break;
       case 102:
         if (fwd(1, speed, mission.time, ODO_DISTANCE, IR_NONE))
           mission.state = ms_end;
         break; */

      /*case 101:
        coord_mobot2odo(1, 0);
        //coord_odo2mobot(0, 1);
        coordDist = getDistanceTo(odoCoords[0], odoCoords[1]);
        coordAngle = getAngleTo(odoCoords[0], odoCoords[1]);
        printf("CoordDist: %0.3f\n", coordDist);
        printf("CoordAngle: %0.3f\n", coordAngle);
        printf("Going to: %0.3f, %0.3f\n", odoCoords[0], odoCoords[1]);
        printf("MoboCoords: %0.3f, %0.3f\n", mobotCoords[0], mobotCoords[1]);
        mission.state++; // no break, begin turning in same cycle

      case 102:
        if (turn(coordAngle, 0.3, mission.time))
          mission.state++;
        break;

      case 103:
        // if(direction_control(coordAngle, coordDist, speed, mission.time))
        if (fwd(coordDist, speed, mission.time, ODO_DISTANCE, IR_NONE))
          mission.state = ms_end;
        break;*/

    case 9:
      // printf("LASER_RIGHT: %0.3f\n", laserpar[LASER_RIGHT]);
      if (followline(FOLLOW_MIDDLE, 2.3, speed, mission.time, LASER_DISTANCE, LASER_RIGHT, BLACK))
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 10:
      if (fwd(0.8, speed, mission.time, ODO_DISTANCE, IR_NONE))
        printf("Entering Case %d\n", ++mission.state);
      break;

    case 11:
      if (turn(deg2rad(305), speed, mission.time))
        printf("Entering Case %d\n", ++mission.state);
      break;

    case 12:
      if (fwd(0.5, speed, mission.time, ODO_DISTANCE, IR_NONE))
        printf("Entering Case %d\n", ++mission.state);
      break;

    case 13:
      if (followline(FOLLOW_MIDDLE, 1, speed, mission.time, ODO_DISTANCE, IR_NONE, BLACK))
        printf("Entering Case %d\n", ++mission.state);
      break;

    case 14:
      if (followline(FOLLOW_MIDDLE, 1, speed, mission.time, CROSSING_BLACK_LINE, IR_NONE, BLACK)) // Pushing box in Gate 1
        printf("Entering Case %d\n", ++mission.state);
      break;

    case 15:
      if (fwd(0.15, speed, mission.time, ODO_DISTANCE, IR_NONE)) // Pushing box in Gate 1
      {
        // mission.state = 16;
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 16:
      if (fwd(1, -speed, mission.time, ODO_DISTANCE, IR_NONE)) // Pushing box in Gate 1
        printf("Entering Case %d\n", ++mission.state);
      break;

    case 17:
      if (turn(deg2rad(270), speed, mission.time)) // Pushing box in Gate 1
      {
        mission.state = 115;
        printf("Entering Case %d\n", mission.state);
      }
      break;

    case 115:
      if (fwd(0.2, speed, mission.time, ODO_DISTANCE, IR_NONE))
      {
        mission.state = 18;
        printf("Entering Case %d\n", mission.state);
      }

    case 18:
      if (fwd(1, speed, mission.time, CROSSING_BLACK_LINE, IR_NONE))
        printf("Entering Case %d\n", ++mission.state);
      break;

    case 19:
      if (fwd(0.2, speed, mission.time, ODO_DISTANCE, IR_NONE))
        printf("Entering Case %d\n", ++mission.state);
      break;

    case 20:
      if (turn(deg2rad(90), speed, mission.time))
        printf("Entering Case %d\n", ++mission.state);
      break;

    case 21:
      if (followline(FOLLOW_MIDDLE, 1, speed, mission.time, CROSSING_BLACK_LINE, IR_NONE, BLACK))
        printf("Entering Case %d\n", ++mission.state);
      break;

    case 22:
      if (fwd(0.2, speed, mission.time, ODO_DISTANCE, IR_NONE))
        printf("Entering Case %d\n", ++mission.state);
      break;

    case 23:
      if (turn(deg2rad(90), speed, mission.time)) // beginning to turn and exit Gate 1
        printf("Entering Case %d\n", ++mission.state);
      break;

    case 24:
      if (fwd(1.2, speed, mission.time, ODO_DISTANCE, IR_NONE))
        printf("Entering Case %d\n", ++mission.state);
      break;

    case 25:
      if (turn(deg2rad(90), speed, mission.time))
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 26:
      if (fwd(0.3, speed, mission.time, IR_DISTANCE, IR_RIGHT))
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 27:
      if (fwd(0.45, speed, mission.time, ODO_DISTANCE, IR_NONE))
      {
        mission.state = 29;
        printf("Entering Case %d\n", mission.state);
      }
      break;

      /*case 28:
        if (followline(FOLLOW_MIDDLE, 0.45, speed, mission.time, ODO_DISTANCE, IR_NONE, BLACK))
        {
          printf("Entering Case %d\n", ++mission.state);
        }
        break;*/

    case 29:
      if (turn(deg2rad(-90), speed, mission.time))
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 30:
      if (fwd(0.3, speed, mission.time, IR_DISTANCE, IR_RIGHT))
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 31:
      if (fwd(0.45, speed, mission.time, ODO_DISTANCE, IR_NONE))
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 32:
      if (turn(deg2rad(-90), speed, mission.time)) // turn towards south wall gate
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 33:
      if (fwd(1, speed, mission.time, ODO_DISTANCE, IR_NONE)) // Drive trough south wall gate
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 34:
      if (turn(deg2rad(90), speed, mission.time)) // Turn north
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 35:
      if (fwd(0.8, speed, mission.time, IR_DISTANCE, IR_RIGHT)) // Drive north along the wall until a gate is visible on the right
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 36:
      if (fwd(0.45, speed, mission.time, ODO_DISTANCE, IR_NONE))
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 37:
      if (turn(deg2rad(-90), speed, mission.time)) // Turn towards floating gate
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 38:
      if (fwd(1, speed, mission.time, CROSSING_BLACK_LINE, IR_NONE)) // drive through the gate
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 39:
      if (fwd(0.2, speed, mission.time, ODO_DISTANCE, IR_NONE)) // drive through the gate
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 40:
      if (turn(deg2rad(90), speed, mission.time)) // turn ccw to be parallel with the black line
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 41:
      if (followline(FOLLOW_MIDDLE, 0.45, speed, mission.time, END_OF_BLACK_LINE, IR_NONE, BLACK)) // follow line
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 42:
      if (followline(FOLLOW_MIDDLE, 0.45, speed, mission.time, END_OF_WHITE_LINE, IR_NONE, WHITE)) // follow white line
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 43:
      if (fwd(0.2, speed, mission.time, ODO_DISTANCE, IR_NONE)) // go a bit forwards
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 44:
      if (turn(deg2rad(-90), speed, mission.time)) // turn towards last gate with door
      {
        printf("Entering Case %d\n", mission.state = 38);
      }
      break;

      /*case 101:
        coord_mobot2odo(3, 2.3);
        coordDist = getDistanceTo(odoCoords[0], odoCoords[1]);
        coordAngle = getAngleTo(odoCoords[0], odoCoords[1]);
        printf("Going to: %0.3f, %0.3f\n", odoCoords[0], odoCoords[1]);
        printf("MoboCoords: %0.3f, %0.3f\n", mobotCoords[0], mobotCoords[1]);
        printf("Dist: %0.3f, angle: %0.3f\n", coordDist, coordAngle);
        mission.state++; // no break, begin turning in same cycle
        break;

      case 102:
        if (turn(coordAngle, 0.3, mission.time))
          printf("Entering Case %d\n", ++mission.state);
        break;

      case 103:
        // if(direction_control(coordAngle, coordDist, speed, mission.time))
        if (fwd(coordDist, speed, mission.time, ODO_DISTANCE, IR_NONE))
        {
          printf("Entering Case %d\n", ++mission.state);
        }
        break;

      case 104:
        if (turn(deg2rad(60), 0.2, mission.time))
        {
          mission.state = 38;
          printf("Entering Case %d\n", mission.state);
        }
        break;*/

    case 45:
      if (followline(FOLLOW_MIDDLE, 0.45, speed, mission.time, CROSSING_BLACK_LINE, IR_NONE, BLACK)) // follow the black line towards gate
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 46:
      if (turn(deg2rad(-90), speed, mission.time)) // turn right
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 47:
      if (fwd(0.8, speed, mission.time, IR_DISTANCE_LOST, IR_LEFT)) // go forwards until left IR sensor does not see box
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;
    case 48:
      if (fwd(0.4, speed, mission.time, ODO_DISTANCE, IR_NONE)) // go a bit forwards
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 49:
      if (turn(deg2rad(90), speed, mission.time)) // turn left
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;
    case 50:
      if (fwd(0.45, speed, mission.time, ODO_DISTANCE, IR_NONE)) // go forwards to see box
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 51:
      if (fwd(0.8, speed, mission.time, IR_DISTANCE_LOST, IR_LEFT)) // follow box
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 52:
      if (fwd(0.4, speed, mission.time, ODO_DISTANCE, IR_NONE)) // forwards a bit
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 53:
      if (turn(deg2rad(90), speed, mission.time)) // turn left
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 54:
      if (fwd(0.45, speed, mission.time, ODO_DISTANCE, IR_NONE)) // forwards a bit
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 55:
      if (fwd(0.8, speed, mission.time, IR_DISTANCE_LOST, IR_LEFT)) // follow box
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 56:
      if (fwd(0.4, speed, mission.time, ODO_DISTANCE, IR_NONE)) // forwards a bit
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 57:
      if (turn(deg2rad(90), speed, mission.time)) // turn left
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 58:
      if (fwd(0.8, speed, mission.time, ODO_DISTANCE, IR_NONE)) // forwards a bit
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 59:
      if (turnr(deg2rad(90), 1, speed, mission.time)) // forwards a bit
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 60:
      if (turn(deg2rad(90), speed, mission.time)) // turn left
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 61:
      if (fwd(0.2, speed, mission.time, ODO_DISTANCE, IR_NONE)) // forwards a bit
      {
        printf("Entering Case %d\n", ++mission.state);
      }
      break;

    case 62:
      if (followline(FOLLOW_MIDDLE, 0.1, speed, mission.time, LASER_DISTANCE, LASER_FRONT, BLACK)) // INTO THE BOX!!
      {
        mission.state = ms_end;
        printf("Entering Case %d\n", mission.state);
      }
      break;

      /*case 59:
        if (followline(FOLLOW_MIDDLE, 0.2, speed, mission.time, IR_DISTANCE, IR_FORWARD_MIDDLE, BLACK)) // INTO THE BOX!!
        {
          mission.state = ms_end;
          printf("Entering Case %d\n", mission.state);
        }
        break;
      */
      /*
      case 18:
        if (followline(FOLLOW_MIDDLE, 0.6, 0.2, mission.time, CROSSING_BLACK_LINE, IR_NONE))
        {
          printf("Entering Case %d\n", ++mission.state);
        }
        break;

      case 19:
        if (fwd(0.1, 0.2, mission.time, ODO_DISTANCE, IR_NONE))
        {
          printf("Entering Case %d\n", ++mission.state);
        }
        break;

      case 20:
        if (followline(FOLLOW_MIDDLE, 0.6, 0.2, mission.time, IR_DISTANCE, IR_LEFT)) // finds first gate opening on left hand
        {
          printf("Entering Case %d\n", ++mission.state);
        }
        break;

      case 21:
        if (followline(FOLLOW_MIDDLE, 0.45, 0.2, mission.time, ODO_DISTANCE, IR_NONE))
        {
          printf("Entering Case %d\n", ++mission.state);
        }
        break;

      case 22:
        if (turn(deg2rad(90), 0.2, mission.time))
        {
          printf("Entering Case %d\n", ++mission.state);
        }
        break;

      case 23:
        if (fwd(1, 0.2, mission.time, ODO_DISTANCE, IR_NONE))
        {
          printf("Entering Case %d\n", ++mission.state);
        }
        break;

      case 24:
        if (turn(deg2rad(90), 0.2, mission.time))
        {
          mission.state = ms_end;
        }
        break;

        */

      /*
          case ms_measureLine:
            if (n == 0)
            {
              lineCalibLogging(mission.time);
              mission.state = ms_end;
            }
            else
            {
              lineCalibLogging(mission.time);
              mission.state = ms_fwd;
            }
            n--;
            break;

          case ms_fwd:
            if (fwd(0.3, 0.1, mission.time))
              mission.state = ms_measureLine;
            break;
      */
      /*
      case ms_fwd:
        if(fwd(2, 0.4, mission.time))
          mission.state = ms_end;
      break;
      */

      /*
           case ms_fwd:
             if (fwd(dist,10,mission.time))  mission.state=ms_turn;
           break;

           case ms_turn:
             if (turn(angle, 0.2, mission.time)){
               n=n-1;
         if (n==0)
           mission.state=ms_end;
         else
           mission.state=ms_fwd;
        }
           break;
      */

    case ms_end:
      mot.cmd = mot_stop;
      running = 0;
      break;
    }
    /*  end of mission  */

    mot.left_pos = odo.left_pos;
    mot.right_pos = odo.right_pos;
    update_motcon(&mot);
    speedl->data[0] = 100 * mot.motorspeed_l;
    speedl->updated = 1;
    speedr->data[0] = 100 * mot.motorspeed_r;
    speedr->updated = 1;
    if (time % 100 == 0)
      //    printf(" laser %f \n",laserpar[3]);
      time++;
    /* stop if keyboard is activated
     *
     */
    ioctl(0, FIONREAD, &arg);
    if (arg != 0)
      running = 0;

  } /* end of main control loop */

  // it will print out remaining data
  logging(mission.time, mot.motorspeed_l, mot.motorspeed_r, LAST_OUTPUT);
  odoLogging(odo.theta, odo.x, odo.y, LAST_OUTPUT);
  laserLogging(laserpar, LAST_OUTPUT);

  speedl->data[0] = 0;
  speedl->updated = 1;
  speedr->data[0] = 0;
  speedr->updated = 1;
  rhdSync();
  rhdDisconnect();
  exit(0);
}

/*
 * Routines to convert encoder values to positions.
 * Encoder steps have to be converted to meters, and
 * roll-over has to be detected and corrected.
 */

void reset_odo(odotype *p)
{
  p->right_pos = p->left_pos = 0.0;
  p->right_enc_old = p->right_enc;
  p->left_enc_old = p->left_enc;
  p->theta = 1.570;
  // coord_mobot2odo(3.4, -0.1);
  p->x = 0; // odoCoords[0];
  p->y = 0; // odoCoords[1];
}

void update_odo(odotype *p)
{
  int deltaR, deltaL;

  deltaR = p->right_enc - p->right_enc_old;

  if (deltaR > 0x8000)
    deltaR -= 0x10000;
  else if (deltaR < -0x8000)
    deltaR += 0x10000;

  p->right_enc_old = p->right_enc;
  p->right_pos += deltaR * p->cr;

  deltaL = p->left_enc - p->left_enc_old;

  if (deltaL > 0x8000)
    deltaL -= 0x10000;
  else if (deltaL < -0x8000)
    deltaL += 0x10000;

  p->left_enc_old = p->left_enc;
  p->left_pos += deltaL * p->cl;

  // our contribution to odo.
  double delta_Ul = deltaL * p->cl;
  double delta_Ur = deltaR * p->cr;
  double delta_Ui = (delta_Ul + delta_Ur) / 2;
  double delta_THETAi = (delta_Ur - delta_Ul) / p->w;

  p->theta += delta_THETAi;
  p->x += delta_Ui * cos(p->theta);
  p->y += delta_Ui * sin(p->theta);

  // printf("deltaTheta: %0.3f, theta: %0.3f, dUL: %0.3f, dUR: %0.3f\n", delta_THETAi, p->theta, delta_Ul, delta_Ur);

  odoLogging(p->theta, p->x, p->y, NOT_LAST_OUTPUT);
}

void update_motcon(motiontype *p)
{

  if (p->cmd != 0)
  {

    p->finished = 0;
    switch (p->cmd)
    {
    case mot_stop:
      printf("Mot_Stop\n");
      p->curcmd = mot_stop;
      break;
    case mot_followline:
      p->startpos = (p->left_pos + p->right_pos) / 2;
      if (p->angle > 0)
        p->startpos = p->right_pos;
      else
        p->startpos = p->left_pos;

      p->curcmd = mot_followline;
      break;
    case mot_direction_control:
      p->startpos = (p->left_pos + p->right_pos) / 2;
      if (p->angle > 0)
        p->startpos = p->right_pos;
      else
        p->startpos = p->left_pos;

      p->curcmd = mot_direction_control;
      break;

    case mot_followwall:
      p->startpos = (p->left_pos + p->right_pos) / 2;

      p->curcmd = mot_turn;
      break;

    case mot_move:
      p->startpos = (p->left_pos + p->right_pos) / 2;
      p->curcmd = mot_move;
      break;

    case mot_turn:
      if (p->angle > 0)
        p->startpos = p->right_pos;
      else
        p->startpos = p->left_pos;

      p->curcmd = mot_turn;
      break;

    case mot_turnr:
      if (p->angle > 0)
        p->startpos = p->right_pos;
      else
        p->startpos = p->left_pos;

      p->curcmd = mot_turnr;
      break;
    }

    p->cmd = 0;
  }

  switch (p->curcmd)
  {
  case mot_stop:
    p->motorspeed_l = 0;
    p->motorspeed_r = 0;
    break;

  case mot_followline:

    // Maximum allowed speed increment: v_delta_max = 0.005 m/s (updating at 100Hz)
    // options for modes:
    // CROSSING_BLACK_LINE 0  // Mode 0 = crossingblackline
    // END_OF_BLACK_LINE 1
    // IR_DISTANCE 2  // Mode 1 = IR distance
    // LASER_DISTANCE 3  // Mode 2 = Laser distance
    // ODO_DISTANCE 4  // Mode 3 = odo distance

    {
      static double motorR = 0, motorL = 0;
      double K = 1;
      double numerator = 0;
      double denuminator = 0;
      double xc = 0;
      double delta_v = 0;
      int sum = 0;
      if (p->colour == BLACK)
      {
        for (int i = 0; i < 8; i++)
        {
          // printf("Line values: ID: %d, Value: %d\n", i, linesensor->data[i]);
          sum += lineSensorCalibrated(linesensor->data[i]);
          numerator += i * (1 - linesensor->data[i]);
          denuminator += (1 - linesensor->data[i]);
        }
      }
      else if (p->colour == WHITE)
      {
        for (int i = 0; i < 8; i++)
        {
          // printf("Line values: ID: %d, Value: %d\n", i, linesensor->data[i]);
          sum += lineSensorCalibrated(linesensor->data[i]);
          numerator += i * (linesensor->data[i]);
          denuminator += linesensor->data[i];
        }

        // numerator = -numerator;
      }

      // for(int i = 0; i < 9; i++){
      //   printf("Laser: %0.3f\n", laserpar[i]);
      // }

      // printf("IR value: %0.3f\n", IRcalibrated(irsensor->data[IR_FORWARD_MIDDLE]));
      //  depending on the 'mode' it will will ether use IR distance, ODO driven distance, or black crossings to stop.
      //  if none of the stopping criteria are met, then it will always either ignore crossings, or correct its position, and keep following the line.
      sum /= 8;
      // printf("Sum: %d\n", sum);
      // printf("ODODISTANCE: %0.3f, %0.3f\n", ((p->right_pos + p->left_pos) / 2 - p->startpos), p->dist);
      // printf("ODO, x: %0.3f, y: %0.3f\n", odo.x, odo.y);
      if (p->mode == LASER_DISTANCE && (laserpar[p->IR_mode] <= p->dist && laserpar[p->IR_mode] != 0))
      {
        printf("Laser stop at dist: %0.3f\n", laserpar[p->IR_mode]);
        printf("Distance to box %0.3f\n", (laserpar[p->IR_mode] + 0.25));
        motorR = 0;
        motorL = 0;
        p->curcmd = mot_stop;
        p->finished = 1;
      }
      else if (p->mode == IR_DISTANCE && (IRcalibrated(irsensor->data[p->IR_mode]) <= p->dist))
      {
        // printf("IR, %0.3f\n", IRcalibrated(irsensor->data[p->IR_mode]));
        // printf("Final distance: %0.3f\n", fabs(odo.x) + IRcalibrated(irsensor->data[p->IR_mode]) + 0.25);
        motorR = 0;
        motorL = 0;
        p->curcmd = mot_stop;
        p->finished = 1;
      }
      else if (p->mode == ODO_DISTANCE && ((p->right_pos + p->left_pos) / 2 - p->startpos >= p->dist))
      {
        motorR = 0;
        motorL = 0;
        // printf("Odo Stop\n");
        p->curcmd = mot_stop;
        p->finished = 1;
      }
      else if (p->mode == CROSSING_BLACK_LINE && sum <= 32)
      {
        // stop
        // printf("Black line crossing\n");
        motorR = 0;
        motorL = 0;
        p->curcmd = mot_stop;
        p->finished = 1;
      }
      else if (p->mode == END_OF_BLACK_LINE && sum >= 125)
      {
        // printf("Black line ended\n");
        motorR = 0;
        motorL = 0;
        p->curcmd = mot_stop;
        p->finished = 1;
      }
      else if (p->mode == CROSSING_WHITE_LINE && sum >= 207)
      {
        // the motherfucking FLOOR IS WHITE. How in the FUCK are we supposed to CODE THIS TO WORK IN THE SIMULATION???????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
        // nvm it's 255, floor is 128
        // printf("White line crossing\n");
        motorR = 0;
        motorL = 0;
        p->curcmd = mot_stop;
        p->finished = 1;
      }
      else if (p->mode == END_OF_WHITE_LINE && (sum == 128 || sum == 0))
      {
        // printf("White line ended\n");
        motorR = 0;
        motorL = 0;
        p->curcmd = mot_stop;
        p->finished = 1;
      }
      /*else if (sum <= 50 && p->motorspeed_l == 0)
      {
        printf("Oof");
        p->motorspeed_l = 0.05;
        p->motorspeed_r = 0.05;
      }*/
      else if (sum <= 50)
      {
        // ignore crossing and drive forwards
        p->motorspeed_l = p->motorspeed_l;
        p->motorspeed_r = p->motorspeed_r;
      }
      else
      {

        double v_delta_max = 0.005;
        double v_max;
        double v_increment;

        if (p->mode == ODO_DISTANCE)
        {
          // The maximum allowed speed in relation to allowed deacceleration and remaining distance
          v_max = sqrt(2 * 0.5 * (p->dist - ((p->right_pos + p->left_pos) / 2 - p->startpos)));
        }
        else
        {
          // with a value this high, it will never come into play
          v_max = 10000;
        }

        // actual speed - wanted speed = error
        double v_error_l = p->speedcmd - motorL;

        // find the lowest between error and the maximum allowed speed increment, as to not overshoot the wanted speed.
        if (v_delta_max > v_error_l)
          v_increment = v_error_l;
        else
          v_increment = v_delta_max;

        // if the current speed is lower than what would otherwise be allowed due
        // to remaining distance, increase the speed by the increment.
        // otherwise, set the speed to v_max, the maximum allowed speed.

        xc = ((numerator / denuminator) * K);
        delta_v = K * (p->line - xc);

        // if no crossing, correct movement as usual
        if (p->colour == BLACK)
        {
          if (motorL < v_max)
          {
            // add the increment
            motorL += v_increment;
            motorR += v_increment;

            p->motorspeed_l = motorL - delta_v / 2;
            p->motorspeed_r = motorR + delta_v / 2;
          }
          else
          {
            p->motorspeed_l = v_max - delta_v / 2;
            p->motorspeed_r = v_max + delta_v / 2;
          }
        }
        else if (p->colour == WHITE)
        {
          if (motorL < v_max)
          {
            // add the increment
            motorL += v_increment;
            motorR += v_increment;

            p->motorspeed_l = motorL + delta_v / 2;
            p->motorspeed_r = motorR - delta_v / 2;
          }
          else
          {
            p->motorspeed_l = v_max + delta_v / 2;
            p->motorspeed_r = v_max - delta_v / 2;
          }
        }

        /*
        if (p->colour == BLACK)
        {
          xc = ((numerator / denuminator) * K);
          delta_v = K * (p->line - xc);
          p->motorspeed_l = p->speedcmd - delta_v / 2;
          p->motorspeed_r = p->speedcmd + delta_v / 2;
        }
        else if (p->colour == WHITE)
        {
          xc = ((numerator / denuminator) * K);
          delta_v = K * (p->line - xc);
          p->motorspeed_l = p->speedcmd + delta_v / 2;
          p->motorspeed_r = p->speedcmd - delta_v / 2;
        }*/
      }
    }

    break;

  case mot_direction_control:
    if ((p->right_pos + p->left_pos) / 2 - p->startpos >= p->dist)
    {
      p->finished = 1;

      p->motorspeed_l = 0;
      p->motorspeed_r = 0;
    }
    else
    {
      // Maximum allowed speed increment: v_delta_max = 0.005 m/s (updating at 100Hz)
      double K = 1;
      double delta_v = K * (p->ref_angle - odo.theta);

      p->motorspeed_l = p->speedcmd - delta_v / 2;
      p->motorspeed_r = p->speedcmd + delta_v / 2;
    }
    break;

  case mot_move:
  {
    int sum = 0;
    for (int i = 0; i < 8; i++)
    {
      sum += lineSensorCalibrated(linesensor->data[i]);
    }
    sum /= 8;
    printf("IR_Value1: %0.3f\n", IRcalibrated(irsensor->data[p->IR_mode]));
    if (p->mode == LASER_DISTANCE && (laserpar[p->IR_mode] <= p->dist && laserpar[p->IR_mode] != 0))
    {
      // printf("Laser stop at dist: %0.3f\n", laserpar[p->IR_mode]);
      p->curcmd = mot_stop;
      p->finished = 1;
    }
    else if (p->mode == ODO_DISTANCE && (fabs((p->right_pos + p->left_pos) / 2 - p->startpos) >= p->dist))
    {
      // printf("odox: %0.3f, odoy: %0.3f\n", odo.x, odo.y);
      p->finished = 1;

      p->motorspeed_l = 0;
      p->motorspeed_r = 0;
    }
    else if (p->mode == IR_DISTANCE && (IRcalibrated(irsensor->data[p->IR_mode]) > 0) && (IRcalibrated(irsensor->data[p->IR_mode]) <= p->dist))
    {
      p->finished = 1;

      p->motorspeed_l = 0;
      p->motorspeed_r = 0;
    }
    else if (p->mode == IR_DISTANCE_LOST && (IRcalibrated(irsensor->data[p->IR_mode]) >= p->dist))
    {
      // printf("IR_Value2: %0.3f\n", IRcalibrated(irsensor->data[p->IR_mode]));
      p->finished = 1;

      p->motorspeed_l = 0;
      p->motorspeed_r = 0;
    }
    else if (p->mode == CROSSING_BLACK_LINE && sum <= 32)
    {
      // stop
      // printf("Black line Stop\n");
      p->curcmd = mot_stop;
      p->finished = 1;
    }
    /*else
    {
      // Maximum allowed speed increment: v_delta_max = 0.005 m/s (updating at 100Hz)
      double K = 3;
      double delta_v = K * (p->ref_angle - odo.theta);

      double v_delta_max = 0.005;
      double v_max;
      double v_increment;

      // The maximum allowed speed in relation to allowed deacceleration and remaining distance
      v_max = sqrt(2 * 0.5 * (p->dist - ((p->right_pos + p->left_pos) / 2 - p->startpos)));

      // actual speed - wanted speed = error
      double v_error_l = p->speedcmd - p->motorspeed_l;

      // find the lowest between error and the maximum allowed speed increment, as to not overshoot the wanted speed.
      if (v_delta_max > v_error_l)
        v_increment = v_error_l;
      else
        v_increment = v_delta_max;

      // if the current speed is lower than what would otherwise be allowed due
      // to remaining distance, increase the speed by the increment.
      // otherwise, set the speed to v_max, the maximum allowed speed.
      if (p->motorspeed_l < v_max)
      {
        // add the increment

        p->motorspeed_l += v_increment - delta_v / 2;
        p->motorspeed_r += v_increment + delta_v / 2;
      }
      else
      {
        p->motorspeed_l = v_max - delta_v / 2;
        p->motorspeed_r = v_max + delta_v / 2;
      }
    }*/

    else
    {
      // Maximum allowed speed increment: v_delta_max = 0.005 m/s (updating at 100Hz)
      double v_delta_max = 0.005;
      double v_max;
      double v_increment;

      // The maximum allowed speed in relation to allowed deacceleration and remaining distance
      v_max = sqrt(2 * 0.5 * (p->dist - ((p->right_pos + p->left_pos) / 2 - p->startpos)));

      // actual speed - wanted speed = error
      double v_error_l = p->speedcmd - p->motorspeed_l;

      // find the lowest between error and the maximum allowed speed increment, as to not overshoot the wanted speed.
      if (v_delta_max > v_error_l)
        v_increment = v_error_l;
      else
        v_increment = v_delta_max;

      // if the current speed is lower than what would otherwise be allowed due
      // to remaining distance, increase the speed by the increment.
      // otherwise, set the speed to v_max, the maximum allowed speed.
      if (p->motorspeed_l < v_max)
      {
        // add the increment

        p->motorspeed_l += v_increment;
        p->motorspeed_r += v_increment;
      }
      else
      {

        p->motorspeed_l = v_max;
        p->motorspeed_r = v_max;
      }
    }
  }
  break;

  case mot_followwall:
  {
    static double wallDist = 0;
    static double motorL, motorR;

    if (wallDist == 0)
    {
      wallDist = IRcalibrated(irsensor->data[p->IR_mode]);
      motorL = p->motorspeed_l;
      motorR = p->motorspeed_r;
    }

    if ((p->right_pos + p->left_pos) / 2 - p->startpos >= p->dist)
    {
      // stop and return
      // printf("mot_followwall stop: distance\n");
      wallDist = 0;
      p->curcmd = mot_stop;
      p->finished = 1;
    }
    else if (IRcalibrated(irsensor->data[p->IR_mode]) > wallDist + 0.2f)
    {
      // stop and return
      // printf("mot_followwall stop: wall end\n");
      wallDist = 0;
      p->curcmd = mot_stop;
      p->finished = 1;
    }
    else if (IRcalibrated(irsensor->data[p->IR_mode_stop] < p->IR_dist_stop))
    {
      // stop and return
      // printf("mot_followwall stop: sensor no %d trigger\n", p->IR_mode_stop);
      wallDist = 0;
      p->curcmd = mot_stop;
      p->finished = 1;
    }
    else
    {
      // Maximum allowed speed increment: v_delta_max = 0.005 m/s (updating at 100Hz)
      double delta_v = 0;

      /*if (IRcalibrated(irsensor->data[p->IR_mode]) > wallDist + 0.01)
      {
        switch (p->mode)
        {
        case IR_LEFT:
          delta_v = 0.03;
          break;
        case IR_RIGHT:
          delta_v = -0.03;
          break;
        }
      }
      else if (IRcalibrated(irsensor->data[p->IR_mode]) < wallDist - 0.01)
      {
        switch (p->mode)
        {
        case IR_LEFT:
          delta_v = -0.03;
          break;
        case IR_RIGHT:
          delta_v = 0.03;
          break;
        }
      }
      else*/
      delta_v = 0;

      double v_delta_max = 0.005;
      double v_max;
      double v_increment;

      // The maximum allowed speed in relation to allowed deacceleration and remaining distance
      v_max = sqrt(2 * 0.5 * (p->dist - ((p->right_pos + p->left_pos) / 2 - p->startpos)));

      // actual speed - wanted speed = error
      double v_error_l = p->speedcmd - motorL;

      // find the lowest between error and the maximum allowed speed increment, as to not overshoot the wanted speed.
      if (v_delta_max > v_error_l)
        v_increment = v_error_l;
      else
        v_increment = v_delta_max;

      // if the current speed is lower than what would otherwise be allowed due
      // to remaining distance, increase the speed by the increment.
      // otherwise, set the speed to v_max, the maximum allowed speed.
      if (motorL < v_max)
      {
        // add the increment
        motorL += v_increment;
        motorR += v_increment;

        p->motorspeed_l = motorL - delta_v / 2;
        p->motorspeed_r = motorR + delta_v / 2;
      }
      else
      {
        p->motorspeed_l = v_max - delta_v / 2;
        p->motorspeed_r = v_max + delta_v / 2;
      }
    }
  }

  break;

  case mot_turn:

    if (p->angle > 0)
    {
      // Maximum allowed speed increment: v_delta_max = 0.005 m/s (updating at 100Hz)
      double v_delta_max = 0.005;
      double v_max;
      double v_increment;

      if (p->right_pos - p->startpos < p->angle * p->w / 2)
      {

        // The maximum allowed speed in relation to allowed deacceleration and remaining distance
        v_max = sqrt(2 * 0.5 * (p->angle * p->w / 2 - (p->right_pos - p->startpos)));

        // actual speed - wanted speed = error
        double v_error_r = p->speedcmd - p->motorspeed_r;

        // find the lowest between error and the maximum allowed speed increment, as to not overshoot the wanted speed.
        if (v_delta_max > v_error_r)
          v_increment = v_error_r;
        else
          v_increment = v_delta_max;

        // if the current speed is lower than what would otherwise be allowed due
        // to remaining distance, increase the speed by the increment.
        // otherwise, set the speed to v_max, the maximum allowed speed.
        if (p->motorspeed_r < v_max)
        {
          // add the increment
          p->motorspeed_l -= v_increment;
          p->motorspeed_r += v_increment;
        }
        else
        {
          p->motorspeed_l = -v_max;
          p->motorspeed_r = v_max;
        }
      }
      else
      {
        p->motorspeed_r = 0;
        p->motorspeed_l = 0;
        p->finished = 1;
      }
    }
    else
    {
      // Maximum allowed speed increment: v_delta_max = 0.005 m/s (updating at 100Hz)
      double v_delta_max = 0.005;
      double v_max;
      double v_increment;

      if (p->left_pos - p->startpos < fabs(p->angle) * p->w / 2)
      {

        // The maximum allowed speed in relation to allowed deacceleration and remaining distance
        v_max = sqrt(2 * 0.5 * (fabs(p->angle) * p->w / 2 - (p->left_pos - p->startpos)));

        // actual speed - wanted speed = error
        double v_error_l = p->speedcmd - p->motorspeed_l;

        // find the lowest between error and the maximum allowed speed increment, as to not overshoot the wanted speed.
        if (v_delta_max > v_error_l)
          v_increment = v_error_l;
        else
          v_increment = v_delta_max;

        // if the current speed is lower than what would otherwise be allowed due
        // to remaining distance, increase the speed by the increment.
        // otherwise, set the speed to v_max, the maximum allowed speed.
        if (p->motorspeed_l < v_max)
        {
          // add the increment
          p->motorspeed_l += v_increment;
          p->motorspeed_r -= v_increment;
        }
        else
        {
          p->motorspeed_l = v_max;
          p->motorspeed_r = -v_max;
        }
      }
      else
      {
        p->motorspeed_r = 0;
        p->motorspeed_l = 0;
        p->finished = 1;
      }
    }

    break;

  case mot_turnr:

    if (p->angle > 0)
    {
      double theta_dot = p->speedcmd / p->radius;

      // Stopping at correct arc length from the angular displacement
      if (p->angle * (p->radius - p->w) >= p->right_pos + p->w - p->startpos)
      {
        p->motorspeed_l = theta_dot * (p->radius - p->w);
        p->motorspeed_r = theta_dot * (p->radius + p->w);
      }
      else
      {
        p->motorspeed_r = 0;
        p->motorspeed_l = 0;
        p->finished = 1;
      }
    }
    else
    {
      double theta_dot = p->speedcmd / p->radius;
      // Stopping at correct arc length from the angular displacement
      if (fabs(p->angle) * (p->radius - p->w) >= p->left_pos + p->w - p->startpos)
      {

        p->motorspeed_l = theta_dot * (p->radius + p->w);
        p->motorspeed_r = theta_dot * (p->radius - p->w);
      }
      else
      {
        p->motorspeed_r = 0;
        p->motorspeed_l = 0;
        p->finished = 1;
      }
    }

    break;
  }
}

int followWall(double dist, double speed, int time, int IR_mode, int IR_mode_stop, double IR_dist_stop)
{
  if (time == 0)
  {
    mot.cmd = mot_followwall;
    mot.speedcmd = speed;
    mot.dist = dist;
    mot.IR_mode = IR_mode;
    mot.IR_mode_stop = IR_mode_stop;
    mot.IR_dist_stop = IR_dist_stop;
    return 0;
  }
  else
    return mot.finished;
}

int followline(double line, double dist, double speed, int time, int mode, int IR_mode, int colour)
{
  if (time == 0)
  {
    mot.cmd = mot_followline;
    mot.speedcmd = speed;
    mot.dist = dist;
    mot.line = line;
    mot.mode = mode;
    mot.IR_mode = IR_mode;
    mot.colour = colour;
    return 0;
  }
  else
    return mot.finished;
}

int direction_control(double ref_angle, double dist, double speed, int time)
{
  if (time == 0)
  {
    mot.cmd = mot_direction_control;
    mot.speedcmd = speed;
    mot.dist = dist;
    mot.ref_angle = ref_angle;
    return 0;
  }
  else
    return mot.finished;
}

int fwd(double dist, double speed, int time, int mode, int IR_mode)
{
  if (time == 0)
  {
    mot.cmd = mot_move;
    mot.speedcmd = speed;
    mot.dist = dist;
    mot.mode = mode;
    mot.IR_mode = IR_mode;
    return 0;
  }
  else
    return mot.finished;
}

int turn(double angle, double speed, int time)
{
  if (time == 0)
  {
    mot.cmd = mot_turn;
    mot.speedcmd = speed;
    mot.angle = angle;
    return 0;
  }
  else
    return mot.finished;
}

int turnr(double angle, double radius, double speed, int time)
{
  if (time == 0)
  {
    mot.cmd = mot_turnr;
    mot.speedcmd = speed;
    mot.angle = angle;
    mot.radius = radius;
    return 0;
  }
  else
    return mot.finished;
}

void sm_update(smtype *p)
{
  if (p->state != p->oldstate)
  {
    p->time = 0;
    p->oldstate = p->state;
  }
  else
  {
    p->time++;
  }
}

// int lineDataLinear[8];
// int lineCalibMeasurements[100];
//  int lineCalibCurrent = 0;

// needs mission.time as input.
void lineCalibLogging(int time)
{
  FILE *fp;

  fp = fopen("lineSensorCalibData.log", "a");

  // if(lineCalibCurrent == 100)
  //{
  for (int i = 0; i < 100; i++)
  {
    for (int j = 0; j < 8; j++)
    {
      fprintf(fp, "%d, ", linesensor->data[j]);
    }
    fprintf(fp, "measuring");
    fprintf(fp, "\n");
  }
  fclose(fp);
  //}
}

// measured values in the real world: 47, 58, ???
int pureBlack = 0, pureFloor = 128, pureWhite = 255, measBlack = 45, measFloor = 55, measWhite = 72, measOffset = 5;
// uses a linear expression derived from the measured values: y = a * x + b
// a = (measWhite - measBlack) / measWhite
// b = measBlack
// solved for 'x'

// This has to be changed. Something that takes values in different groups and makes them 0, 128 and 255 respectively.
int lineSensorCalibrated(int rawValue)
{
  if ((rawValue >= measBlack - measOffset) && (rawValue <= measBlack + measOffset))
  {
    return pureBlack;
  }
  /*else if((rawValue >= measFloor - measOffset) && (rawValue <= measFloor + measOffset))
  {
    return 128;
  }*/
  else if ((rawValue >= measWhite - measOffset) && (rawValue <= measWhite + measOffset))
  {
    return pureWhite;
  }
  else
  {
    return pureFloor;
  }

  /*
  double a = ((double)measWhite - (double)measBlack) / (double)pureWhite;

  return (int)((double)rawValue - (double)measBlack) / a;
  */

  // if we have to return a double between 0 and 1:
  // return (((double)rawValue - (double)measBlack) / a) / 128.0f;
}

int lineMinInt(int input[])
{
  int lowestInput = 129;
  for (int i = 0; i < 8; i++)
  {
    if (input[i] < lowestInput)
      lowestInput = input[i];
  }
  return lowestInput;
}

double IRcalibrated(int rawValue)
{
  double a = 12.0;
  double b = 81.0;

  return a / (rawValue - b);
}

double deg2rad(double degree)
{
  return degree / 180 * M_PI;
}

// returns an angle to a certain point on the course
double getAngleTo(double dest_x, double dest_y)
{
  double slope = (dest_x - odo.x) / (dest_y - odo.y);
  double newAngle = 0;

  if (dest_x > odo.x && dest_y > odo.y)
    newAngle = atan(slope);
  else if (dest_x > odo.x && dest_y < odo.y)
    newAngle = atan(slope) + 3 * M_PI;
  else if (dest_x < odo.x && dest_y > odo.y)
    newAngle = atan(slope) + M_PI;
  else if (dest_x < odo.x && dest_y < odo.y)
    newAngle = atan(slope) + 2 * M_PI;

  if (newAngle >= 2 * M_PI)
    newAngle -= 2 * M_PI;

  // printf("%0.3f, %0.3f\n", newAngle, deg2rad(odo.theta));

  if (fabs(newAngle - deg2rad(odo.theta)) < (0.5 * M_PI / 180))
    return 0;

  return newAngle - deg2rad(odo.theta);
}

// returns a distance to a certain point on the course
double getDistanceTo(double dest_x, double dest_y)
{
  return sqrt(pow((dest_x - odo.x), 2) + pow((dest_y - odo.y), 2));
}

void coord_odo2mobot(double x, double y)
{
  mobotCoords[0] = T1 * x + T2 * y;
  mobotCoords[1] = T3 * x + T4 * y;
}

void coord_mobot2odo(double x, double y)
{
  double inverse = 1 / ((T1 * T4) - (T2 * T3));
  odoCoords[0] = (inverse * T4) * x + (inverse * -T2) * y;
  odoCoords[1] = (inverse * -T3) * x + (inverse * T1) * y;
}