#include "MatrixMath.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[42]; // FIFO storage buffer

// orientation/motion vars
Quaternion quat;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3], ypr_prev[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int16_t gyro_raw[3], accel_raw[3];
float gyro[3] = {0,0,0}, accel[3] = {0,0,0}, roll, pitch, yaw, p, q, r, dp_dt, dq_dt, dr_dt, phi_dot, theta_dot, psi_dot, phi, theta, psi, phi_accel, theta_accel;
//float B_gyro = 1.0;
//volatile float ypr_shared[3];

// ================================================================
// ===                    mean and std_dev                      ===
// ================================================================
int NS = 1000;
float data[1000];
int Ni = 0;
float mean = 0.0;
float std_dev = 0.0;

char serialprint[100];

// ================================================================
// ===                  Motor Control Vars                      ===
// ================================================================
#include "DSHOT.h"
const int n_esc = 6;
uint16_t cmd[n_esc];
uint8_t tlm[n_esc];
bool disarm = true;
float telem;
uint16_t Dshot_delay = 1500;

//not used, just for reference (these pins cannot be changed)
const uint8_t m1_PIN = 4;  // CW
const uint8_t m2_PIN = 8;  // CCW
const uint8_t m3_PIN = 22; // CCW
const uint8_t m4_PIN = 9;  // CW

float m1_thr, m2_thr, m3_thr, m4_thr;

float thrust = 0.3f;
float L = 0.0f;
float M = 0.0f;
float N = 0.0f;

//gains
float Kp_roll = 7.0;
float Ki_roll = 0.0008;

float Kp_pitch = 6.0;
float Ki_pitch = 0.0008;


float Kp_L = 0.3;
float Kd_L = 10e-4;
float Ki_L = 0.0003;

float Kp_M = 0.34;
float Kd_M = 10e-4;
float Ki_M = 0.0003;//0.0012;

float Kp_N = 0.8;
float Kd_N = 0.0;//4e-4;
float Ki_N = 0.0003;

//float Kp_L = 0.8;
//float Kd_L = 10e-4;
//float Ki_L = 0.0003;
//
//float Kp_M = 0.9;
//float Kd_M = 10e-4;
//float Ki_M = 0.0003;//0.0012;
//
//float Kp_N = 0.8;
//float Kd_N = 0.0;//4e-4;
//float Ki_N = 0.0003;

// desired variables
float thrust_des = 0.0f;

float roll_des = 0.0;
float pitch_des = 0.0;

float rolld = 0.0;
float pitchd = 0.0;
float yawd = 0.0;

float p_des = 0.0;
float q_des = 0.0;
float r_des = 0.0;

// for low passing 
float thrust_n = 0.0;
float roll_des_n = 0.0;
float pitch_des_n = 0.0;
float p_des_n = 0.0;
float q_des_n = 0.0;
float r_des_n = 0.0;

// velocity commands
float yaw_des = 0.0;
float u_des = 0.0;
float v_des = 0.0;
float alt_des = 0.0;
float u_des_n = 0.0;
float v_des_n = 0.0;
float alt_n = 0.0;
float yaw_des_n = 0.0;

// integral variables
float err_sum_roll = 0.0;
float err_sum_pitch = 0.0;

float err_sum_p = 0.0;
float err_sum_q = 0.0;
float err_sum_r = 0.0;

// low pass
float LP_accel0 = 0.05;//0.05 - 0.5;
float LP_accel1 = 0.05;//0.05 - 0.5;
float LP_accel2 = 0.05;//0.05 - 0.5;
float LP_gyro   = 1.0;//0.05 - 0.5;

float LP_roll = 1.0;//0.009;
float LP_pitch = 1.0;//0.009;

// through Serial
float r_d_telem = 0.0;
float p_d_telem = 0.0;
float y_d_telem = 0.0;

// ================================================================
// ===                       RECEIVER VARS                      ===
// ================================================================
#include <SpektrumSattelite.h>

SpektrumSattelite rx;
volatile bool Rec_valid = 1;
unsigned long time;

float LP_rec = 0.5;

//#define rate_control

#ifdef rate_control
//// rate control
#define Thro_Lim_Low  153.0f
#define Thro_Lim_High 869.0f
#define Thro_slope  4.0f/(Thro_Lim_High - Thro_Lim_Low)
#define Thro_bias  -Thro_slope * Thro_Lim_Low

#define Rudd_Lim_Low  153.0f
#define Rudd_Lim_High 870.0f
#define Rudd_slope -240.0f*M_PI/180.0f/(Rudd_Lim_High - Rudd_Lim_Low) //twice of max/delta
#define Rudd_bias  -Rudd_slope * 511

#define Elev_Lim_Low  153.0f
#define Elev_Lim_High 870.0f
#define Elev_slope -240.0f*M_PI/180.0f/(Elev_Lim_High - Elev_Lim_Low) //twice of max/delta
#define Elev_bias  -Elev_slope * 507

#define Aile_Lim_Low  153.0f
#define Aile_Lim_High 817.0f
#define Aile_slope -240.0f*M_PI/180.0f/(Aile_Lim_High - Aile_Lim_Low) //twice of max/delta
#define Aile_bias  -Aile_slope * 487

#else
//// for attitude control
#define Thro_Lim_Low  150.0f
#define Thro_Lim_High 869.0f
#define Thro_slope  4.0f/(Thro_Lim_High - Thro_Lim_Low)
#define Thro_bias  -Thro_slope * Thro_Lim_Low

#define Rudd_Lim_Low  153.0f
#define Rudd_Lim_High 870.0f
#define Rudd_slope -240.0f*M_PI/180.0f/(Rudd_Lim_High - Rudd_Lim_Low) //twice of max/delta
#define Rudd_bias  -Rudd_slope * 511

#define Elev_Lim_Low  153.0f
#define Elev_Lim_High 870.0f
#define Elev_slope -60.0f*M_PI/180.0f/(Elev_Lim_High - Elev_Lim_Low) //twice of max/delta
#define Elev_bias  -Elev_slope * 507

#define Aile_Lim_Low  153.0f
#define Aile_Lim_High 817.0f
#define Aile_slope -60.0f*M_PI/180.0f/(Aile_Lim_High - Aile_Lim_Low) // twice of max/delta
#define Aile_bias  -Aile_slope * 487
#endif

//// velocity control
#define Alt_Lim_Low  150.0f
#define Alt_Lim_High 869.0f
#define Alt_slope  2.0f/(Alt_Lim_High - Alt_Lim_Low) 
#define Alt_bias  -Alt_slope * Alt_Lim_Low

#define yaw_Lim_Low  153.0f
#define yaw_Lim_High 870.0f
#define yaw_slope -360.0f*M_PI/180.0f/(yaw_Lim_High - yaw_Lim_Low) //twice of max/delta
#define yaw_bias  -yaw_slope * 511

#define u_Lim_Low  153.0f
#define u_Lim_High 870.0f
#define u_slope  3.0/(u_Lim_High - u_Lim_Low) //twice of max/delta
#define u_bias  -u_slope * 507

#define v_Lim_Low  153.0f
#define v_Lim_High 817.0f
#define v_slope -3.0/(v_Lim_High - v_Lim_Low) //twice of max/delta
#define v_bias  -v_slope * 487


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
unsigned long current_time, prev_time, start_time, log_time, blink_time;
float dt_loop;
float dt;

uint8_t vrbl_trnsmt_ctr = 0;


// ================================================================
// ===                     KALMAN(for gyro)                     ===
// ================================================================

IntervalTimer stability_timer;
int stability_freq = 1000;
double dt_KF = 1/(1.0*stability_freq);
double F_KF[2][2] = {{1.0, dt_KF}, {0.0, 1.0}};
double F_KF_T[2][2] = {{1.0, 0.0}, {dt_KF, 1.0}};

double sigma_alphax2 = 1e7, sigma_alphay2 = 1e7, sigma_alphaz2 = 1e7; // process model disturbances
double sigma_gx2 = 1e-3, sigma_gy2 = 1e-3, sigma_gz2 = 1e-3; // measurement noise, increase for higher filtering
//double sigma_alphax2 = 1e4, sigma_alphay2 = 1e4, sigma_alphaz2 = 1e4; // process model disturbances
//double sigma_gx2 = 1e-4, sigma_gy2 = 1e-4, sigma_gz2 = 1e-4; // measurement noise, increase for higher filtering
double Q_KF[2][2] = {{0.25*pow(dt_KF, 4.0), 0.5*pow(dt_KF, 3.0)}, {0.5*pow(dt_KF, 3.0), pow(dt_KF, 2.0)}};

double x_k_p[2] = {0.0, 0.0};
double x_k_q[2] = {0.0, 0.0};
double x_k_r[2] = {0.0, 0.0};

double P_k_p[2][2] = {{1000.0, 1000.0},{1000.0, 1000.0}};
double P_k_q[2][2] = {{1000.0, 1000.0},{1000.0, 1000.0}};
double P_k_r[2][2] = {{1000.0, 1000.0},{1000.0, 1000.0}};


double p_temp = 0.0;

// ================================================================
// ===                  KALMAN 1(for angles)                    ===
// ================================================================

double F_KF1[2][2] = {{1.0, -dt_KF}, {0.0, 1.0}};
double F_KF1_T[2][2] = {{1.0, 0.0}, {-dt_KF, 1.0}};
double B_KF1[2] = {dt_KF, 0.0};

//double sigma_phidot2 = 1e-5, sigma_thetadot2 = 1e-5, sigma_psidot2 = 1e-5; // process model disturbances1(contains noise from gyroscope)
//double sigma_phiddot2 = 1e7, sigma_thetaddot2 = 1e7, sigma_psiddot2 = 1e7; // process model disturbances2(true process model noise)
double sigma_phidot2 = 1e-4;
double sigma_phiddot2 = 1;
double sigma_phidot_phiddot = 1e-2;
double sigma_bdot2  = 1e-1;
double sigma_phi2 = 0.04, sigma_theta2 = 0.04, sigma_psi2 = 0.04; // measurement noise from accelerometers
double Q_KF1[2][2] = {{sigma_phidot2*pow(dt_KF, 2.0) + sigma_phidot_phiddot*pow(dt_KF,3.0) + 0.25*sigma_phiddot2*pow(dt_KF, 4.0), 0.0}, {0.0, sigma_bdot2*pow(dt_KF, 2.0)}};
//double QG1_KF1[2][2] = {{pow(dt_KF, 2.0), 0.0}, {0.0, 0.0}};
//double QG2_KF1[2][2] = {{0.25*pow(dt_KF, 4.0)*1e6, 0.0}, {0.0, pow(dt_KF, 2.0)*1e1}};

double x_k_phi[2]   = {0.0, 0.0};
double x_k_theta[2] = {0.0, 0.0};
double x_k_psi[2]   = {0.0, 0.0};

double P_k_phi[2][2]   = {{1000.0, 1000.0},{1000.0, 1000.0}};
double P_k_theta[2][2] = {{1000.0, 1000.0},{1000.0, 1000.0}};
double P_k_psi[2][2]   = {{0.0, 0.0},{0.0, 0.0}};


// ===============================================================
// ========             reading flight computer         ==========
// ===============================================================
char inputString[200] = "";         // a String to hold incoming data
int idx = 0;
char prev_char = 0;
char prev_prev_char = 0;;
char new_char = 0;
String value = "";
int comma_received = 0;
float outer_loop_cmd[5];
long sum_of_digits = 0;
long checksum = 0, n_data_received = 0;
float Td = 0, phid = 0, thetad = 0, r_d = 0;

char print_out[100];

bool recvInProgress = false;
bool newData = false;

int auto_on = 0;

unsigned long outer_loop_receive_time;



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup(){
    Serial.begin(2000000);
    Serial5.begin(2000000);
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    
    setup_IMU();
    
    setup_ESCs();

    setup_Receiver();
    
    // get current time for measuring loop rate
    current_time = micros();
    start_time = current_time;
    delayMicroseconds(500);
    calibrate_gyro();
    log_data();
    log_time = micros();
    blink_time = log_time;
}


//// ================================================================
//// ===                       PROGRAM LOOP                       ===
//// ================================================================
unsigned short rec_fs = 0;
void loop() {
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0; 
//  Serial.println(dt_loop*1000);
      F_KF[0][1] = dt;
      F_KF_T[1][0] = dt;
      Q_KF[0][0] = 0.25*pow(dt, 4.0);
      Q_KF[0][1] = 0.50*pow(dt, 3.0);
      Q_KF[1][0] = 0.50*pow(dt, 3.0);
      Q_KF[1][1] = pow(dt, 2.0);
    
      F_KF1[0][1] = -dt;
      F_KF1_T[1][0] = -dt;
      B_KF1[0] = dt;
      Q_KF1[0][0] = sigma_phidot2*pow(dt, 2.0) + sigma_phidot_phiddot*pow(dt, 3.0) + 0.25*sigma_phiddot2*pow(dt, 4.0);
      Q_KF1[1][1] = sigma_bdot2*pow(dt_KF, 2.0);

//      QG1_KF1[0][0] = pow(dt, 2.0);
//      QG2_KF1[0][0] = 0.25*pow(dt, 4.0);
//      QG2_KF1[1][1] = 0.25*pow(dt, 4.0);

  read_rec();
  if(millis() - time >= 500)
  {
    Rec_valid = false;
  }
//  Serial.print(time);
//Serial.println(rx.getThro());
  if(Rec_valid)
  {
    rec_fs = 0;
    
    #ifdef rate_control
      thrust_n = constrain(rx.getThro(), Thro_Lim_Low, Thro_Lim_High) * Thro_slope + Thro_bias;
      p_des_n = constrain(rx.getAile(), Aile_Lim_Low, Aile_Lim_High) * Aile_slope + Aile_bias;
      q_des_n = constrain(rx.getElev(), Elev_Lim_Low, Elev_Lim_High) * Elev_slope + Elev_bias;
      r_des_n = constrain(rx.getRudd(), Rudd_Lim_Low, Rudd_Lim_High) * Rudd_slope + Rudd_bias;

      thrust_des = (1-LP_rec)*thrust_des + LP_rec*thrust_n;
      p_des  = (1-LP_rec)*p_des  + LP_rec*p_des_n;
      q_des  = (1-LP_rec)*q_des  + LP_rec*q_des_n;
      r_des  = (1-LP_rec)*r_des  + LP_rec*r_des_n;
      
    #else
      thrust_n = constrain(rx.getThro(), Thro_Lim_Low, Thro_Lim_High) * Thro_slope + Thro_bias;
      roll_des_n = constrain(rx.getAile(), Aile_Lim_Low, Aile_Lim_High) * Aile_slope + Aile_bias;
      pitch_des_n = constrain(rx.getElev(), Elev_Lim_Low, Elev_Lim_High) * Elev_slope + Elev_bias;
      r_des_n = constrain(rx.getRudd(), Rudd_Lim_Low, Rudd_Lim_High) * Rudd_slope + Rudd_bias;

      thrust_des    = (1-LP_rec)*thrust_des    + LP_rec*thrust_n;
      roll_des  = (1-LP_rec)*roll_des  + LP_rec*roll_des_n;
      pitch_des = (1-LP_rec)*pitch_des + LP_rec*pitch_des_n;
      r_des     = (1-LP_rec)*r_des     + LP_rec*r_des_n; 
    #endif

    alt_n = constrain(rx.getThro(), Alt_Lim_Low, Alt_Lim_High) * Alt_slope + Alt_bias;
    v_des_n = constrain(rx.getAile(), v_Lim_Low, v_Lim_High) * v_slope + v_bias;
    u_des_n = constrain(rx.getElev(), u_Lim_Low, u_Lim_High) * u_slope + u_bias;
    yaw_des_n = constrain(rx.getRudd(), yaw_Lim_Low, yaw_Lim_High) * yaw_slope + yaw_bias;
  
    alt_des = (1-LP_rec)*alt_des + LP_rec*alt_n;
    v_des  = (1-LP_rec)*v_des  + LP_rec*v_des_n;
    u_des  = (1-LP_rec)*u_des  + LP_rec*u_des_n;
    yaw_des  = (1-LP_rec)*yaw_des  + LP_rec*yaw_des_n;


    
//  Serial.println(rx.getThro());
    if(rx.getAux1() >= 546 && disarm)
    {
        disarm = false;
        cmd[0] = 48;
      cmd[1] = 48;
      cmd[2] = 48;
      cmd[3] = 48;
      cmd[4] = 48;
      cmd[5] = 48;
//        stability_timer.end();
        for (int i=0; i<350; i++)
        {
        DSHOT_send( cmd, tlm );
        delayMicroseconds(Dshot_delay);
        DSHOT_send( cmd, tlm );
        delayMicroseconds(Dshot_delay);
        }
        err_sum_roll = 0.0;
        err_sum_pitch = 0.0;
        err_sum_p = 0.0;
        err_sum_q = 0.0;
        err_sum_r = 0.0; 
//        stability_timer.begin(stability_loop_angle, 500);// stability loop at 2kHz
    }
    else if(rx.getAux1() < 546 && !disarm)
    {
      cmd[0] = 48;
      cmd[1] = 48;
      cmd[2] = 48;
      cmd[3] = 48;
      cmd[4] = 48;
      cmd[5] = 48;
      DSHOT_send( cmd, tlm );
      delayMicroseconds(200);
      disarm = true;
    }
//    Serial.println(rx.getAux1());
    if(rx.getGear() > 600)
    {
      auto_on = true;
    }
    else
    {
      auto_on = false;
    }
      
  }
  else //!Rec_valid
  {
//     failsafe values
//    thrust = 0.4f;
//    p_des = 0.0;
//    q_des=  0.0;
//    roll_des = r_d_telem;
//    pitch_des = p_d_telem;
//    r_des = y_d_telem;

  rec_fs++;
  }

  read_Fcom();
//  Serial.println(outer_loop_cmd[4]);
  if(newData)
  {
            Td = constrain(outer_loop_cmd[0], 0.0, 4.0);
            phid = constrain(outer_loop_cmd[1], -30.0, 30.0)*M_PI/180.0;
            thetad = constrain(outer_loop_cmd[2], -30.0, 30.0)*M_PI/180.0;
            r_d = constrain(outer_loop_cmd[3], -30.0, 30.0)*M_PI/180.0;
//  sprintf(print_out, "%f, %f, %f, %f, %f, %ld, %ld, %ld\n", Td, phid*180.0/M_PI, thetad*180.0/M_PI, psid*180.0/M_PI, outer_loop_cmd[4], sum_of_digits, checksum, n_data_received);
//  Serial.print(print_out);
            newData = false;
            outer_loop_receive_time = micros();
  }
//  Serial.println(auto_on);
  if(auto_on && (int)outer_loop_cmd[4])// && micros() - outer_loop_receive_time < 20000 )// 50 Hz
  {
    
    thrust = Td;
    rolld = phid;
    pitchd = thetad;
    r_des = r_d;
//    Serial.println(thrust);
    
//  thrust = thrust_des;
//    rolld = roll_des;
//    pitchd = pitch_des;
  }
  else
  {
    thrust = thrust_des;
    rolld = roll_des;
    pitchd = pitch_des;
  }
//  Serial.println(String(auto_on,5));

  if (thrust < 0.3)
    {
//      thrust = 0.0f;
      
      err_sum_roll = 0.0;
      err_sum_pitch = 0.0;
      err_sum_p = 0.0;
      err_sum_q = 0.0;
      err_sum_r = 0.0; 
    }

    

        
        // rates in deg/s
        mpu.getRotation(&gyro_raw[0], &gyro_raw[1], &gyro_raw[2]);
        gyro[0] = (1 - LP_gyro) * gyro[0] + LP_gyro * ( gyro_raw[0]/131.0);// - gyro_err[0]);
        gyro[1] = (1 - LP_gyro) * gyro[1] + LP_gyro * (-gyro_raw[1]/131.0);// - gyro_err[1]);
        gyro[2] = (1 - LP_gyro) * gyro[2] + LP_gyro * (-gyro_raw[2]/131.0);// - gyro_err[2]); 

        //rates in radian/s
        p = gyro[0] * M_PI / 180.0;
        q = gyro[1] * M_PI / 180.0;
        r = gyro[2] * M_PI / 180.0;

        //Kalman
        Kalman(x_k_p, P_k_p, p, sigma_alphax2, sigma_gx2);
        Kalman(x_k_q, P_k_q, q, sigma_alphay2, sigma_gy2);
        Kalman(x_k_r, P_k_r, r, sigma_alphaz2, sigma_gz2);

        p = x_k_p[0];
        q = x_k_q[0];
        r = x_k_r[0];
        dp_dt = x_k_p[1];
        dq_dt = x_k_q[1];
        dr_dt = x_k_r[1];
        
//dp_dt = 0.0;
//dq_dt = 0.0;
//dr_dt = 0.0;

#ifndef rate_control

      phi_dot = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
      theta_dot = cos(phi)*q - sin(phi)*r;
      psi_dot = sin(phi)/cos(theta)*q + cos(phi)/cos(theta)*r;

      mpu.getAcceleration(&accel_raw[0], &accel_raw[1], &accel_raw[2]);
      
      accel[0] = (1 - LP_accel0) * accel[0] + LP_accel0 * (-accel_raw[0]/8192.0);
      accel[1] = (1 - LP_accel1) * accel[1] + LP_accel1 * ( accel_raw[1]/8192.0);
      accel[2] = (1 - LP_accel2) * accel[2] + LP_accel2 * ( accel_raw[2]/8192.0);
//      Serial.println(accel[2]);

      phi_accel = atan(accel[1]/accel[2]);
      theta_accel = atan(-accel[0]/sqrt(accel[1]*accel[1] + accel[2]*accel[2]));
////      
      Kalman1(x_k_phi,   P_k_phi,   phi_dot,   phi_accel,  sigma_phi2);
      Kalman1(x_k_theta, P_k_theta, theta_dot, theta_accel, sigma_theta2);

      phi = x_k_phi[0];
      theta = x_k_theta[0];

//      Kalman2(x_k_psi, P_k_psi, psi_dot, sigma_alphaz2, sigma_gz2);
//      psi = x_k_psi[0];
      psi = psi + psi_dot*dt;// - x_k_phi[1]*dt;

//Serial.println(theta*180.0/M_PI);
//Serial.print(",  ");
//Serial.
      
      p_des_n = Kp_roll * (rolld - phi) +  Ki_roll * err_sum_roll;
      q_des_n = Kp_pitch * (pitchd - theta) + Ki_pitch * err_sum_pitch;
//    r_d = Kp_yaw * (yaw_des - yaw) + Ki_yaw * err_sum_yaw;



    err_sum_roll += rolld - phi;
    err_sum_pitch += pitchd - theta;

    err_sum_roll = constrain(err_sum_roll , -1.0/Ki_roll, 1.0/Ki_roll);
    err_sum_pitch = constrain(err_sum_pitch , -1.0/Ki_pitch, 1.0/Ki_pitch);

    p_des = LP_roll * p_des_n + (1-LP_roll)*p_des;
    q_des = LP_pitch * q_des_n + (1-LP_pitch)*q_des;


    p_des = constrain(p_des, -200, 200);
    q_des = constrain(q_des, -200, 200);
    r_des = constrain(r_des, -200, 200);

//    Serial.print(" ,p_des, ");
//    Serial.print(p_des*180/M_PI);
//    Serial.print(" ,q_des, ");
//    Serial.print(q_des*180/M_PI);
//    Serial.print("\n");
//Serial.println(r_des*180/M_PI);
#endif

    L = Kp_L * (p_des - p) + Kd_L * (0 - dp_dt) + Ki_L * err_sum_p;
    M = Kp_M * (q_des - q) + Kd_M * (0 - dq_dt) + Ki_M * err_sum_q;
    N = Kp_N * (r_des - r) + Kd_N * (0 - dr_dt) + Ki_N * err_sum_r;
    
    err_sum_p += p_des - p;
    err_sum_q += q_des - q;
    err_sum_r += r_des - r;

    err_sum_p = constrain(err_sum_p, -0.4/Ki_L, 0.4/Ki_L);
    err_sum_q = constrain(err_sum_q, -0.4/Ki_M, 0.4/Ki_M);
    err_sum_r = constrain(err_sum_r, -0.8/Ki_N, 0.8/Ki_N);

//    Serial.print(" ,p, ");
//    Serial.print(p*180/M_PI);
//    Serial.print(" ,q, ");
//    Serial.print(q*180/M_PI);
//    Serial.print(" ,r, ");
//    Serial.print(r*180/M_PI);
//    Serial.print("\n");

    
    
  
//    L = 0;
//    M = 0;
//    N = 0;
//    
    
    
    //  mixer T/T_max L/l*T_max M/l*T_max N/Q_max, T_max and Q_max are motors max thrust and torque
    m1_thr = (thrust + L + M + N) * 0.25;
    m2_thr = (thrust + L - M - N) * 0.25; 
    m3_thr = (thrust - L - M + N) * 0.25;
    m4_thr = (thrust - L + M - N) * 0.25;

//          
    m1_thr = constrain(m1_thr, 0.0f, 1.0f);
    m2_thr = constrain(m2_thr, 0.0f, 1.0f);
    m3_thr = constrain(m3_thr, 0.0f, 1.0f);
    m4_thr = constrain(m4_thr, 0.0f, 1.0f);

//    Serial.print(" ,m1_thr, ");
//    Serial.print(m1_thr);
//    Serial.print(" ,m2_thr, ");
//    Serial.print(m2_thr);
//    Serial.print(" ,m3_thr, ");
//    Serial.print(m3_thr);
//    Serial.print(" ,m4_thr, ");
//    Serial.print(m4_thr);
//    Serial.print("\n");

    // DSHOT cmds
    cmd[0] = m1_thr*1999 + 48;
    cmd[1] = m2_thr*1999 + 48;
    cmd[3] = m3_thr*1999 + 48;
    cmd[5] = m4_thr*1999 + 48;
    
    if(!disarm && rec_fs<=100)
    {
      DSHOT_send( cmd, tlm );
      delayMicroseconds(200);
      // ESC disarms if nothing is received within 1.5ms, send zero throttle(or 48) to arm again
    }
//    Serial.println(dt*1000);

//  if(micros() - log_time > 10000)
//  {
//    log_data();
//    log_time = micros();
//  }

send_data();
//Serial.println(thrust);
//Serial.println(micros() - current_time);


    while(micros() - current_time < 1000)
    {
      
    }

//    // blink LED to indicate activity
if(micros() - blink_time > 500000)
{
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
  blink_time = micros();
}
//    blinkState = !blinkState;
//    digitalWrite(LED_PIN, blinkState);
//    delayMicroseconds(2000);// 500Hz loop   
    
}

void read_Fcom() {
  
  while (Serial5.available()>0 && newData == false) {
    // get the new byte:
    prev_prev_char = prev_char;
    prev_char = new_char;
    new_char = (char)Serial5.read();
//    Serial.println(new_char);
    if(recvInProgress == true)
    {
      if(comma_received < 5)
      {
        if(new_char == '-' && prev_char == ',')
        {
          value = new_char;
        }
        else if ((new_char >= '0' && new_char <= '9') || new_char == '.')
        {
          value += new_char;
          if(new_char != '.')
          {
            sum_of_digits += new_char - '0';
          }
        }
        else if(new_char == ',' && value != "")
        {// new command received
          
          outer_loop_cmd[comma_received] = value.toFloat();  
          
//          Serial.println(comma_received);
          
          value = "";
          comma_received++;
        }
        else
        {// wrong character
          recvInProgress = false;
        }
      }
      else
      {
        if (new_char >= '0' && new_char <= '9')
        {
          value += new_char;
        }
        else if(new_char == '\n')
        {
          checksum = value.toInt();
          
          if(checksum == sum_of_digits)
          { 
            newData = true;
            n_data_received++;
          }
          recvInProgress = false;
          
        }
        else
        {
          recvInProgress = false;
        } 
      }
   
    }
    else if(prev_prev_char == 'H' && prev_char == 'i' && new_char == ',')
    {
      recvInProgress = true;
      comma_received = 0;
      value = "";
      sum_of_digits = 0;
    }
  }
}

void send_data()
{
  String data = "Hi,prm," ;
  if(vrbl_trnsmt_ctr == 0)
  {
    data = data + "rdo," + String(auto_on,5);
  }
  else if(vrbl_trnsmt_ctr == 1)
  {
    data = data + "alt," + String(alt_des,5);
  }
  else if(vrbl_trnsmt_ctr == 2)
  {
    data = data + "_u_," + String(u_des,5);
  }
  else if(vrbl_trnsmt_ctr == 3)
  {
    data = data + "_v_," + String(v_des,5);
  }
  else if(vrbl_trnsmt_ctr == 4)
  {
    data = data + "ywd," + String(yaw_des,5);
  }
  else
  {
    Serial.println("WTF: error in send_data");
  }
  
  data = data + "," + String(data.length()) + "\n";
  Serial5.print(data);
  vrbl_trnsmt_ctr++;
  if(vrbl_trnsmt_ctr == 5)
  {
    vrbl_trnsmt_ctr = 0;
  }

  
  
}

void log_data()
{
//  String data = "Hi, " + String(micros()-start_time) + "," + String(dt*1000);
////  data = data + " ,roll_des, " + String(roll_des*180/M_PI, 3);
//  data = data + " ,roll_accel, " + String(phi_accel*180/M_PI, 3);
//  data = data + " ,roll, " + String(phi*180/M_PI, 3);
////  data = data + " ,p_des, " + String(p_des*180/M_PI, 3);
//  data = data + " ,p, " + String(p*180/M_PI, 3);
//  data = data + " ,p_gyro, " + String(gyro[0], 3);
//
////  String data = "Hi, " + String(micros()-start_time) + "," + String(dt*1000);
////  data = data + " ,pitch_des, " + String(pitch_des*180/M_PI, 3);
//  data = data + " ,pitch_accel, " + String(theta_accel*180/M_PI, 3);
//  data = data + " ,pitch, " + String(theta*180/M_PI, 3);
////  data = data + " ,q_des, " + String(q_des*180/M_PI, 3);
//  data = data + " ,q, " + String(q*180/M_PI, 3);
//  data = data + " ,q_gyro, " + String(gyro[1], 3);


  String data = "Hi," + String(micros()-start_time);
  data = data + "," + String(phi_accel*180/M_PI, 5) + "," + String(theta_accel*180/M_PI, 5);
  data = data + "," + String(gyro[0], 5) + "," + String(gyro[1], 5) + "," + String(gyro[2], 5);



  
  data = data + "," + String(data.length()) + "\n";
  Serial5.print(data);

}


void calibrate_gyro()
{
  for(int i = 0; i<300; i++)
  {
    prev_time = current_time;      
    current_time = micros();      
    dt = (current_time - prev_time)/1000000.0;
    //  Serial.println(dt*1000); 
    F_KF[0][1] = dt;
    F_KF_T[1][0] = dt;
    Q_KF[0][0] = 0.25*pow(dt, 4.0);
    Q_KF[0][1] = 0.50*pow(dt, 3.0);
    Q_KF[1][0] = 0.50*pow(dt, 3.0);
    Q_KF[1][1] = pow(dt, 2.0);

    get_gyro();
      
    while(micros() - current_time < 500)
    {
      
    }
    
  }
}

void get_gyro()
{
      mpu.getRotation(&gyro_raw[0], &gyro_raw[1], &gyro_raw[2]);
      gyro[0] =  gyro_raw[0]/131.0;
      gyro[1] = -gyro_raw[1]/131.0;
      gyro[2] = -gyro_raw[2]/131.0;
      
      //rates in radian/s
      p = gyro[0] * M_PI / 180.0;
      q = gyro[1] * M_PI / 180.0;
      r = gyro[2] * M_PI / 180.0;
      
      //Kalman
      Kalman(x_k_p, P_k_p, p, sigma_alphax2, sigma_gx2);
      Kalman(x_k_q, P_k_q, q, sigma_alphay2, sigma_gy2);
      Kalman(x_k_r, P_k_r, r, sigma_alphaz2, sigma_gz2);
      
      p = x_k_p[0];
      q = x_k_q[0];
      r = x_k_r[0];
      dp_dt = x_k_p[1];
      dq_dt = x_k_q[1];
      dr_dt = x_k_r[1];
      
//      p_temp = p_temp + dp_dt * dt;
//      sprintf(serialprint, ",p_int: %.2f,\tp_filt: %.2f,\tp_meas: %.2f", p_temp*180.0/M_PI, x_k_p[0]*180.0/M_PI, gyro[0]);
//      Serial.println(serialprint);
}



void setup_IMU() {
   // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        delay(20);
        Wire.setClock(400000); //1MHz lol!!
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(1000, true);
    #endif

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
//    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    devStatus = mpu.testConnection();
    
    // make sure it worked (returns true if so)
    if (devStatus == true) {
        // set DLPF to 256 to get minimum delay between readings but no low pass :(
        Serial.println(F("Setting DLPF bandwidth to 188Hz..."));
        mpu.setDLPFMode(MPU6050_DLPF_BW_42);
        // set Rate to 0 for minimum delay between readings
        mpu.setRate(0);
    
        // setting gyro sensitivity
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

        // setting accel sensitivity
        mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(9);
        delay(20);
        mpu.CalibrateGyro(9);
        delay(20);
        mpu.PrintActiveOffsets();

        // enable Arduino interrupt detection
//        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
//        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
//        Serial.println(F(")..."));
//        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
    }
    else {
      /// mpu connection failed, just blink rapidly
        while(1){
            digitalWrite(LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
            delay(500);               // wait for 0.5 second
            digitalWrite(LED_PIN, LOW);    // turn the LED off by making the voltage LOW
            delay(500);               // wait for 0.5 second
        }
    }

    for (int i=0;i<NS; i++)
    {
      mpu.getRotation(&gyro_raw[0], &gyro_raw[1], &gyro_raw[2]);
      data[i] = gyro_raw[0]/131.0; 
    }
    mean_std_dev(NS, data, &mean, &std_dev);
    sprintf(serialprint, ", X_gyro: mean: %.3f, std_dev: %.3f", mean, std_dev);
    Serial.println(serialprint);

    for (int i=0;i<NS; i++)
    {
      mpu.getRotation(&gyro_raw[0], &gyro_raw[1], &gyro_raw[2]);
      data[i] = -gyro_raw[1]/131.0; 
    }
    mean_std_dev(NS, data, &mean, &std_dev);
    sprintf(serialprint, ", Y_gyro: mean: %.3f, std_dev: %.3f", mean, std_dev);
    Serial.println(serialprint);

    for (int i=0;i<NS; i++)
    {
      mpu.getRotation(&gyro_raw[0], &gyro_raw[1], &gyro_raw[2]);
      data[i] = -gyro_raw[2]/131.0; 
    }
    mean_std_dev(NS, data, &mean, &std_dev);
    sprintf(serialprint, ", Z_gyro: mean: %.3f, std_dev: %.3f", mean, std_dev);
    Serial.println(serialprint);
}

void setup_ESCs()
{
    delay(1000);

  // initialize DSHOT
  DSHOT_init( n_esc );

  // ARM ESC( though I think this just selects DSHOT)
  for (int i=0; i<n_esc; i++)
  {
    cmd[i] = 0;
    tlm[i] = 0;
  }
  DSHOT_send( cmd, tlm );
  delayMicroseconds(Dshot_delay);
}

void setup_Receiver()
{
  Serial3.begin(115200); //Uses Serial3 for input as default
}
void read_rec()
{
  rx.getFrame();
  Rec_valid = rx.parseFrame();
  
  if(Rec_valid)
  {
    time = millis();
    //flush serial3
    Serial3.clear();
  }
//
//  Serial.print(rx.getThro());
//  Serial.print("\t");
//  Serial.print(rx.getAile());
//  Serial.print("\t");
//  Serial.print(rx.getElev());
//  Serial.print("\t");
//  Serial.print(rx.getRudd());
//  Serial.print("\t");
//  Serial.print(rx.getGear());
//  Serial.print("\t");
//  Serial.print(rx.getAux1());
//
//  Serial.print('\n');


//  while (Serial3.available()) {
//    byte b = Serial3.read();
//    Serial.println(b);
//  }

//char val[16]; 
//  while (Serial3.available() >= 16) {
//    Serial3.readBytes(val, 16);
//    for(int i = 0; i <= 15; i++)
//    {
//      Serial.print((int)val[i]);
//      Serial.print("  ");
//    }
//    Serial.print("\n");
//}
}

void Kalman1(double x_k[2], double P_k[2][2], double u, double z_k, double sigma_M2)
{
  // special kalman filter to find velocity from measurement of position for 1-DOF system
  
  //predict
    double x_k1[2];
    x_k1[0] = F_KF1[0][0] * x_k[0] + F_KF1[0][1] * x_k[1] + B_KF1[0] * u;
    x_k1[1] = F_KF1[1][0] * x_k[0] + F_KF1[1][1] * x_k[1] + B_KF1[1] * u;

    double P_k1[2][2];
    matmul(F_KF1, P_k, P_k1);
    matmul(P_k1, F_KF1_T, P_k);
//    P_k1[0][0] = P_k[0][0] + QG1_KF1[0][0] * sigma_P2 + QG2_KF1[0][0];
//    P_k1[0][1] = P_k[0][1] + QG1_KF1[0][1] * sigma_P2 + QG2_KF1[0][1];
//    P_k1[1][0] = P_k[1][0] + QG1_KF1[1][0] * sigma_P2 + QG2_KF1[1][0];
//    P_k1[1][1] = P_k[1][1] + QG1_KF1[1][1] * sigma_P2 + QG2_KF1[1][1];

    P_k1[0][0] = P_k[0][0] + Q_KF1[0][0];
    P_k1[0][1] = P_k[0][1] + Q_KF1[0][1];
    P_k1[1][0] = P_k[1][0] + Q_KF1[1][0];
    P_k1[1][1] = P_k[1][1] + Q_KF1[1][1];

    
    //update
    double y_tilda = z_k - x_k1[0];
    double S_k = P_k1[0][0] + sigma_M2;
    double K_KF[2];
    K_KF[0] = P_k1[0][0]/S_k;
    K_KF[1] = P_k1[1][0]/S_k;
    
    x_k[0] = x_k1[0] + K_KF[0] * y_tilda;
//    Serial.println(K_KF[0]);
    x_k[1] = x_k1[1] + K_KF[1] * y_tilda;
    
    P_k[0][0] = (1.0 - K_KF[0])*P_k1[0][0] + 0.0 * P_k1[1][0];  
    P_k[0][1] = (1.0 - K_KF[0])*P_k1[0][1] + 0.0 * P_k1[1][1]; 
    P_k[1][0] = -K_KF[1]*P_k1[0][0] + 1.0 * P_k1[1][0];
    P_k[1][1] = -K_KF[1]*P_k1[0][1] + 1.0 * P_k1[1][1];
}

void Kalman(double x_k[2], double P_k[2][2], double z_k, double sigma_P2, double sigma_M2)
{
  // special kalman filter to find velocity from measurement of position for 1-DOF system
  
  //predict
    double x_k1[2];
    x_k1[0] = F_KF[0][0] * x_k[0] + F_KF[0][1] * x_k[1];
    x_k1[1] = F_KF[1][0] * x_k[0] + F_KF[1][1] * x_k[1] ;

    double P_k1[2][2];
    matmul(F_KF, P_k, P_k1);
    matmul(P_k1, F_KF_T, P_k);
    P_k1[0][0] = P_k[0][0] + Q_KF[0][0] * sigma_P2;
    P_k1[0][1] = P_k[0][1] + Q_KF[0][1] * sigma_P2;
    P_k1[1][0] = P_k[1][0] + Q_KF[1][0] * sigma_P2;
    P_k1[1][1] = P_k[1][1] + Q_KF[1][1] * sigma_P2;
    
    //update
    double y_tilda = z_k - x_k1[0];
    double S_k = P_k1[0][0] + sigma_M2;
    double K_KF[2];
    K_KF[0] = P_k1[0][0]/S_k;
    K_KF[1] = P_k1[1][0]/S_k;
    
    x_k[0] = x_k1[0] + K_KF[0] * y_tilda;
//    Serial.println(K_KF[0]);
    x_k[1] = x_k1[1] + K_KF[1] * y_tilda;
    
    P_k[0][0] = (1.0 - K_KF[0])*P_k1[0][0] + 0.0 * P_k1[1][0];  
    P_k[0][1] = (1.0 - K_KF[0])*P_k1[0][1] + 0.0 * P_k1[1][1]; 
    P_k[1][0] = -K_KF[1]*P_k1[0][0] + 1.0 * P_k1[1][0];
    P_k[1][1] = -K_KF[1]*P_k1[0][1] + 1.0 * P_k1[1][1];
}

void Kalman2(double x_k[2], double P_k[2][2], double z_k, double sigma_P2, double sigma_M2)
{
  // special kalman filter to find position from measurement of velocity for 1-DOF system
  
  //predict
    double x_k1[2];
    x_k1[0] = F_KF[0][0] * x_k[0] + F_KF[0][1] * x_k[1];
    x_k1[1] = F_KF[1][0] * x_k[0] + F_KF[1][1] * x_k[1] ;

    double P_k1[2][2];
    matmul(F_KF, P_k, P_k1);
    matmul(P_k1, F_KF_T, P_k);
    P_k1[0][0] = P_k[0][0] + Q_KF[0][0] * sigma_P2;
    P_k1[0][1] = P_k[0][1] + Q_KF[0][1] * sigma_P2;
    P_k1[1][0] = P_k[1][0] + Q_KF[1][0] * sigma_P2;
    P_k1[1][1] = P_k[1][1] + Q_KF[1][1] * sigma_P2;
    
    //update
    double y_tilda = z_k - x_k1[1];
    double S_k = P_k1[1][1] + sigma_M2;
    double K_KF[2];
    K_KF[0] = P_k1[0][1]/S_k;
    K_KF[1] = P_k1[1][1]/S_k;
    
    x_k[0] = x_k1[0] + K_KF[0] * y_tilda;
//    Serial.println(K_KF[0]);
    x_k[1] = x_k1[1] + K_KF[1] * y_tilda;
    
    P_k[0][0] = (1.0 - 0.0)*P_k1[0][0] + (0.0 - K_KF[0]) * P_k1[1][0];  
    P_k[0][1] = (1.0 - 0.0)*P_k1[0][1] + (0.0 - K_KF[0]) * P_k1[1][1]; 
    P_k[1][0] = (0.0 - 0.0)*P_k1[0][0] + (1.0 - K_KF[1]) * P_k1[1][0];
    P_k[1][1] = (0.0 - 0.0)*P_k1[0][1] + (1.0 - K_KF[1]) * P_k1[1][1];
}

void matmul(double A[2][2], double B[2][2],  double C[2][2])
{
  // A = input matrix (m x p)
  // B = input matrix (p x n)
  // m = number of rows in A
  // p = number of columns in A = number of rows in B
  // n = number of columns in B
  // C = output matrix = A*B (m x n)
  int i, j, k;
  for (i = 0; i < 2; i++)
    for(j = 0; j < 2; j++)
    {
      C[i][j] = 0;
      for (k = 0; k < 2; k++){
        C[i][j] = C[i][j] + A[i][k] * B[k][j];
      }
    }
}

void mean_std_dev(int n, float* data, float *mean, float *std_dev)
{
  float sampleSum = 0;
  for(int i = 0; i < n; i++) {
    sampleSum += data[i];
  }
  
  mean[0] = sampleSum/float(n);

  float sqDevSum = 0.0;
  for(int i = 0; i < n; i++) {
    // pow(x, 2) is x squared.
    sqDevSum += pow((mean[0] - data[i]), 2);
  }


  std_dev[0] = sqrt(sqDevSum/float(n));
}
