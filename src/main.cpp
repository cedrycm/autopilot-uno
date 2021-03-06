#include <Arduino.h>
#include <math.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <Servo.h>

#include "main_header.h"
#include "RoTV.h"

//packet size definitions---------------------------------------------------------------
#define TELEMETRY_SIZE 44
#define COMMAND_SIZE 8
#define PACKET_SIZE 50
#define LIDAR_SAMPLE_SIZE 31

//servo definitions---------------------------------------------------------------------
#define TRIM_DURATION 2                                          // compensation ticks to trim adjust for digitalWrite delays
#define usToTicks(_us) ((clockCyclesPerMicrosecond() * _us) / 8) // converts microseconds to tick (assumes prescale of 8)
#define MIN_PULSE_WIDTH 544                                      // the shortest pulse sent to a servo in microseconds
#define MAX_PULSE_WIDTH 2400                                     // the longest pulse sent to a servo in microseconds
#define DEFAULT_PULSE_WIDTH 1500                                 // default pulse width when servo is attached
#define REFRESH_INTERVAL 20000                                   // minimum time to refresh servos in microseconds

//world measurement definitions---------------------------------------------------------
#define TREE_RADIUS 3.0

#define DELIVERY_SITE_RADIUS 5.0

#define DELIVERY_SITE_LIDAR_DIAMETER 1.05 //accounting for precision error

#define MAX_LIDAR_DISTANCE 255
#define MAX_LIDAR_ANGLE 15.0

#define VEHICLE_AIRSPEED_X 30.0

#define DELIVERY_ZONE 250 //recovery zone

#define VEHICLE_AVOID_THRESHOLD 55.0 //max distance to engage collision avoidance
#define VEHICLE_DROP_THRESHOLD 40.0  //min distance to try for drop

#define ABS_AVOIDANCE_ANGLE 3.9 //max angle range +/- 5deg to check for collision
#define PACKAGE_DROP_TIME 0.5   //time for package to reach the ground
#define DROP_TIMEOUT 750        //500ms before dropping

#define degreesToRadians(_angleDegrees) (_angleDegrees * PI / 180.0)
#define radiansToDegrees(_angleRadians) (_angleRadians * 180.0 / PI)

//Buzzer variables---------------------------------------------------------------
static const uint8_t tonePin = 4; //set pwm output to digital pin 4
uint16_t note_index = 0;          //index of note in the track
int16_t duration_elapsed = 0;     //start of new note
bool zip_launched = false;
//Servo variables----------------------------------------------------------------
bool moveRotor = false;
static const uint8_t servoPin = 9; //set pwm output to digital pin 9
unsigned int servoTicks;
uint8_t oldSREG;

Servo myServo;
//---------------------------------------=----------------------------------------
Packet tx_packet; // store packet to be sent
//Telemetry rx_data; // store received data

CircularBuffer<Telemetry *, 10> telem_buffer;

Telemetry telemetry_array[10];
uint8_t avg_samples[LIDAR_SAMPLE_SIZE];

Group groups[LIDAR_SAMPLE_SIZE];
size_t num_groups = 0;

Measurements target_object;

Airspeed zip_speed;

CtrlFlags statusFlag;

int16_t drop_timestamp = 0;

int telem_count = 0;
int avoid_count = 0;

void SetServo(int value)
{
  /* timer1 function
  // value = map(value, -30, 30, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

  // value = value - TRIM_DURATION;
  // value = usToTicks(value);

  // oldSREG = SREG;

  // moveRotor = true;
  // servoTicks = value;

  // TIMSK0 &= ~_BV(OCIE0A);  // disable timer 0 output compare interrupt
  // TIMSK1 |= (1 << OCIE1A); // enable timer 1 compare interrupt */

  //servo lib function
  int angle = map(value, -30.0, 30.0, 0, 180); //map angle propotional to rotor angle

  myServo.write(angle);
}

ISR(TIMER0_COMPA_vect)
{ //timer0 interrupt 768Hz plays notes for Ride of the Valkyries
  //generates pwm at frequency defined by MIDI of file for "Ride of the Valkyries"
  //freq = (96 bpm * 480 ppq)/60 s
  if (zip_launched)
  {
    if (duration_elapsed < 0 && note_index < Melody_Length)
    {
      note_index++;
      uint16_t data = pgm_read_word((uint16_t *)&Melody[note_index]);
      duration_elapsed = data >> 8;

      if ((data & 0xF) == 0xF)
      {
        noTone(tonePin);
        duration_elapsed--;
      }
      else
      {
        uint16_t Freq = pgm_read_word(&Freq8[data & 0xF]) / (1 << (8 - (data >> 4 & 0xF)));
        tone(tonePin, Freq);
        duration_elapsed--;
      }
    }
    else if (note_index == Melody_Length)
    {
      note_index = 0;
    }
    else
    {
      duration_elapsed--;
    }
  }
}

//SERVO INTERRUPT
//#TODO: having issues with timer1 interrupt implementation raising too often and preventing tune from playing 
// ISR(TIMER1_COMPA_vect)
// {
//   if (moveRotor)
//   {
//     digitalWrite(servoPin, LOW); //pulse low if refreshing
//     OCR1A = TCNT1 + servoTicks; //set timer count flag
//     digitalWrite(servoPin, HIGH);
//   }
//   else if (servoTicks > 0)
//   {
//     if (((unsigned)TCNT1) + 4 < usToTicks(REFRESH_INTERVAL)) // allow a few ticks to ensure the next OCR1A not missed
//       OCR1A = (unsigned int)usToTicks(REFRESH_INTERVAL);
//     else
//       OCR1A = TCNT1 + 4; // at least REFRESH_INTERVAL has elapsed
//     servoTicks = 0;
//     moveRotor = false;
//     TCNT1 = 0;
//     SREG = oldSREG; //reset timer interrupts
//   }
// }

void setup()
{       
   // put your setup code here, to run once:
  cli(); //stop interrupts

  //set timer0 interrupt at 960Hz
  TCCR0A = 0; // set entire TCCR0A register to 0
  TCCR0B = 0; // same for TCCR0B
  TCNT0 = 0;  //initialize counter value to 0

  // set compare match register for 768hz increments
  OCR0A = 64; // = (16*10^6) / (960*256) - 1 (must be <256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS02 bit for 256 prescaler
  TCCR0B |= (1 << CS02);
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);

  // //set timer1 interrupt at 1Hz
  // TCCR1A = 0; // set entire TCCR1A register to 0
  // TCCR1B = 0; // same for TCCR1B
  // TCNT1 = 0;  //initialize counter value to 0

  // // turn on CTC mode
  // TCCR1B |= (1 << WGM12);
  // // Set CS11 bit for 8 prescaler
  // TCCR1B |= (1 << CS11);
  // // enable timer compare interrupt
  // TIMSK1 |= (1 << OCIE1A);
  sei();

  // SetServo(0);
  myServo.attach(9);
  myServo.write(90);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(38400);
  Serial.setTimeout(5000); // give up waiting if nothing can be read in 2s

  // init tx packet
  tx_packet.start_seq = 0x0210;
  tx_packet.end_seq = 0x0310;

  //init rx packet
  tx_packet.len = COMMAND_SIZE;

  statusFlag = CtrlFlags::SEARCH_DELIVERY_SITE;

  target_object.delivery_distance = MAX_LIDAR_DISTANCE;
  target_object.obstacle_distance = MAX_LIDAR_DISTANCE;

  zip_speed.v_x = 30.0;

  while (!Serial)
  {
    // wait until Serial is ready
    zip_launched = false;
  }
    
}

void loop()
{ // put your main code here, to run repeatedly:

  if (ReadPacket())
  {
    zip_launched= true;
    telem_buffer.push(&telemetry_array[telem_count]); //add to bvuffer

    if (statusFlag != CtrlFlags::RECOVER) //if recovery has started no need to rely on lidar data
    {
      //Interpret Telemetry data
      InterpretLidar();

      GetClosestObject();
    }

    InterpretData();

    SendPacket();
  }

  telem_count--;
  telem_count = (telem_count + 10) % 10;
}

float ReverseFloat(const float inFloat)
{
  float retVal;
  char *floatToConvert = (char *)&inFloat;
  char *returnFloat = (char *)&retVal;

  // swap the bytes into a temporary buffer
  returnFloat[0] = floatToConvert[3];
  returnFloat[1] = floatToConvert[2];
  returnFloat[2] = floatToConvert[1];
  returnFloat[3] = floatToConvert[0];

  return retVal;
}

void ReverseArr(uint8_t arr[], int start, int end)
{
  uint8_t temp;
  while (start < end)
  {
    temp = arr[start];
    arr[start] = arr[end];
    arr[end] = temp;
    start++;
    end--;
  }
}

uint8_t CalcChecksum(void *data, uint8_t len)
{
  uint8_t checksum = 0;
  uint8_t *addr;
  for (addr = (uint8_t *)data; addr < (data + len); addr++)
  {
    // xor all the bytes
    checksum ^= *addr; // checksum = checksum xor value stored in addr
  }
  return checksum;
}

bool ReadPacket()
{

  uint8_t payload_length, checksum;
  while (Serial.available() < PACKET_SIZE)
  {
    // not enough bytes to read
  }

  if (Serial.read() != 0x10)
  {
    // first byte not DLE, not a valid packet

    return false;
  }

  // first byte is DLE, read next byte
  if (Serial.read() != 0x02)
  {
    // second byte not STX, not a valid packet
    return false;
  }

  // seems to be a valid packet
  payload_length = Serial.read(); // get length of payload

  // can compare payload length or extra packet type byte to decide where to write received data to
  if (payload_length == TELEMETRY_SIZE)
  {
    if (Serial.readBytes((char *)&telemetry_array[telem_count], payload_length) != payload_length)
    {
      // cannot receive required length within timeout

      return false;
    }
  }
  else
  {
    // invalid data length
    return false;
  }

  checksum = Serial.read();

  if (CalcChecksum((char *)&telemetry_array[telem_count], payload_length) != checksum)
  {

    // checksum error
    return false;
  }

  if (Serial.read() != 0x10)
  {
    // last 2nd byte not DLE, not a valid packet
    return false;
  }

  // last 2nd byte is DLE, read next byte
  if (Serial.read() != 0x03)
  {
    // last byte not ETX, not a valid packet
    return false;
  }
  //Perform endian conversion
  telemetry_array[telem_count].timestamp = __builtin_bswap16(telemetry_array[telem_count].timestamp);
  telemetry_array[telem_count].recovery_x_error = __builtin_bswap16(telemetry_array[telem_count].recovery_x_error);
  telemetry_array[telem_count].wind_vector_x = ReverseFloat(telemetry_array[telem_count].wind_vector_x);
  telemetry_array[telem_count].wind_vector_y = ReverseFloat(telemetry_array[telem_count].wind_vector_y);

  ReverseArr(telemetry_array[telem_count].lidar_samples, 0, LIDAR_SAMPLE_SIZE - 1);
  // Yeah! a valid packet is received

  return true;
}

void SendPacket()
{
  tx_packet.len = sizeof(tx_packet.tx_data);
  tx_packet.checksum = CalcChecksum(&tx_packet.tx_data, tx_packet.len); //produce new checksum

  Serial.write((char *)&tx_packet, sizeof(tx_packet)); // send the packet
}

void InterpretLidar()
{
  int last = 0;
  Telemetry *telem = telem_buffer.last();

  for (int j = 0; j < 31; j++) //iterate through list elements
  {
    avg_samples[j] += telem->lidar_samples[j];

    if (j == 0) //if first entry in list - create first group entry
    {
      num_groups++;

      groups[num_groups - 1].start_index = j;
      groups[num_groups - 1].end_index = j;

      last = j;
    }
    else if ((avg_samples[j] != 0 && avg_samples[last] != 0) && (abs(avg_samples[j] - avg_samples[last]) <= TREE_RADIUS)) //extend object to adjacent index
    {
      groups[num_groups - 1].end_index = j;
      last = j;
    }
    else if (avg_samples[j] == 0 && avg_samples[last] == 0) //if adjacent zeros - group together
    {
      groups[num_groups - 1].end_index = j;
      last = j;
    }
    else //create next entry
    {
      num_groups++;

      groups[num_groups - 1].start_index = j;
      groups[num_groups - 1].end_index = j;

      last = j;
    }
  }
}

void GetClosestObject() //Fills euclidian measurements for objects in the lidar samples
{
  Measurements current = {0};
  float closest_obstacle = MAX_LIDAR_DISTANCE;
  float closest_target = MAX_LIDAR_DISTANCE;
  float closest_target_angle = MAX_LIDAR_ANGLE;
  float closest_obstacle_angle = MAX_LIDAR_ANGLE;

  for (int i = 0; (unsigned)i < num_groups; i++)
  {
    int idx_first = groups[i].start_index;
    int idx_last = groups[i].end_index;

    if (avg_samples[idx_first] != 0 && avg_samples[idx_last] != 0) //if a sample is non-zero
    {
      if (idx_first != idx_last)
      {
        //Get measurements for object group
        EuclidValues(&current, idx_first, idx_last);
        int d_height = abs(avg_samples[idx_first] - avg_samples[idx_last]);

        if ((current.obstacle_diameter <= DELIVERY_SITE_LIDAR_DIAMETER && d_height <= DELIVERY_SITE_LIDAR_DIAMETER / 2.0) && (current.obstacle_distance < closest_target && current.theta_mid < closest_target_angle))
        {
          //assign object as a target
          closest_target = current.obstacle_distance;

          target_object.delivery_distance = current.obstacle_distance;
          target_object.delivery_theta = current.theta_mid;
        }
        else if ((current.obstacle_diameter > DELIVERY_SITE_LIDAR_DIAMETER || d_height > DELIVERY_SITE_LIDAR_DIAMETER / 2.0) && (current.obstacle_distance < closest_obstacle && current.theta_mid < closest_obstacle_angle))
        {
          //assign object as obstacle
          closest_obstacle = current.obstacle_distance;

          target_object.obstacle_distance = current.obstacle_distance;
          target_object.obstacle_diameter = current.obstacle_diameter;
          target_object.theta_edge1 = current.theta_edge1;
          target_object.theta_edge2 = current.theta_edge2;
          target_object.theta_mid = current.theta_mid;
        }
      }
      else
      {
        float theta = (-1) * (idx_first) + 15.0;
        if (avg_samples[idx_first] < closest_target && theta < closest_target_angle)
        {
          //assign as target
          closest_target = avg_samples[idx_first];
          closest_target_angle = theta;

          target_object.delivery_distance = avg_samples[idx_first];
          target_object.delivery_theta = theta;
        }
      }
    }
  }
}

void EuclidValues(Measurements *object, int idx_first, int idx_last) //Compute euclidian values for objects
{
  float d_1, d_2, x_1, x_2, y_1, y_2, m_x, m_y;

  int theta_edge1 = (-1) * idx_first + 15;
  int theta_edge2 = (-1) * idx_last + 15;

  d_1 = avg_samples[idx_first];
  d_2 = avg_samples[idx_last];

  x_1 = d_1 * cos(degreesToRadians(theta_edge1));
  y_1 = d_1 * sin(degreesToRadians(theta_edge1));

  x_2 = d_2 * cos(degreesToRadians(theta_edge2));
  y_2 = d_2 * sin(degreesToRadians(theta_edge2));

  m_x = (x_1 + x_2) / 2.0; //proximal distance
  m_y = (y_1 + y_2) / 2.0; //lateral distance

  object->obstacle_diameter = sqrt((x_1 - x_2) * (x_1 - x_2) + (y_1 - y_2) * (y_1 - y_2));
  object->obstacle_distance = sqrt(m_x * m_x + m_y * m_y);
  object->theta_edge1 = theta_edge1;
  object->theta_edge2 = theta_edge2;
  object->theta_mid = radiansToDegrees(atan(m_y / m_x));
}

void InterpretData() //Updates airspeed and command telemetry according to control status flag
{
  Telemetry *last_telem = telem_buffer.last(); //retrieve last data packet stored to buffer

  if (last_telem->recovery_x_error < DELIVERY_ZONE || statusFlag == CtrlFlags::RECOVER) //update lateral velocity to head into recovery site
  {
    statusFlag = CtrlFlags::RECOVER;
    UpdateAirspeed(&zip_speed, last_telem->wind_vector_x, last_telem->wind_vector_y,
                   last_telem->recovery_x_error, last_telem->recovery_y_error);

    tx_packet.tx_data.lateral_airspeed = zip_speed.v_y;
    tx_packet.tx_data.drop_package = 0;
  }
  else if (target_object.obstacle_distance < VEHICLE_AVOID_THRESHOLD && abs(target_object.theta_mid) < ABS_AVOIDANCE_ANGLE) //update lateraL velocity to avoid object
  {
    statusFlag = CtrlFlags::AVOID_COLLISION;

    UpdateAvoidAirspeed(&zip_speed, last_telem->wind_vector_x, last_telem->wind_vector_y,
                        target_object.theta_edge1, target_object.theta_edge2);

    tx_packet.tx_data.lateral_airspeed = zip_speed.v_y;
    tx_packet.tx_data.drop_package = 0;
  }
  else if (statusFlag == CtrlFlags::AVOID_COLLISION && avoid_count < 4) // if last flag was to avoid
  {
    //continue to clear away from trees for 5 loops (1/12th s) for clearance
    if (avoid_count > 4)
    {
      avoid_count = 0;
      statusFlag = CtrlFlags::SEARCH_DELIVERY_SITE;
    }

    tx_packet.tx_data.lateral_airspeed = zip_speed.v_y + (-1) * last_telem->wind_vector_y;
    tx_packet.tx_data.drop_package = 0;

    avoid_count++;
  }
  else if (target_object.delivery_distance != 0 && statusFlag != CtrlFlags::AVOID_COLLISION)
  {
    UpdateAirspeed(&zip_speed, last_telem->wind_vector_x, last_telem->wind_vector_y,
                   target_object.delivery_theta);

    statusFlag = (target_object.delivery_distance <= VEHICLE_DROP_THRESHOLD && abs(target_object.delivery_theta) < 14.0) ? CtrlFlags::DELIVER_PACKAGE : CtrlFlags::APPROACH_DELIVERY_SITE; //Approach Zip Delivery Site

    if (statusFlag == CtrlFlags::DELIVER_PACKAGE)
    {
      float v_x_sum = zip_speed.v_x + last_telem->wind_vector_x;
      float v_y_sum = zip_speed.v_y + last_telem->wind_vector_y;
      tx_packet.tx_data.drop_package = CheckForTarget(v_x_sum, v_y_sum, target_object.delivery_theta, target_object.delivery_distance, last_telem->timestamp);
    }
    else
    {
      zip_speed.v_y = zip_speed.v_y * 1.5;
      tx_packet.tx_data.drop_package = 0;
    }

    tx_packet.tx_data.lateral_airspeed = zip_speed.v_y;
  }
  else
  {
    UpdateAirspeed(&zip_speed, last_telem->wind_vector_x, last_telem->wind_vector_y);
    statusFlag = CtrlFlags::SEARCH_DELIVERY_SITE;

    tx_packet.tx_data.lateral_airspeed = zip_speed.v_y;
    tx_packet.tx_data.drop_package = 0;
  }

  if(telem_count > 4) //update every 1/30th of a second to avoid jitteryness 
    SetServo(int(zip_speed.v_y));

  ClearGroups();
}

void UpdateAirspeed(Airspeed *zip_speed, float v_x_wind, float v_y_wind, float theta) //Assign lateral airspeed to zip_speed.v_y given object angle from vehicle
{

  zip_speed->v_y = (((zip_speed->v_x + v_x_wind) * atan(degreesToRadians(theta))) - v_y_wind);
}

void UpdateAvoidAirspeed(Airspeed *zip_speed, float v_x_wind, float v_y_wind, float theta1, float theta2) //Assign lateral airspeed to zip_speed.v_y given target distance vectors. Optional bool input theta range to avoid.
{
  theta1 = theta1 > 0 ? ++theta1 : --theta1;
  theta2 = theta2 > 0 ? ++theta2 : --theta2;

  float multiplier = 1.25;

  float v_y_1 = (((zip_speed->v_x + v_x_wind) * atan(degreesToRadians(theta1))) - v_y_wind);
  float v_y_2 = (((zip_speed->v_x + v_x_wind) * atan(degreesToRadians(theta2))) - v_y_wind);

  if (abs(theta1) < abs(theta2))
  {
    zip_speed->v_y = v_y_1 * multiplier;
  }
  else
  {
    zip_speed->v_y = v_y_2 * multiplier;
  }
}

void UpdateAirspeed(Airspeed *zip_speed, float v_x_wind, float v_y_wind, int16_t d_x, int8_t d_y)
{
  float theta = d_y != 0 ? atan(d_y / d_x) : 0;

  zip_speed->v_y = ((zip_speed->v_x + v_x_wind) * atan(theta)) - v_y_wind;
}

void UpdateAirspeed(Airspeed *zip_speed, float v_x_wind, float v_y_wind) //return lateral airspeed to go counter wind
{

  zip_speed->v_y = (-1) * v_y_wind;
}

void ClearGroups() //Needed to clear groups for next iteration
{
  num_groups = 0;

  for (int i = 0; i < 31; i++)
  {
    avg_samples[i] = 0;

    groups[i].start_index = 0;
    groups[i].end_index = 0;
  }
}

int CheckForTarget(float v_x_sum, float v_y_sum, float theta, float distance, int16_t current_time) //Returns 1 if calculated package fall lands object in delivery zone - 0 if it doesn't
{
  int16_t time_elapsed = current_time - drop_timestamp;
  float target_x = distance * cos(degreesToRadians(theta));
  float target_y = distance * sin(degreesToRadians(theta));
  float position_x = v_x_sum * PACKAGE_DROP_TIME;
  float position_y = v_y_sum * PACKAGE_DROP_TIME;
  bool in_range = Contains(target_x, target_y, position_x, position_y);

  if (drop_timestamp != 0 && time_elapsed <= DROP_TIMEOUT)
  {
    return 0;
  }
  else if (in_range)
  {

    drop_timestamp = current_time;
    return 1;
  }
  else
  {
    drop_timestamp = 0;
    return 0;
  }
}

bool Contains(float target_x, float target_y, float position_x, float position_y)
{
  float delta_x = abs(target_x - position_x);
  float delta_y = abs(target_y - position_y);
  float delta_p = (delta_x * delta_x) + (delta_y * delta_y);
  float d_2 = (DELIVERY_SITE_RADIUS - 0.5) * (DELIVERY_SITE_RADIUS - 0.5);

  return (delta_p < d_2);
}