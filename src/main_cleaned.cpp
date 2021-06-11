#include <Arduino.h>
#include "controller_structs.h"
#include "CircularBuffer.h"

#define TELEMETRY_SIZE 44
#define COMMAND_SIZE 8
#define PACKET_SIZE 50
#define LIDAR_SAMPLE_SIZE 31


float ReverseFloat(const float inFloat);

void ReverseArr(uint8_t arr[], int start, int end);

uint8_t calc_checksum(void *data, uint8_t len);

bool ReadPacket();

void SendPacket();

void InterpretLidar();

Packet tx_packet;  // store packet to be sent
Telemetry rx_data; // store received data

CircularBuffer<Telemetry, 5>

void setup()
{ // put your setup code here, to run once:

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(38400);
  Serial.setTimeout(5000); // give up waiting if nothing can be read in 2s

  // init tx packet
  tx_packet.start_seq = 0x0210;
  tx_packet.end_seq = 0x0310;

  //init rx packet 
  tx_packet.len = COMMAND_SIZE;

  while (!Serial)
  {
    // wait until Serial is ready
  }
}

void loop()
{ // put your main code here, to run repeatedly:
  if (ReadPacket())
  {
    //Interpret Telemetry data
    InterpretLidar();
    // valid packet received, pack new data in new packet and send it out
    // print it out in many formats:
    SendPacket();
  }
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

uint8_t calc_checksum(void *data, uint8_t len)
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

  char tmp[PACKET_SIZE];

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

    if (Serial.readBytes((uint8_t *)&rx_data, payload_length) != payload_length)
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

  if (calc_checksum(&rx_data, payload_length) != checksum)
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

  // Yeah! a valid packet is received
  rx_data.timestamp = __builtin_bswap16(rx_data.timestamp);
  rx_data.recovery_x_error = __builtin_bswap16(rx_data.recovery_x_error);
  rx_data.wind_vector_x = ReverseFloat(rx_data.wind_vector_x);
  rx_data.wind_vector_y = ReverseFloat(rx_data.wind_vector_y);

  ReverseArr(rx_data.lidar_samples, 0, LIDAR_SAMPLE_SIZE - 1);

  return true;
}

void SendPacket()
{
  tx_packet.checksum = calc_checksum(&tx_packet.tx_data, tx_packet.len); //produce new checksum

  Serial.write((char *)&tx_packet, sizeof(tx_packet)); // send the packet
}

void InterpretLidar()
{

}