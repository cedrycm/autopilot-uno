#include <Arduino.h>
#include "autopilot_controller.h"

typedef struct
{
  // send first
  uint16_t start_seq; // 0x0210, 0x10 will be sent first
  uint8_t len;        // length of payload
  Command tx_data;
  uint8_t checksum;
  uint16_t end_seq; // 0x0310, 0x10 will be sent first
  // send last
} Packet;

Packet tx_packet;  // store packet to be sent
Telemetry rx_data; // store received data
AutoPilot pilot;

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

bool readPacket()
{
  uint8_t payload_length, checksum;
  while (Serial.available() < 50)
  {
    // not enough bytes to read
  }
  char tmp[50];

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
  if (payload_length == 44)
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

  return true;
}

void send_packet()
{
  tx_packet.len = sizeof(tx_packet.tx_data);
  // tx_packet.tx_data.drop_package = 1;
  // tx_packet.tx_data.lateral_airspeed = (float)12.0;
  tx_packet.checksum = calc_checksum(&tx_packet.tx_data, tx_packet.len);

  // tx_test.len = sizeof(rx_data);
  // tx_test.tx_data = rx_data;
  // tx_test.checksum = calc_checksum(&tx_test.tx_data, tx_packet.len);

  // Serial.write((char *)&tx_test, sizeof(tx_test));

  Serial.write((char *)&tx_packet, sizeof(tx_packet)); // send the packet
}

void setup()
{ // put your setup code here, to run once:

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(38400);
  Serial.setTimeout(5000); // give up waiting if nothing can be read in 2s

  // init tx packet
  tx_packet.start_seq = 0x0210;
  tx_packet.end_seq = 0x0310;

  while (!Serial)
  {
    // wait until Serial is ready
  }
}

void loop()
{ // put your main code here, to run repeatedly:
  if (readPacket())
  {
    //Interpret Telemetry data
    pilot.send_receive_data(&rx_data, &(tx_packet.tx_data));
    // valid packet received, pack new data in new packet and send it out
    // print it out in many formats:
    Serial.println(rx_data.timestamp); // print as an ASCII-encoded decimal - same as "DEC"
    Serial.println("\t");              // prints a tab

    Serial.println(rx_data.wind_vector_x, 1); // print as an ASCII-encoded decimal
    Serial.println("\t");                     // prints a tab

    Serial.println(rx_data.wind_vector_y, 1); // print as an ASCII-encoded decimal
    Serial.println("\t");                     // prints a tab

    Serial.println(tx_packet.tx_data.drop_package); // print as an ASCII-encoded decimal - same as "DEC"
    Serial.println("\t");                           // prints a tab

    Serial.println(tx_packet.tx_data.lateral_airspeed, 1); // print as an ASCII-encoded decimal
    Serial.println("\t");                                  // prints a tab

    send_packet();
  }
}