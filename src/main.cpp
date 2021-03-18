/*
 * This is a small example design for an arduino based, low power can node
 * It uses an MCP2515 to drive a CAN transceiver like a TJA1051.
 * By using sleep modes, the node will put itself into a low-power
 * sleep mode when no bus traffic is detected and wake up if messages
 * are detected.
 *
 * Please see https://github.com/gfornax/LowPowerCANode for more
 * information.
 * (c) 2021 Andreas Fiessler, fornax@leyanda.de
*/

#include <Arduino.h>
#include <avr/sleep.h>
#include <LowPower.h>
#include <mcp_can.h>

#define CAN0_INT 2 // use an interrupt-capable pin
#define STATUS_LED 10 // Optional: Indicator to see if node is active
#define XCVR_SLEEP 9 // Optional: this goes to the CAN Bus transceiver's sleep enable pin.If you don't need TX, wire the pin to logic high directly.
#define BAT_AIN A0 // battery voltage via voltage divider and filters
#define SPI_CS_PIN 7 // for MCP CAN

// NOTE this is measured including all diodes, filters etc.
// should roughly equal 11.2V using a 220k/100k voltage divider, 10bit ADC and 5V vref
#define LOWVOLT_THRESH 665

// delay after which sleep mode is entered after no CAN message has been received in 100ms
#define SLEEP_DELAY 100

struct doorstate {
  char left_open:1;
  char right_open:1;
  char opened:1;
  char closed:1;
  char override:1;
};

doorstate frontdoor = {0, 0, 0, 0, 0};
doorstate slidingdoor = {0, 0, 0, 0, 0};
doorstate reardoor = {0, 0, 0, 0, 0};

unsigned char flagRecv = 0;
uint8_t sleepcounter = 0;

MCP_CAN CAN(SPI_CS_PIN);

static void ISR_CAN()
{
    flagRecv = 1;
    sleepcounter = 0;
}

void setup_can()
{
  // IMPORTANT: this needs to be configured according to your setup
  while (CAN_OK != CAN.begin(MCP_ANY, CAN_100KBPS, MCP_8MHZ) ) {
    Serial.println("MCP init fail, retry");
    delay(100);
  }
  Serial.println("MCP init ok!");
	
  // set to MCP_NORMAL if you want TX
  CAN.setMode(MCP_LISTENONLY);
	CAN.setSleepWakeup(1); // Enable wake up interrupt when in sleep mode	  

  // set filters to match on all messages 0x3**
  CAN.init_Mask(0, 0, 0x700);
  CAN.init_Mask(1, 0, 0x700);
  // set all filters to the same mode
  for (uint8_t i = 0; i <= 5; i++)
    CAN.init_Filt(i, 0, 0x300); 
	attachInterrupt(digitalPinToInterrupt(CAN0_INT), ISR_CAN, FALLING);
}

void setup() {
  pinMode(STATUS_LED, OUTPUT);
  pinMode(XCVR_SLEEP, OUTPUT);
  pinMode(BAT_AIN, INPUT);

  Serial.begin(115200);
  Serial.println(F("setup"));
  digitalWrite(STATUS_LED, HIGH);
  digitalWrite(XCVR_SLEEP, LOW);
  pinMode(CAN0_INT, INPUT);

  setup_can();
}

void go_sleep(void)
{
  Serial.println("prep for sleep...");
  // disable all outputs
  digitalWrite(STATUS_LED, LOW);

  // set transceiver to recessive/sleep mode
  digitalWrite(XCVR_SLEEP, HIGH);
 
  CAN.setMode(MCP_SLEEP);
  Serial.println("going to sleep");
  Serial.flush();
  // wake up every 8S to check for any missed interrupt -> leave sleep mode
  // also, if voltage is low keep sleeping whatsoever
  do{
    cli();
    if(digitalRead(CAN0_INT)) // last check for messages before going to sleep
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    sei();
    analogRead(BAT_AIN); // dummy readout after powering ADC
  } while (analogRead(BAT_AIN) <= LOWVOLT_THRESH || digitalRead(CAN0_INT));

  Serial.println("leave sleep");

  // enable whatever is needed after wake-up
  digitalWrite(STATUS_LED, HIGH);
  digitalWrite(XCVR_SLEEP, LOW);

  // MCP should be in LISTENONLY after wakeup anyway. Set to NORMAL if TX is needed
	CAN.setMode(MCP_LISTENONLY); 
}

void every_100ms(void)
{
  sleepcounter++;
  if (sleepcounter == SLEEP_DELAY)
    go_sleep();

  // better: use a moving AVG with >=8 taps for this
  if (analogRead(BAT_AIN) <= LOWVOLT_THRESH)
    go_sleep();
}

void parse_can_message(unsigned long rxId, byte *rxBuf)
{
  // Note: this is an example state change detection for a VW T5
  if (rxId == 0x371){
    Serial.println("\ngot door status");
    // detect state change of each door
    if (rxBuf[0] & 0b1000){ //sliding door right
      if (slidingdoor.right_open == 0)
        slidingdoor.opened = 1;
      slidingdoor.right_open = 1;
    } else {
      if (slidingdoor.right_open == 1)
        slidingdoor.closed = 1;
      slidingdoor.right_open = 0;
    }
    if (rxBuf[0] & 0b100){ //sliding door left
      if (slidingdoor.left_open == 0)
        slidingdoor.opened = 1;
      slidingdoor.left_open = 1;
    } else {
      if (slidingdoor.left_open == 1)
        slidingdoor.closed = 1;
      slidingdoor.left_open = 0;
    }

    if (rxBuf[0] & 0b10){ //front door right
      if (frontdoor.right_open == 0)
        frontdoor.opened = 1;
      frontdoor.right_open = 1;
    } else {
      if (frontdoor.right_open == 1)
        frontdoor.closed = 1;
      frontdoor.right_open = 0;
    }
    if (rxBuf[0] & 0b1){ //front door left
      if (frontdoor.left_open == 0)
        frontdoor.opened = 1;
      frontdoor.left_open = 1;
    } else {
      if (frontdoor.left_open == 1)
        frontdoor.closed = 1; 
      frontdoor.left_open = 0;
    }

    if (rxBuf[1] & 0b1){ //rear (note: 0b11 for r/l, but we only have one door)
      if (reardoor.right_open == 0)
        reardoor.opened = 1;
      reardoor.right_open = 1;
    } else {
      if (reardoor.right_open == 1)
        reardoor.closed = 1;
      reardoor.right_open = 0;
    }

    // evaluate state changes and do something
    if (frontdoor.opened == 1){
      frontdoor.opened = 0;
      frontdoor.override = 0;
      // this is executed if any of the front doors was opened
    } else if (frontdoor.closed == 1) {
      frontdoor.closed = 0;
      // this is executed if any of the front doors was closed
    }
    if (slidingdoor.opened == 1){
      slidingdoor.opened = 0;
      slidingdoor.override = 0;
      // same for the sliding doors
    } else if (slidingdoor.closed == 1) {
      slidingdoor.closed = 0;
      // you get the picture
    }

    if (reardoor.opened == 1){
      reardoor.opened = 0;
      reardoor.override = 0;
    } else if (reardoor.closed == 1) {
      reardoor.closed = 0;
    }                               
  }
}

void loop() {
	static unsigned long lastRefreshTime100 = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - lastRefreshTime100 >= 100) { lastRefreshTime100 += 100; every_100ms(); }

  if(!digitalRead(CAN0_INT))
	{
		unsigned long rxId;
		byte len;
		byte rxBuf[16];
  
    sleepcounter = 0; //reset the sleep counter for any message

    // NOTE: This was mostly taken from another example.
    // I'll leave it here for debugging. Remove it if everything works, just keep the parse_can_message() call

	  if(CAN.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK) // Read data: len = data length, buf = data byte(s)
    {
			char msgString[128]; // Array to store serial string
			if(rxId & CAN_IS_EXTENDED) // Determine if ID is standard (11 bits) or extended (29 bits)
				sprintf_P(msgString, PSTR("Ext ID: 0x%.8lX  DLC: %1d  Data:"), (rxId & CAN_EXTENDED_ID), len);
			else
				sprintf_P(msgString, PSTR("Std ID: 0x%.3lX       DLC: %1d  Data:"), rxId, len);

			Serial.print(msgString);

			if(rxId & CAN_IS_REMOTE_REQUEST) // Determine if message is a remote request frame.
				Serial.print(F(" REMOTE REQUEST FRAME"));
			else
			{
				for(byte i=0;i<len;i++)	{
					sprintf_P(msgString, PSTR(" 0x%.2X"), rxBuf[i]);
					Serial.print(msgString);
				}
        parse_can_message(rxId, rxBuf);

      }
			Serial.println();
		}
		else
			Serial.println(F("IRQ, but no message"));
	}
}
