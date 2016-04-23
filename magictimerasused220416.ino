#include <math.h>

void TimersStart() {

        uint32_t config = 0;

  // Set up the power management controller for TC0 and TC2

        pmc_set_writeprotect(false);    // Enable write access to power management chip
        pmc_enable_periph_clk(ID_TC0);  // Turn on power for timer block 0 channel 0
        pmc_enable_periph_clk(ID_TC6);  // Turn on power for timer block 2 channel 0

  // Timer block zero channel zero is connected only to the PPS 
  // We set it up to load regester RA on each PPS and reset
  // So RA will contain the number of clock ticks between two PPS, this
  // value should be very stable +/- one tick

        config = TC_CMR_TCCLKS_TIMER_CLOCK1 |        // Select fast clock MCK/2 = 42 MHz
                 TC_CMR_ETRGEDG_RISING |             // External trigger rising edge on TIOA0
                 TC_CMR_ABETRG |                     // Use the TIOA external input line
                 TC_CMR_LDRA_RISING;                 // Latch counter value into RA

        TC_Configure(TC0, 0, config);                // Configure channel 0 of TC0
        TC_Start(TC0, 0);                            // Start timer running

        TC0->TC_CHANNEL[0].TC_IER =  TC_IER_LDRAS;   // Enable the load AR channel 0 interrupt each PPS
        TC0->TC_CHANNEL[0].TC_IDR = ~TC_IER_LDRAS;   // and disable the rest of the interrupt sources
        NVIC_EnableIRQ(TC0_IRQn);                    // Enable interrupt handler for channel 0

  // Timer block 2 channel zero is connected to the OR of the PPS and the RAY event
 
        config = TC_CMR_TCCLKS_TIMER_CLOCK1 |        // Select fast clock MCK/2 = 42 MHz
                 TC_CMR_ETRGEDG_RISING |             // External trigger rising edge on TIOA1
                 TC_CMR_ABETRG |                     // Use the TIOA external input line
                 TC_CMR_LDRA_RISING;                 // Latch counter value into RA
  
  TC_Configure(TC2, 0, config);                // Configure channel 0 of TC2
  TC_Start(TC2, 0);          // Start timer running
 
  TC2->TC_CHANNEL[0].TC_IER =  TC_IER_LDRAS;   // Enable the load AR channel 0 interrupt each PPS
  TC2->TC_CHANNEL[0].TC_IDR = ~TC_IER_LDRAS;   // and disable the rest of the interrupt sources
  NVIC_EnableIRQ(TC6_IRQn);                    // Enable interrupt handler for channel 0

  // Set up the PIO controller to route input pins for TC0 and TC2

  PIO_Configure(PIOC,PIO_INPUT,
          PIO_PB25B_TIOA0,  // D2 Input 
          PIO_DEFAULT);

  PIO_Configure(PIOC,PIO_INPUT,
          PIO_PC25B_TIOA6,  // D5 Input
          PIO_DEFAULT);
}

// Define some output debug pins to monitor whats going on via my scope

#define PPS_PIN 13    // PPS (Pulse Per Second) and LED
#define EVT_PIN 12    // Cosmic ray event detected
#define FLG_PIN 11    // Debug flag

static uint32_t displ = 1;  // Display values in loop

static uint32_t ppsfl = LOW,  // PPS Flag boolean
    rega0 = 0,  // RA reg
    stsr0 = 0,  // Interrupt status register
    ppcnt = 0;  // PPS count


static uint32_t event = 0,  // Time of the last event (This should be a queue !)
    rega1 = 0, 
    stsr1 = 0;

void TC0_Handler() {

  // This ISR is connected only to the PPS (Pulse Per Second) GPS event
  // Each time this runs, set the flag to tell the TC6 ISR we have seen it
  // This logic only works if the TC0 handler gets called before the TC6 handler
  // hence the debug flag which I look at with a scope to be sure.
  // I may introduce a small delay line to ensure this is true, so far it is.

  //ppsfl = HIGH;       // Seen a rising edge on the PPS
  //digitalWrite(PPS_PIN,HIGH);   // PPS arrived
  //digitalWrite(FLG_PIN,ppsfl);    // Flag set (for debug)
  rega0 = TC0->TC_CHANNEL[0].TC_RA; // Read the RA reg (PPS period)
  stsr0 = TC_GetStatus(TC0, 0);     // Read status and clear load bits
  //event = 0;

  ppcnt++;        // PPS count
  displ = 1;        // Display stuff in the loop
}

 
void TC6_Handler() {

  // This ISR is connected to the OR of the event and the PPS 
  // If the TC0 has seen the PPS it sets the flag high
  // and if its high we are seeing the PPS here, but if the
  // flag is not set then this is a cosmic ray event.

  //if (ppsfl == HIGH) {      // Was ther a PPS ? 
    //ppsfl = LOW;      // Yes so we have seen it here
    //digitalWrite(PPS_PIN,LOW);  // Reset PPS
    //digitalWrite(EVT_PIN,LOW);  // Not an event
  //} else {
    //digitalWrite(EVT_PIN,HIGH); // Event detected
    //event = TC2->TC_CHANNEL[0].TC_RA;
  //}
  
  //digitalWrite(FLG_PIN,ppsfl);    // Flag out
  //Serial.print("event");

  event = TC0->TC_CHANNEL[0].TC_CV; // Read thge RA on channel 1 (PPS period)
  stsr1 = TC_GetStatus(TC2, 0);     // Read status clear load bits
}

void setup() {
  //pinMode(FLG_PIN, OUTPUT);   // Pin for the ppsfl flag for debug
  //pinMode(EVT_PIN, OUTPUT);   // Pin for the cosmic ray event 
  //pinMode(PPS_PIN, OUTPUT);   // Pin for the PPS (LED pin)
  Serial.begin(115200);     // Start the serial line
  TimersStart();        // Start timers
}

void loop() {
  if (displ) {
    Serial.print("ppcnt: "); Serial.print(ppcnt);Serial.print(", ");
    Serial.print("rega0: "); Serial.print(rega0);Serial.print(", ");
    Serial.print("event: "); Serial.print(event);
    Serial.println("");
    displ = 0;
  }
}
