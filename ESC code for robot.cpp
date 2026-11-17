/*


instructions:

run .exe and it will do a simulated ESC commutation and we will be able to separately
build this code using a .bat file to build this code using Keil® PK51 Developer’s Kit's
linker, assembler and hex converter which turns our code into a .hex file ready for
flashing onto BlHeliS MCUs or w/e...  - this way we can test and write our code on
windows but make the MCU version build separately with that .bat helping

commands to esc should be like:
       acceleration 7 (out of 10), speed 5 (out of 10), power 5 (out of 10)
       and this equates to: this would say accelerate at 7 out of 10 acceleration speed 
       (and this would be some preset thing like move commutation forward to next 
       commutation step every 2ms then 1ms then .5ms then .2ms or w/e to ramp up at a 7
       out of 10 ramp up rate but once we hit speed of 5 which is overal ms between 
       commutation steps, no longer accelerate now coast at that rate, and power would 
       be duty cycle we use AT each commutation step

esc defaults to freewheeling so it draws little to no power when turned on at first and awaits
  commands and if a power cut happened while robot is sprinting, a esc coming back online does
  not default to a full stall as that would jerk a leg moving forward rapidly to a fast halt
  stressing the system and causing a fall etc better to be limp mode at first startup
  
esc defaults to no back emf usage and no feedback at all - it acts like a dumb brushed motor
just trying to move forward no idea where rotor is and that's fine - the arduino "flight controller"
controlling the esc is to watch the potentiometer on the finger joint and determine hey motor
has not moved for a while turn up its power level and lower its speed a bit because it must
be bumping something and stalled... - its up to main brains pc really to make these determinations
of if something is stalled, how fast to go, how much load is expected to be fighting each motor
and how hard the motor is to run its power setting (which is duty cycle of it) etc so as to meet
the needs it guesses the esc will have to successfully push the rotor as intended without slippage/skipping
of the commutation steps faster than rotor can keep up etc - and this type of guessed appropriate speed
and power level can be learned over time and accurately guessed and thereby eliminate any need for
complicated and unreliable back emf readings to assist in locating rotor etc - this also greatly
simplifies the ESC firmware which is ideal - we want the ESC to be dumb - just a dumb brushed
dc motor or dumb 3d printer stepper motor type vibe, not needing to know its rotor position or if
it is stalled or anything, just using guessed motion profiles and guessing accuracy is NOT up to
it but up to the main brains pc of the robot to guess accurately or not and main brains pc of
robot will GROW its knowledge of accurate guessing over time through trial and error and learning!

note: one reason to keep the ESC VERY VERY basic and dumb is so that the sophistication of its
control is not IN the ESC but IN the main brains PC which we can update often easily over wifi
or w/e whereas the firmware of the 300+ ESCs we want to RARELY EVER need to flash as the task
of flashing them all would be a pain IMO and this would have to be done after every change ugh! - we
want the ESC to be a nice dumb worker bot... - in fact, even the arduino acting as the "flight
controller" shouldn't be deciding much at all - let the main brains pc decide it all and the arduino
network is just the messengers of that

note:  this code was made initially for the EFM8BB21F16G-C02M2E Busy Bee MCU of a LANRC 45a BLHeli_S
2-6s Lipo but we can build tweaked versions of this code for the other cheap ESCs I also bought
off aliexpress that presumably use similar chips just changing the pin mapping and register
values or w/e to fit those or w/e... and then output a separate .hex for those when we do convert
this code to a .hex using our .bat file and its commands sent through the keil toolchain's assembler
and linker and hex converter or w/e...; this chip's datasheet is here:
https://www.silabs.com/documents/public/data-sheets/efm8bb2-datasheet.pdf --- and its pinout
is on page 44

note: the 3 phase mosfet gate driver ic for all 6 mosfets for the Busy Bee MCU of a LANRC 45a BLHeli_S
ESC is called the:  HGSEMI HD6288Q/TR-HG  https://www.lcsc.com/product-detail/C54348660.html




















nextsection

index

variables declarations section
initialize microcontroller of ESC section
interrupt for communication pin change detection and recording in realtime section

*/



#define _WIN32_WINNT 0x0500            //tells the compiler that this is windows2000 or higher... by telling the compiler this, we gain access to the GetConsoleWindow(); command!  This is a necessary command for being able to get the handle of the console window... once we have the handle of the window, we can tell it to do things.. like dissapear for a while! lol 
                                         //also note that this command MUST be written in BEFORE #include <windows.h>...
#include <iostream>
#include <windows.h>
#include <iomanip>
#include <time.h>  //allows getting system time
#include <math.h>  //allows use of sqrt() function...
#include <graphics.h>      //the header we need to use for creating of windows using borland's winBGlm library...
#include <fstream>    // must include this header in order to access files
#include <lm.h>  //must include this header for clipboard stuff

using namespace std;    





//-/sfr watchdogTimerControlRegister = 0x97;
//-/sfr P1 = 0x90;
//-/sfr P1MDOUT = 0xA5;
//-/sfr P1MDIN = 0xF2;
//-/sfr CLKSEL = 0xA9;
//-/sfr PFE0CN = 0xC1;
//-/sfr CKCON0  = 0x8E;
//-/sfr TMR2CN0 = 0xC8;
//-/sfr TMR2RLL = 0xCA;
//-/sfr TMR2RLH = 0xCB;
//-/sfr TMR2L   = 0xCC;
//-/sfr TMR2H   = 0xCD;
//-/sfr IT01CF = 0xE4;
//-/sfr TCON = 0x88;
//-/sfr IE = 0xA8;




void SiLabs_Startup(void)
{
    //disables watchdog timer of the ESC
    //this is needed because otherwise the timer resets loading the code in a infinite loop if it takes too 
    //long to load due to having too many global variable names?  ... w/e...
    
    //-/watchdogTimerControlRegister = 0xDE;
    //-/watchdogTimerControlRegister = 0xAD;
}






bool debuggingExpensesTallyingLogFileProcessing = false;
bool debuggingAddingEarningsForEachColumn = false;
bool debuggingEarningsTallyingLogFileProcessing = false;
bool beginCode = true;
bool initializeMicroControllerOfESC = true;
bool phaseOneHighsideMosfet = false;
bool phaseOneLowsideMosfet = false;
bool phaseTwoHighsideMosfet = false;
bool phaseTwoLowsideMosfet = false;
bool phaseThreeHighsideMosfet = false;
bool phaseThreeLowsideMosfet = false;
unsigned int systemTimer;
//unsigned int duration;
char testChar;
char receiveBuffer[256];
int receiveBufferWritePosition = 0;
int receiveBufferReadPosition = 0;
//-/void communicationInterrupt(void) interrupt 0;
/*-*/void communicationInterrupt(void);
int receivedBit = 0;
int receiveBufferDistance = 0;
int forwardAcceleration = 0;
int forwardSpeed = 0;
int forwardPower = 0;
bool highSignalRecordedPreviously = false;
bool checkForArduinoIncomingCommand = false;
bool initialEscFirmwareStartupWaitingForArduinoIncomingCommandCheck = false;
int bufferPositionsCheckedCount = 0;
bool seeIfWeHaveAnOpeningBracketForACommandFromArduino = false;
bool initialEscFirmwareStartupWaitingForSeeingIfTheseEightBitsAreAOpeningBracketForAnArduinoCommand = false;
int receivedCharacter = 0;
int receivedBitCount = 0;
int receiveBufferCheckPosition = 0;
bool grabABitFromReceivedMessageBuffer = false;
bool seeingIfWeHaveAnOpeningBracketForACommandFromArduinoWaitingForGrabbingABitFromReceivedMessageBuffer = false;
bool grabbingABitFromReceivedMessageBufferCompleted = false;
bool weFoundTheOpeningBracket = false;
bool seeingIfOpenBracketCommandFromArduinoIsHereCompleted = false;
bool arduinoIncomingCommandCheckCompleted = false;
int locationOfOpeningBracket = 0;
int locationOfStartOfPotentialValidCommand = 0;
bool seeIfWeHaveAClosingBracketForACommandFromArduino = false;
bool initialEscFirmwareStartupWaitingForSeeingIfTheseEightBitsAreAClosingBracketForAnArduinoCommand = false; 
bool seeingIfWeHaveAClosingBracketForACommandFromArduinoWaitingForGrabbingABitFromReceivedMessageBuffer = false;
bool weFoundTheClosingBracket = false;
bool seeingIfClosingBracketCommandFromArduinoIsHereCompleted = false;
int totalPositionsContainedInCommand = 0;
bool pullInTheCommandContainedWithinTheBrackets = false;
bool initialFirmwareStartupWaitingForPullingInTheCommandContainedWithinTheBrackets = false;
int commandBufferPosition = 0;
bool pullInNextCommandCharacter = false;
bool initialFirmwareStartupWaitingForPullingInTheNextCommandCharacter = false;
char commandBuffer[33];
bool pullingInNextCommandCharacterCompleted = false;
bool pullingInTheCommandContainedWithinTheBracketsCompleted = false;
bool clearTheCommandWeJustGrabbedFromTheReceiveBuffer = false;
bool initialFirmwareStartupWaitingForClearingTheCommandFromTheReceiveBuffer = false;
int totalPositionsOfCommandIncludingOpeningAndClosingBits = 0;
int countNum = 0;
bool clearingTheCommandFromTheReceiveBufferCompleted = false;
bool verifyTheIncomingCommandIsAValidKnownCommand = false;
bool initialFirmwareStartupWaitingForVerifyingTheIncomingCommandIsAValidKnownCommand = false;
bool aValidCommandWasDetected = false;
bool forwardCommandDetected = false;
bool reverseCommandDetected = false;
int reverseAcceleration = 0;
int reverseSpeed = 0;
int reversePower = 0;
bool brakeCommandDetected = false;
int brakeDeceleration = 0;
int brakePower = 0;
bool verifyingCommandIsKnownAndValidCompleted = false;
bool executeForwardMotionAnimation = false;
bool executeReverseMotionAnimation = false;
bool executeBrakeMotionAnimation = false;
bool initiateForwardMotorEngagementCommutationStep = false; 
bool forwardMotorEngagementCommutationStepInitiationCompleted = false;
bool turnOnFirstCommutationStep = false;
bool turnOnSpecificRegisterLocationsOfMCU = false;
bool turningOnFirstCommutationStepWaitingForTurningOnSpecificRegisterLocationsOfMCU = false;
bool turningOnSpecificRegisterLocationsOfMCUCompleted = false;
bool turningOnFirstCommutationStepCompleted = false;
bool escFirmwareWaitingForTurningOnFirstCommutationStep = false;
bool communicationsSignalPinTurnedHigh = false; 
bool runInterruptCodeMainWhileLoop = false;
bool seeIfThisIsFirstEverRecordedHighSignal = false;
bool interruptCodeSectionWaitingToSeeIfThisIsFirstEverRecordedHighSignal = false;
bool weAreHighSignalAsOfLastKnownStateFromLastTimeWeCheckedOnInterrupts = false;
bool seeingIfThisIsFirstEverRecordedHighSignalCompleted = false;
bool markInitialLowSignalTriggerPointTimingForFutureReference = false;
bool interruptCodeSectionWaitingForMarkingInitialLowSignalTriggerPointTimingForFutureReference = false;
bool markTimeOfThisEndingLowSignalPointAndDecipherDurationToDetermineHighOrLowBitMeaning = false;
bool interruptCodeSectionWaitingForMarkingTimeOfThisLowSignalEndingPointAndStoringBitMeaning = false; 
bool markingInitialLowTriggerPointTimingForFutureReferenceCompleted = false; 
unsigned int durationOfLowPulse = 0;
unsigned int lowPulseEndTime = 0;
unsigned int lowPulseStartTime = 0;
bool storingLowBitEndingMeaningCompleted = false;
bool forwardMotionAnimationWaitingForForwardMotorEngagementCommutationStepToEngage = false;
bool forwardNudgeCommandDetected = false;
int forwardNudgeSpeed = 0;
int forwardNudgePower = 0;
int forwardNudgeRotations = 0;
bool backwardNudgeCommandDetected = false;
int backwardNudgeSpeed = 0;
int backwardNudgePower = 0;
int backwardNudgeRotations = 0;
bool executeForwardNudgeMotionAnimation = false;
bool executeBackwardNudgeMotionAnimation = false;
int positionsAdvancedInCommand = 0;








//nextSection                       variables declarations section

bool debuggingInitialFirmwareStartup = true;
bool insertDummyArduinoIncomingCommandDirectlyForTesting = true;











int main()              
{     
      
      
    while (beginCode == true)
    {
        
        
        //nextSection                       initialize microcontroller of ESC section       
        
        if (initializeMicroControllerOfESC == true)
        {
            initializeMicroControllerOfESC = false;
            
            if (debuggingInitialFirmwareStartup == true)
            {
                /*-*/cout << "initializing microcontroller of ESC now\n";
                /*-*/system("Pause");                                    
            }
            
            
            //physical trace mapping:
                       
            //MCU P1.0 ? HD6288 HIN1 (HD6288 is name of the 6 mosfet gate driver chip that mcu's 6 pins drive)
            //MCU P1.1 ? HD6288 HIN2
            //MCU P1.2 ? HD6288 HIN3
            //MCU P1.3 ? HD6288 LIN1
            //MCU P1.4 ? HD6288 LIN2
            //MCU P1.5 ? HD6288 LIN3
            
            //MCU P0.4 signal line of white communications wire
            
            
            
            // Set gate-driver inputs LOW
            
            // Set P1.0/HIN1 latch LOW 
            //-/P1 &= 0xFE;
        
            // Set P1.1/HIN2 latch LOW
            //-/P1 &= 0xFD;
        
            // Set P1.2/HIN3 latch LOW
            //-/P1 &= 0xFB;
        
            // Set P1.3/LIN1 latch LOW
            //-/P1 &= 0xF7;
        
            // Set P1.4/LIN2 latch LOW
            //-/P1 &= 0xEF;
        
            // Set P1.5/LIN3 latch LOW
            //-/P1 &= 0xDF;
            
            
            
            


            
            // Configure as digital
            // Configure P1.0 as digital
            //-/P1MDIN |= 0x01;
        
            // Configure P1.1 as digital
            //-/P1MDIN |= 0x02;
        
            // Configure P1.2 as digital
            //-/P1MDIN |= 0x04;
        
            // Configure P1.3 as digital
            //-/P1MDIN |= 0x08;
        
            // Configure P1.4 as digital
            //-/P1MDIN |= 0x10;
        
            // Configure P1.5 as digital
            //-/P1MDIN |= 0x20;
            
            
            // Configure as push-pull
            // Configure P1.0 as push-pull
            //-/P1MDOUT |= 0x01;
        
            // Configure P1.1 as push-pull
            //-/P1MDOUT |= 0x02;
        
            // Configure P1.2 as push-pull
            //-/P1MDOUT |= 0x04;
        
            // Configure P1.3 as push-pull
            //-/P1MDOUT |= 0x08;
        
            // Configure P1.4 as push-pull
            //-/P1MDOUT |= 0x10;
        
            // Configure P1.5 as push-pull
            //-/P1MDOUT |= 0x20;   
            
            
            
            
            
            //this is the communication white signal wire input/output line for getting commands from arduino
            // Configure P0.4 as digital
            //-/P0MDIN |= 0x10;
            
            // Configure P0.4 as input / open-drain
            //-/P0MDOUT &= 0xEF;
            
            
            
            
            
            
            
            // Enable flash prefetch and set flash read timing for SYSCLK < 50 MHz
            //-/PFE0CN = 0x30;
            
            // First switch from the reset 3.0625 MHz SYSCLK
            // to the 24.5 MHz HFOSC0 source.
            //-/CLKSEL = 0x00;
            
            // Wait until the new clock has propagated through the divider.
            //-/while ((CLKSEL & 0x80) == 0)
            //-/{
            //-/    //intentially wait here
            //-/}
            
            // Now switch from 24.5 MHz HFOSC0 to 49 MHz HFOSC1.
            //-/CLKSEL = 0x03;
            
            // Wait until the new clock has propagated through the divider.
            //-/while ((CLKSEL & 0x80) == 0)
            //-/{
            //-/    //intentially wait here
            //-/}
            

            
            
            
            
            
            
            // Stop Timer 2 while configuring it.
            //-/TMR2CN0 = 0x00;
        
            // 16-bit auto-reload mode.
            // Reload value 0x0000 gives the full 0x0000-0xFFFF range.
            //-/TMR2RLL = 0x00;
            //-/TMR2RLH = 0x00;
        
            // Start counting from zero.
            //-/TMR2L = 0x00;
            //-/TMR2H = 0x00;
        
            // Select SYSCLK as Timer 2 clock.
            // CKCON0 bit 4 = T2ML = 1.
            //-/CKCON0 |= 0x10;
        
            // Start Timer 2.
            // TMR2CN0 bit 2 = TR2.
            //-/TMR2CN0 |= 0x04;
            
            
            
            
            
            
            //-/IT01CF = 0x04;       // Select P0.4
            //-/TCON |= 0x01;        // INT0 edge sensitive
            //-/IE |= 0x81;          // Enable INT0 and global interrupts
            //-/IT01CF |= 0x08;    // Configure INT0 to trigger on rising edge 
            
            
            
            
            if (insertDummyArduinoIncomingCommandDirectlyForTesting == true)
            {
                /*-*/cout << "plugging in a test case receive buffer command to simulate reading/acting on it\n";
                /*-*/system("Pause");  
                
                countNum = 0;
                
                while (countNum < 256)
                {
                    receiveBuffer[countNum] = 0;
                    
                    countNum = countNum + 1;
                }
                
                // [ = 91 = 01011011
                receiveBuffer[252] = 0;
                receiveBuffer[253] = 1;
                receiveBuffer[254] = 0;
                receiveBuffer[255] = 1;
                receiveBuffer[0] = 1;
                receiveBuffer[1] = 0;
                receiveBuffer[2] = 1;
                receiveBuffer[3] = 1;     
                
                
                // F = 70 = 01000110
                receiveBuffer[4]  = 0;
                receiveBuffer[5]  = 1;
                receiveBuffer[6]  = 0;
                receiveBuffer[7]  = 0;
                receiveBuffer[8]  = 0;
                receiveBuffer[9]  = 1;
                receiveBuffer[10] = 1;
                receiveBuffer[11] = 0;
                
                
                // ] = 93 = 01011101
                receiveBuffer[12] = 0;
                receiveBuffer[13] = 1;
                receiveBuffer[14] = 0;
                receiveBuffer[15] = 1;
                receiveBuffer[16] = 1;
                receiveBuffer[17] = 1;
                receiveBuffer[18] = 0;
                receiveBuffer[19] = 1;                      
            }
            
            
            
            
            

            
            checkForArduinoIncomingCommand = true;
            
            initialEscFirmwareStartupWaitingForArduinoIncomingCommandCheck = true;
        }
        
        
        
        
        
        
        
        if (checkForArduinoIncomingCommand == true)
        {
            checkForArduinoIncomingCommand = false;
            
            
            //[FORWARD 5 5 5]   --- this is command where forward is direction, 5(of 10) is acceleration, 5(of 10) is speed and 5 (of 10 is power level/duty cycle)
            //[Reverse 5 5 5]   --- this is command where reverse is direction, 5(of 10) is acceleration, 5(of 10) is speed and 5 (of 10 is power level/duty cycle)
            //[Brake 5 5]   --- this is command where brake is the job, 5(of 10) is deceleration, 5(of 10) is power level/duty cycle)
            
            
            if (debuggingInitialFirmwareStartup == true)
            {
                /*-*/cout << "checking for arduino incoming command\n";
                /*-*/system("Pause");                                
            }
            
            
            
            receiveBufferReadPosition = 0;
            
            bufferPositionsCheckedCount = 0;
            
            seeIfWeHaveAnOpeningBracketForACommandFromArduino = true;
            
            initialEscFirmwareStartupWaitingForSeeingIfTheseEightBitsAreAOpeningBracketForAnArduinoCommand = true;                                                 
        }
        
        
        
        
        
        
        
        if (seeIfWeHaveAnOpeningBracketForACommandFromArduino == true)
        {
            seeIfWeHaveAnOpeningBracketForACommandFromArduino = false;
            
            
            if (debuggingInitialFirmwareStartup == true)
            {
                /*-*/cout << "seeing if we have an opening bracket for a command from arduino\n";
                /*-*/system("Pause");                                
            }
            
            
            
            receivedCharacter = 0;
            receivedBitCount = 0;
            receiveBufferCheckPosition = receiveBufferReadPosition;    
            
            grabABitFromReceivedMessageBuffer = true;
            
            seeingIfWeHaveAnOpeningBracketForACommandFromArduinoWaitingForGrabbingABitFromReceivedMessageBuffer = true;                                                      
        }
        
        
        
        
        
        
        
        if (grabABitFromReceivedMessageBuffer == true)
        {
            grabABitFromReceivedMessageBuffer = false;
            
            
            if (debuggingInitialFirmwareStartup == true)
            {
                /*-*/cout << "grabbing a bit from receive message buffer\n";
                /*-*/system("Pause");                                
            }
            
            
            
            if (debuggingInitialFirmwareStartup == true)
            {
                /*-*/cout << "the bit is: (int)receiveBuffer[receiveBufferCheckPosition]: " << (int)receiveBuffer[receiveBufferCheckPosition] << "\n";
                /*-*/system("Pause");                                
            }
            
            
            receivedCharacter = receivedCharacter << 1;
            receivedCharacter = receivedCharacter | receiveBuffer[receiveBufferCheckPosition];
    
            receiveBufferCheckPosition = receiveBufferCheckPosition + 1;
    
            if (receiveBufferCheckPosition >= 256)
            {
                receiveBufferCheckPosition = 0;
            }   
            
            
            receivedBitCount = receivedBitCount + 1;
            
            
            
            grabbingABitFromReceivedMessageBufferCompleted = true;                                   
        }
        
        
        
        
        
        
        
        if (seeingIfWeHaveAnOpeningBracketForACommandFromArduinoWaitingForGrabbingABitFromReceivedMessageBuffer == true && grabbingABitFromReceivedMessageBufferCompleted == true)
        {
            seeingIfWeHaveAnOpeningBracketForACommandFromArduinoWaitingForGrabbingABitFromReceivedMessageBuffer = false;
            
            grabbingABitFromReceivedMessageBufferCompleted = false;
            
            
            
            if (receivedBitCount < 8)
            {
                if (debuggingInitialFirmwareStartup == true)
                {
                    /*-*/cout << "need to grab more bits since we haven't grabbed 8 bits yet\n";
                    /*-*/system("Pause");                                
                }
                
                grabABitFromReceivedMessageBuffer = true;
            
                seeingIfWeHaveAnOpeningBracketForACommandFromArduinoWaitingForGrabbingABitFromReceivedMessageBuffer = true;                  
            }
            
            if (receivedBitCount >= 8)
            {
                if (debuggingInitialFirmwareStartup == true)
                {
                    /*-*/cout << "we grabbed 8 bits now...\n";
                    /*-*/cout << "and the resulting character is: (int)receivedCharacter: " << (int)receivedCharacter << "...\n";
                    /*-*/cout << "and the resulting character is: (char)receivedCharacter: " << (char)receivedCharacter << "...\n";
                    /*-*/system("Pause");                                
                }
                
                
                weFoundTheOpeningBracket = false;
                
                if (receivedCharacter == '[')
                {
                    weFoundTheOpeningBracket = true;
                }   
                
                
                seeingIfOpenBracketCommandFromArduinoIsHereCompleted = true;              
            }                                                                                                  
        }
        
        
        
        
        
        
        
        if (initialEscFirmwareStartupWaitingForSeeingIfTheseEightBitsAreAOpeningBracketForAnArduinoCommand == true && seeingIfOpenBracketCommandFromArduinoIsHereCompleted == true)
        {
            initialEscFirmwareStartupWaitingForSeeingIfTheseEightBitsAreAOpeningBracketForAnArduinoCommand = false;
            
            seeingIfOpenBracketCommandFromArduinoIsHereCompleted = false;
            
            
            
            
            if (weFoundTheOpeningBracket == false)
            {
                if (debuggingInitialFirmwareStartup == true)
                {
                    /*-*/cout << "this was not the opening bracket...\n";
                    /*-*/system("Pause");                                
                }
                
                receiveBufferReadPosition = receiveBufferReadPosition + 1;

                if (receiveBufferReadPosition >= 256)
                {
                    receiveBufferReadPosition = 0;
                }    
                
                if (debuggingInitialFirmwareStartup == true)
                {
                    /*-*/cout << "receiveBufferReadPosition = " << receiveBufferReadPosition << endl;
                    /*-*/system("Pause");                                
                }
                
                
                bufferPositionsCheckedCount = bufferPositionsCheckedCount + 1;
                
                if (bufferPositionsCheckedCount < 256)
                {
                    if (debuggingInitialFirmwareStartup == true)
                    {
                        /*-*/cout << "bufferPositionsCheckedCount = " << bufferPositionsCheckedCount << endl;
                        /*-*/system("Pause");                                
                    }
                    
                    
                    seeIfWeHaveAnOpeningBracketForACommandFromArduino = true;
            
                    initialEscFirmwareStartupWaitingForSeeingIfTheseEightBitsAreAOpeningBracketForAnArduinoCommand = true;                            
                }
                
                if (bufferPositionsCheckedCount >= 256)
                {
                    if (debuggingInitialFirmwareStartup == true)
                    {
                        /*-*/cout << "we have checked 256 buffer positions and now consider this arduino incoming commands check pass done\n";
                        /*-*/system("Pause");                                
                    }
                    
                    arduinoIncomingCommandCheckCompleted = true;                            
                }                   
            }
            
            if (weFoundTheOpeningBracket == true)
            {
                if (debuggingInitialFirmwareStartup == true)
                {
                    /*-*/cout << "opening bracket detected!\n";
                    /*-*/system("Pause");                                
                }
                
                locationOfOpeningBracket = receiveBufferReadPosition;
                
                if (debuggingInitialFirmwareStartup == true)
                {
                    /*-*/cout << "locationOfOpeningBracket: " << locationOfOpeningBracket << "\n";
                    /*-*/system("Pause");                                
                }
                
                receiveBufferReadPosition = receiveBufferReadPosition + 8;
                
                
                if (debuggingInitialFirmwareStartup == true)
                {
                    /*-*/cout << "advancing receiveBufferReadPosition by 8 slots forward: receiveBufferReadPosition: " << receiveBufferReadPosition << "\n";
                    /*-*/system("Pause");                                
                }
                
                
                if (receiveBufferReadPosition >= 256)
                {
                    receiveBufferReadPosition = receiveBufferReadPosition - 256;
                    
                    if (debuggingInitialFirmwareStartup == true)
                    {
                        /*-*/cout << "receiveBufferReadPosition was over 256 after adding 8 to it - so subtracting 256 from\n";
                        /*-*/cout << "it to loop back around to start of buffer but still be 8 positions forward from\n";
                        /*-*/cout << "the start of the opening bracket location: new receiveBufferReadPosition: " << receiveBufferReadPosition << "\n";
                        /*-*/system("Pause");                                
                    }
                }  
                
                
                
                //[FORWARD 5 5 5]   --- this is command where forward is direction, 5(of 10) is acceleration, 5(of 10) is speed and 5 (of 10 is power level/duty cycle)
                //[Reverse 5 5 5]   --- this is command where reverse is direction, 5(of 10) is acceleration, 5(of 10) is speed and 5 (of 10 is power level/duty cycle)
                //[Brake 5 5]   --- this is command where brake is the job, 5(of 10) is deceleration, 5(of 10) is power level/duty cycle)
                //[Forward Nudge 5 5 5]  --- this is command where forward is direction, 5(of 10) is speed, 5(of 10) is power, 5(of 10) is full rotations
                
                locationOfStartOfPotentialValidCommand = receiveBufferReadPosition;
                
                bufferPositionsCheckedCount = 0;
                
                seeIfWeHaveAClosingBracketForACommandFromArduino = true;
            
                initialEscFirmwareStartupWaitingForSeeingIfTheseEightBitsAreAClosingBracketForAnArduinoCommand = true;                    
            }                                                                                                    
        }
        
        
        
        
        
        
        
        if (seeIfWeHaveAClosingBracketForACommandFromArduino == true)
        {
            seeIfWeHaveAClosingBracketForACommandFromArduino = false;
            
            
            if (debuggingInitialFirmwareStartup == true)
            {
                /*-*/cout << "seeing if we have a closing bracket for a command from arduino\n";
                /*-*/system("Pause");                                
            }
            
            
            receivedCharacter = 0;
            receivedBitCount = 0;
            receiveBufferCheckPosition = receiveBufferReadPosition;    
            
            grabABitFromReceivedMessageBuffer = true;
            
            seeingIfWeHaveAClosingBracketForACommandFromArduinoWaitingForGrabbingABitFromReceivedMessageBuffer = true;                                                      
        }
        
        
        
        
        
        
        
        if (seeingIfWeHaveAClosingBracketForACommandFromArduinoWaitingForGrabbingABitFromReceivedMessageBuffer == true && grabbingABitFromReceivedMessageBufferCompleted == true)
        {
            seeingIfWeHaveAClosingBracketForACommandFromArduinoWaitingForGrabbingABitFromReceivedMessageBuffer = false;
            
            grabbingABitFromReceivedMessageBufferCompleted = false;
            
            
            
            if (receivedBitCount < 8)
            {
                if (debuggingInitialFirmwareStartup == true)
                {
                    /*-*/cout << "need to grab more bits since we haven't grabbed 8 bits yet\n";
                    /*-*/system("Pause");                                
                }
                
                grabABitFromReceivedMessageBuffer = true;
            
                seeingIfWeHaveAClosingBracketForACommandFromArduinoWaitingForGrabbingABitFromReceivedMessageBuffer = true;                  
            }
            
            if (receivedBitCount >= 8)
            {
                if (debuggingInitialFirmwareStartup == true)
                {
                    /*-*/cout << "we grabbed 8 bits now...\n";
                    /*-*/cout << "and the resulting character is: (int)receivedCharacter: " << (int)receivedCharacter << "...\n";
                    /*-*/cout << "and the resulting character is: (char)receivedCharacter: " << (char)receivedCharacter << "...\n";
                    /*-*/system("Pause");                                
                }
                
                weFoundTheClosingBracket = false;
                
                if (receivedCharacter == ']')
                {
                    weFoundTheClosingBracket = true;
                }   
                
                
                seeingIfClosingBracketCommandFromArduinoIsHereCompleted = true;              
            }                                                                                                  
        }
        
        
        
        
        
        
        
        if (initialEscFirmwareStartupWaitingForSeeingIfTheseEightBitsAreAClosingBracketForAnArduinoCommand == true && seeingIfClosingBracketCommandFromArduinoIsHereCompleted == true)
        {
            initialEscFirmwareStartupWaitingForSeeingIfTheseEightBitsAreAClosingBracketForAnArduinoCommand = false;
            
            seeingIfClosingBracketCommandFromArduinoIsHereCompleted = false;
            
            
            if (weFoundTheClosingBracket == false)
            {
                if (debuggingInitialFirmwareStartup == true)
                {
                    /*-*/cout << "this was not the closing bracket...\n";
                    /*-*/system("Pause");                                
                }
                
                receiveBufferReadPosition = receiveBufferReadPosition + 1;
                
                if (debuggingInitialFirmwareStartup == true)
                {
                    /*-*/cout << "receiveBufferReadPosition++: new receiveBufferReadPosition: " << receiveBufferReadPosition << "\n";
                    /*-*/system("Pause");                                
                }
                
                if (receiveBufferReadPosition >= 256)
                {
                    receiveBufferReadPosition = 0;
                    
                    if (debuggingInitialFirmwareStartup == true)
                    {
                        /*-*/cout << "receiveBufferReadPosition was over 256 so looping it back to zero to read from start of buffer\n";
                        /*-*/cout << "still in search of closing bracket: new receiveBufferReadPosition: " << receiveBufferReadPosition << "\n";
                        /*-*/system("Pause");                                
                    }
                }    
                
                
                
                
                
                bufferPositionsCheckedCount = bufferPositionsCheckedCount + 1;
                
                if (bufferPositionsCheckedCount < 240)
                {
                    if (debuggingInitialFirmwareStartup == true)
                    {
                        /*-*/cout << "bufferPositionsCheckedCount = " << bufferPositionsCheckedCount << endl;
                        /*-*/system("Pause");                                
                    }
                    
                    seeIfWeHaveAClosingBracketForACommandFromArduino = true;
            
                    initialEscFirmwareStartupWaitingForSeeingIfTheseEightBitsAreAClosingBracketForAnArduinoCommand = true;                           
                }
                
                if (bufferPositionsCheckedCount >= 240)
                {
                    if (debuggingInitialFirmwareStartup == true)
                    {
                        /*-*/cout << "we have checked 240 buffer positions and not found a closing bracket in any of them -\n";
                        /*-*/cout << "therefore we disregard that opening bracket we saw earlier and we now consider this\n";
                        /*-*/cout << "arduino incoming commands check pass done\n";
                        /*-*/system("Pause");                                
                    }
                    
                    arduinoIncomingCommandCheckCompleted = true;                            
                }                   
            }
            
            if (weFoundTheClosingBracket == true)
            {
                if (debuggingInitialFirmwareStartup == true)
                {
                    /*-*/cout << "we have found the closing bracket!\n";
                    /*-*/system("Pause");                                
                }
                
                receiveBufferReadPosition = locationOfStartOfPotentialValidCommand;
                
                totalPositionsContainedInCommand = bufferPositionsCheckedCount;
                
                
                if (debuggingInitialFirmwareStartup == true)
                {
                    /*-*/cout << "totalPositionsContainedInCommand: " << totalPositionsContainedInCommand << "\n";
                    /*-*/system("Pause");                                
                }
                
                
                //[FORWARD 5 5 5]   --- this is command where forward is direction, 5(of 10) is acceleration, 5(of 10) is speed and 5 (of 10 is power level/duty cycle)
                //[Reverse 5 5 5]   --- this is command where reverse is direction, 5(of 10) is acceleration, 5(of 10) is speed and 5 (of 10 is power level/duty cycle)
                //[Brake 5 5]   --- this is command where brake is the job, 5(of 10) is deceleration, 5(of 10) is power level/duty cycle)
                //[Forward Nudge 5 5 5]  --- this is command where forward is direction, 5(of 10) is speed, 5(of 10) is power, 5(of 10) is full rotations
                
                pullInTheCommandContainedWithinTheBrackets = true;
                
                initialFirmwareStartupWaitingForPullingInTheCommandContainedWithinTheBrackets = true;                    
            }                                                                                                    
        }
        
        
        
        
        
        
        
        if (pullInTheCommandContainedWithinTheBrackets == true)
        {
            pullInTheCommandContainedWithinTheBrackets = false;
            
            
            if (debuggingInitialFirmwareStartup == true)
            {
                /*-*/cout << "pulling in the command contained within the brackets\n";
                /*-*/system("Pause");                                
            }
            
            
            //[FORWARD 5 5 5]   --- this is command where forward is direction, 5(of 10) is acceleration, 5(of 10) is speed and 5 (of 10 is power level/duty cycle)
            //[Reverse 5 5 5]   --- this is command where reverse is direction, 5(of 10) is acceleration, 5(of 10) is speed and 5 (of 10 is power level/duty cycle)
            //[Brake 5 5]   --- this is command where brake is the job, 5(of 10) is deceleration, 5(of 10) is power level/duty cycle)
            //[Forward Nudge 5 5 5]  --- this is command where forward is direction, 5(of 10) is speed, 5(of 10) is power, 5(of 10) is full rotations
            
            commandBufferPosition = 0;

            receiveBufferCheckPosition = receiveBufferReadPosition;  
            
            positionsAdvancedInCommand = 0;
            
            if (debuggingInitialFirmwareStartup == true)
            {
                /*-*/cout << "commandBufferPosition: " << commandBufferPosition << "\n";
                /*-*/cout << "receiveBufferReadPosition: " << receiveBufferReadPosition << endl;
                /*-*/cout << "receiveBufferCheckPosition: " << receiveBufferCheckPosition << endl;
                /*-*/cout << "positionsAdvancedInCommand: " << positionsAdvancedInCommand << endl;
                /*-*/system("Pause");                                
            }
            
            
            pullInNextCommandCharacter = true;
            
            initialFirmwareStartupWaitingForPullingInTheNextCommandCharacter = true;                                             
        }
        
        
        
        
        
        
        
        if (pullInNextCommandCharacter == true)
        {
            pullInNextCommandCharacter = false;
            
            
            if (debuggingInitialFirmwareStartup == true)
            {
                /*-*/cout << "pulling in the next command character\n";
                /*-*/system("Pause");                                
            }
            
            
            receivedCharacter = 0;
            receivedBitCount = 0;
    
            while (receivedBitCount < 8)
            {
                receivedCharacter = receivedCharacter << 1;
    
                receivedCharacter = receivedCharacter | receiveBuffer[receiveBufferCheckPosition];
    
                receiveBufferCheckPosition = receiveBufferCheckPosition + 1;
    
                if (receiveBufferCheckPosition >= 256)
                {
                    receiveBufferCheckPosition = 0;
                }
    
                receivedBitCount = receivedBitCount + 1;
            }
    
            commandBuffer[commandBufferPosition] = receivedCharacter;
            commandBufferPosition = commandBufferPosition + 1; 
            
            
            positionsAdvancedInCommand = positionsAdvancedInCommand + 8;
            
            
            if (debuggingInitialFirmwareStartup == true)
            {
                /*-*/cout << "commandBufferPosition: " << commandBufferPosition << endl;
                /*-*/cout << "receivedCharacter: " << receivedCharacter << "\n";
                /*-*/cout << "positionsAdvancedInCommand: " << positionsAdvancedInCommand << endl;
                /*-*/system("Pause");                                
            }
            
            
            pullingInNextCommandCharacterCompleted = true;                             
        }
        
        
        
        
        
        
        
        if (initialFirmwareStartupWaitingForPullingInTheNextCommandCharacter == true && pullingInNextCommandCharacterCompleted == true)
        {
            initialFirmwareStartupWaitingForPullingInTheNextCommandCharacter = false;
            
            pullingInNextCommandCharacterCompleted = false;
            
            
            if (positionsAdvancedInCommand < totalPositionsContainedInCommand)
            {
                pullInNextCommandCharacter = true;
            
                initialFirmwareStartupWaitingForPullingInTheNextCommandCharacter = true;                       
            }  
            
            if (positionsAdvancedInCommand >= totalPositionsContainedInCommand)
            {
                pullingInTheCommandContainedWithinTheBracketsCompleted = true;                      
            }                                                                
        }
        
        
        
        
        
        
        
        if (initialFirmwareStartupWaitingForPullingInTheCommandContainedWithinTheBrackets == true && pullingInTheCommandContainedWithinTheBracketsCompleted == true)
        {
            initialFirmwareStartupWaitingForPullingInTheCommandContainedWithinTheBrackets = false;
            
            pullingInTheCommandContainedWithinTheBracketsCompleted = false;
            
            
            if (debuggingInitialFirmwareStartup == true)
            {
                /*-*/cout << "pulling in all command characters completed...\n";
                /*-*/system("Pause");                                
            }
            
            
            clearTheCommandWeJustGrabbedFromTheReceiveBuffer = true;
            
            initialFirmwareStartupWaitingForClearingTheCommandFromTheReceiveBuffer = true;
        }
        
        
        
        
        
        
        
        if (clearTheCommandWeJustGrabbedFromTheReceiveBuffer == true)
        {
            clearTheCommandWeJustGrabbedFromTheReceiveBuffer = false;
            
            
            if (debuggingInitialFirmwareStartup == true)
            {
                /*-*/cout << "clearing command we just grabbed from the recieve buffer...\n";
                /*-*/system("Pause");                                
            }
            
            
            totalPositionsOfCommandIncludingOpeningAndClosingBits = totalPositionsContainedInCommand + 16;
            
            
            countNum = 0;
            
            receiveBufferCheckPosition = locationOfOpeningBracket;
            
            while (countNum < totalPositionsOfCommandIncludingOpeningAndClosingBits)
            {
                receiveBuffer[receiveBufferCheckPosition] = 0;
                
        
                receiveBufferCheckPosition = receiveBufferCheckPosition + 1;
        
                if (receiveBufferCheckPosition >= 256)
                {
                    receiveBufferCheckPosition = 0;
                }
                
                countNum = countNum + 1;
            }
            
            
            clearingTheCommandFromTheReceiveBufferCompleted = true;
        }
        
        
        
        
        
        
        
        if (initialFirmwareStartupWaitingForClearingTheCommandFromTheReceiveBuffer == true && clearingTheCommandFromTheReceiveBufferCompleted == true)
        {
            initialFirmwareStartupWaitingForClearingTheCommandFromTheReceiveBuffer = false;
            
            clearingTheCommandFromTheReceiveBufferCompleted = false;
            
            
            
            //[FORWARD 5 5 5]   --- this is command where forward is direction, 5(of 10) is acceleration, 5(of 10) is speed and 5 (of 10 is power level/duty cycle)
            //[Reverse 5 5 5]   --- this is command where reverse is direction, 5(of 10) is acceleration, 5(of 10) is speed and 5 (of 10 is power level/duty cycle)
            //[Brake 5 5]   --- this is command where brake is the job, 5(of 10) is deceleration, 5(of 10) is power level/duty cycle)
            //[Forward Nudge 5 5 5]  --- this is command where forward is direction, 5(of 10) is speed, 5(of 10) is power, 5(of 10) is full rotations
            
            commandBuffer[commandBufferPosition] = '\0';
        
            verifyTheIncomingCommandIsAValidKnownCommand = true;   
            
            initialFirmwareStartupWaitingForVerifyingTheIncomingCommandIsAValidKnownCommand = true;                                                                        
        }
        
        
        
        
        
        
        
        if (verifyTheIncomingCommandIsAValidKnownCommand == true)
        {
            verifyTheIncomingCommandIsAValidKnownCommand = false;
            
            
            if (debuggingInitialFirmwareStartup == true)
            {
                /*-*/cout << "verifying incoming command is a valid known command...\n";
                /*-*/system("Pause");                                
            }
            
            
            //[FORWARD 5 5 5]   --- this is command where forward is direction, 5(of 10) is acceleration, 5(of 10) is speed and 5 (of 10 is power level/duty cycle)
            //[Reverse 5 5 5]   --- this is command where reverse is direction, 5(of 10) is acceleration, 5(of 10) is speed and 5 (of 10 is power level/duty cycle)
            //[Brake 5 5]   --- this is command where brake is the job, 5(of 10) is deceleration, 5(of 10) is power level/duty cycle)
            //[Forward Nudge 5 5 5]  --- this is command where forward is direction, 5(of 10) is speed, 5(of 10) is power, 5(of 10) is full rotations
            
            
            aValidCommandWasDetected = false;
            
            forwardCommandDetected = false;
            
            if (commandBuffer[0] == 'F' && commandBuffer[1] == 'O' && commandBuffer[2] == 'R' && commandBuffer[3] == 'W' && commandBuffer[4] == 'A' && commandBuffer[5] == 'R' && commandBuffer[6] == 'D')
            {
                // [FORWARD 5 5 5]
                
                if (commandBuffer[7] == ' ' && commandBuffer[8] >= '1' && commandBuffer[8] <= '9' && commandBuffer[9] == ' ' && commandBuffer[10] >= '1' && commandBuffer[10] <= '9' && commandBuffer[11] == ' ' && commandBuffer[12] >= '1' && commandBuffer[12] <= '9')
                {
                    forwardAcceleration = commandBuffer[8] - '0'; //converts it to integer
                    forwardSpeed = commandBuffer[10] - '0'; //converts it to integer
                    forwardPower = commandBuffer[12] - '0'; //converts it to integer
                    
                    
                    if (debuggingInitialFirmwareStartup == true)
                    {
                        /*-*/cout << "forward command detected...\n";
                        /*-*/system("Pause");                                
                    }
                    
                    
                    forwardCommandDetected = true;
                
                    aValidCommandWasDetected = true;
                }
            }
            
            
            reverseCommandDetected = false;
            
            if (commandBuffer[0] == 'R' && commandBuffer[1] == 'e' && commandBuffer[2] == 'v' && commandBuffer[3] == 'e' && commandBuffer[4] == 'r' && commandBuffer[5] == 's' && commandBuffer[6] == 'e')
            {
                // [Reverse 5 5 5]
                
                if (commandBuffer[7] == ' ' && commandBuffer[8] >= '1' && commandBuffer[8] <= '9' && commandBuffer[9] == ' ' && commandBuffer[10] >= '1' && commandBuffer[10] <= '9' && commandBuffer[11] == ' ' && commandBuffer[12] >= '1' && commandBuffer[12] <= '9')
                {
                    reverseAcceleration = commandBuffer[8] - '0'; //converts it to integer
                    reverseSpeed = commandBuffer[10] - '0'; //converts it to integer
                    reversePower = commandBuffer[12] - '0'; //converts it to integer
                    
                    
                    if (debuggingInitialFirmwareStartup == true)
                    {
                        /*-*/cout << "reverse command detected...\n";
                        /*-*/system("Pause");                                
                    }
                    
                    
                    reverseCommandDetected = true;
            
                    aValidCommandWasDetected = true;
                }
            }
            
            
            
            brakeCommandDetected = false;
            
            
            if (commandBuffer[0] == 'B' && commandBuffer[1] == 'r' && commandBuffer[2] == 'a' && commandBuffer[3] == 'k' && commandBuffer[4] == 'e')
            {
                // [Brake 5 5]
                // [Brake 0 0] = freewheel command so no braking torque/power and no acceleration just release motor fully so joint goes limp
                
                if (commandBuffer[5] == ' ' && commandBuffer[6] >= '0' && commandBuffer[6] <= '9' && commandBuffer[7] == ' ' && commandBuffer[8] >= '0' && commandBuffer[8] <= '9')
                {
                    brakeDeceleration = commandBuffer[6] - '0'; //converts it to integer
                    brakePower = commandBuffer[8] - '0'; //converts it to integer
                    
                    
                    if (debuggingInitialFirmwareStartup == true)
                    {
                        /*-*/cout << "brake command detected...\n";
                        /*-*/system("Pause");                                
                    }
                    
                    
                    brakeCommandDetected = true;
            
                    aValidCommandWasDetected = true;
                }
            }  
            
            
            
            
            forwardNudgeCommandDetected = false;
            
            
            if (commandBuffer[0] == 'F' && commandBuffer[1] == 'o' && commandBuffer[2] == 'r' && commandBuffer[3] == 'w' && commandBuffer[4] == 'a' && commandBuffer[5] == 'r' && commandBuffer[6] == 'd')
            {
                // [Forward Nudge 5 5 5]
                // Forward = direction
                // First 5 = speed out of 10
                // Second 5 = power out of 10
                // Third 5 = full rotations
            
                if (commandBuffer[7] == ' ' && commandBuffer[8] == 'N' && commandBuffer[9] == 'u' && commandBuffer[10] == 'd' && commandBuffer[11] == 'g' && commandBuffer[12] == 'e')
                {
                    if (commandBuffer[13] == ' ' && commandBuffer[14] >= '0' && commandBuffer[14] <= '9' && commandBuffer[15] == ' ' && commandBuffer[16] >= '0' && commandBuffer[16] <= '9' && commandBuffer[17] == ' ' && commandBuffer[18] >= '0' && commandBuffer[18] <= '9')
                    {
                        forwardNudgeSpeed = commandBuffer[14] - '0'; //converts it to integer
                        forwardNudgePower = commandBuffer[16] - '0'; //converts it to integer
                        forwardNudgeRotations = commandBuffer[18] - '0'; //converts it to integer
                        
                        
                        if (debuggingInitialFirmwareStartup == true)
                        {
                            /*-*/cout << "forward nudge command detected...\n";
                            /*-*/system("Pause");                                
                        }
                        
                        
                        forwardNudgeCommandDetected = true;
            
                        aValidCommandWasDetected = true;
                    }
                }
            }
            
            
            
            
            
            backwardNudgeCommandDetected = false;
            
            
            if (commandBuffer[0] == 'B' && commandBuffer[1] == 'a' && commandBuffer[2] == 'c' && commandBuffer[3] == 'k' && commandBuffer[4] == 'w' && commandBuffer[5] == 'a' && commandBuffer[6] == 'r' && commandBuffer[7] == 'd')
            {
                // [Backward Nudge 5 5 5]
                // Backward = direction
                // First 5 = speed out of 10
                // Second 5 = power out of 10
                // Third 5 = full rotations
            
                if (commandBuffer[8] == ' ' && commandBuffer[9] == 'N' && commandBuffer[10] == 'u' && commandBuffer[11] == 'd' && commandBuffer[12] == 'g' && commandBuffer[13] == 'e')
                {
                    if (commandBuffer[14] == ' ' && commandBuffer[15] >= '0' && commandBuffer[15] <= '9' && commandBuffer[16] == ' ' && commandBuffer[17] >= '0' && commandBuffer[17] <= '9' && commandBuffer[18] == ' ' && commandBuffer[19] >= '0' && commandBuffer[19] <= '9')
                    {
                        backwardNudgeSpeed = commandBuffer[15] - '0'; //converts it to integer
                        backwardNudgePower = commandBuffer[17] - '0'; //converts it to integer
                        backwardNudgeRotations = commandBuffer[19] - '0'; //converts it to integer
                        
                        
                        if (debuggingInitialFirmwareStartup == true)
                        {
                            /*-*/cout << "backward nudge command detected...\n";
                            /*-*/system("Pause");                                
                        }
                        
                        
                        backwardNudgeCommandDetected = true;
            
                        aValidCommandWasDetected = true;
                    }
                }
            }
            

            
            verifyingCommandIsKnownAndValidCompleted = true;                      
        }
        
        
        
        
        
        
        
        if (initialFirmwareStartupWaitingForVerifyingTheIncomingCommandIsAValidKnownCommand == true && verifyingCommandIsKnownAndValidCompleted == true)
        {
            initialFirmwareStartupWaitingForVerifyingTheIncomingCommandIsAValidKnownCommand = false;
            
            verifyingCommandIsKnownAndValidCompleted = false;
            
            
            if (aValidCommandWasDetected == true)
            {
                if (forwardCommandDetected == true)
                {
                    executeForwardMotionAnimation = true;                 
                } 
                
                if (reverseCommandDetected == true)
                {
                    executeReverseMotionAnimation = true;                    
                }
                
                if (brakeCommandDetected == true)
                {
                    executeBrakeMotionAnimation = true;                    
                }     
                
                if (forwardNudgeCommandDetected == true)
                {
                    executeForwardNudgeMotionAnimation = true;                   
                }     
                
                if (backwardNudgeCommandDetected == true)
                {
                    executeBackwardNudgeMotionAnimation = true;                   
                }       
            } 
            
            if (aValidCommandWasDetected == false)
            {
                if (debuggingInitialFirmwareStartup == true)
                {
                    /*-*/cout << "a valid command was not detected - ending this arduino incoming command check then...\n";
                    /*-*/system("Pause");                                
                }
                
                arduinoIncomingCommandCheckCompleted = true;                             
            }                                                           
        }
        
        
        
        
        
        
        
        if (executeForwardMotionAnimation == true)
        {
            executeForwardMotionAnimation = false;
            
            
            if (debuggingInitialFirmwareStartup == true)
            {
                /*-*/cout << "executing forward motion animation...\n";
                /*-*/system("Pause");                                
            }
            
            
            initiateForwardMotorEngagementCommutationStep = true;
            
            forwardMotionAnimationWaitingForForwardMotorEngagementCommutationStepToEngage = true;                                  
        }
        
        
        

        
        
        
        if (initiateForwardMotorEngagementCommutationStep == true)
        {
            initiateForwardMotorEngagementCommutationStep = false;
            
                                                              
        }
        
        
        
        
        
        
        
        if (forwardMotionAnimationWaitingForForwardMotorEngagementCommutationStepToEngage == true && forwardMotorEngagementCommutationStepInitiationCompleted == true)
        {
            forwardMotionAnimationWaitingForForwardMotorEngagementCommutationStepToEngage = false;
            
            forwardMotorEngagementCommutationStepInitiationCompleted = false;
            
            
                                                                                          
        }
        
        
        
        
        
        
        
        if (executeReverseMotionAnimation == true)
        {
            executeReverseMotionAnimation = false;
            
            
            if (debuggingInitialFirmwareStartup == true)
            {
                /*-*/cout << "executing reverse motion animation...\n";
                /*-*/system("Pause");                                
            }
            
                                              
        }
        
        
        
        
        
        
        
        if (executeBrakeMotionAnimation == true)
        {
            executeBrakeMotionAnimation = false;
            
            
            if (debuggingInitialFirmwareStartup == true)
            {
                /*-*/cout << "executing brake motion animation...\n";
                /*-*/system("Pause");                                
            }   
            
                                     
        }
        
        
        
        
        
        
        
        if (executeForwardNudgeMotionAnimation == true)
        {
            executeForwardNudgeMotionAnimation = false;
            
            
            if (debuggingInitialFirmwareStartup == true)
            {
                /*-*/cout << "executing forward nudge motion animation...\n";
                /*-*/system("Pause");                                
            } 
                                        
        }
        
        
        
        
        
        
        
        if (executeBackwardNudgeMotionAnimation == true)
        {
            executeBackwardNudgeMotionAnimation = false;
            
            
            if (debuggingInitialFirmwareStartup == true)
            {
                /*-*/cout << "executing backward motion animation...\n";
                /*-*/system("Pause");                                
            } 
            
                                        
        }
        
        
        
        
        
        
        
        if (initialEscFirmwareStartupWaitingForArduinoIncomingCommandCheck == true && arduinoIncomingCommandCheckCompleted == true)
        {
            initialEscFirmwareStartupWaitingForArduinoIncomingCommandCheck = false;
            
            arduinoIncomingCommandCheckCompleted = false;
            
            
            if (debuggingInitialFirmwareStartup == true)
            {
                /*-*/cout << "firware startup arduino incoming command check done...\n";
                /*-*/system("Pause");                                
            } 
            
            
            checkForArduinoIncomingCommand = true;
            
            initialEscFirmwareStartupWaitingForArduinoIncomingCommandCheck = true;                                                               
        }
        
        
        
        
        
        
        /*
        if (checkForArduinoIncomingCommand == true)
        {
            checkForArduinoIncomingCommand = false;
            
            
            
            //Arduino sends:
                      
            //ATTENTION
            //COMMAND
            //DATA LENGTH
            //DATA
            //CHECKSUM
            //RESPONSE REQUEST          
            
            
            //ESC's receive code has very explicit jobs:
            
            //WAIT FOR ATTENTION
            //RECEIVE COMMAND
            //RECEIVE DATA
            //RECEIVE CHECKSUM
            //VERIFY
            //EXECUTE
            //RESPONSE REQUESTED?
            //      / \
            //    NO   YES
            // WAIT   RELEASE WIRE
            //       SEND RESPONSE
            //         WAIT               
            
            
            
            //note: default is low and high indicates arduino is talking to us but attention is 200us of high pulse
            //followed by low pulse 200us followed by high pulse 500us followed by low pulse 200us that's our signal
            //to start listening as a command is immediately to follow that attention signal
            
            
            grabCommunicationSignalHighPulse = true;
            
            initialEscFirmwareStartupWaitingForGrabbingInitialAttentionSignalHighPulse = true;
        }
        
        
        
        
        
        
        
        if (grabCommunicationSignalHighPulse == true)
        {
            grabCommunicationSignalHighPulse = false;
            
            
            communicationSignalPinIsHigh = false;
            
            if (P0 & 0x10)
            {
                // P0.4 is HIGH
                
                communicationSignalPinIsHigh = true;
            }       
            
            
            if (communicationSignalPinIsHigh == false)
            {
                //since pin is low this is likely just default not talking state waiting for talking to start
                //meaning arduino is saying something - however if its low between highs ie he happens to
                //be in the middle of some command we're not gonna know about it have to have him repeat it
                //I guess since it won't pass our specific attention pattern of 200us high, 200us low, 500us high
                //we are looking for
                
                grabCommunicationSignalHighPulse = true;
            }
            
            if (communicationSignalPinIsHigh == true)
            {
                highPulseGrabbingCompleted = true;
            }                                     
        }
        
        
        
        
        
        
        
        if (initialEscFirmwareStartupWaitingForGrabbingInitialAttentionSignalHighPulse == true && highPulseGrabbingCompleted == true)
        {
            initialEscFirmwareStartupWaitingForGrabbingInitialAttentionSignalHighPulse = false;
            
            highPulseGrabbingCompleted = false;
            
            
            //mark time of first high beginning point
                
            systemTime = ((unsigned int)TMR2H << 8) | TMR2L;
            
            firstAttentionSignalHighPulseStartTime = systemTime;
            
            
            
            grabLowCommunicationSignalPulse = true;          
            
            initialEscFirmwareStartupWaitingForGrabbingAttentionSignalLowPulse = true;                                                     
        }
        
        
        
        
        
        
        
        if (grabLowCommunicationSignalPulse == true)
        {
            grabLowCommunicationSignalPulse = false;
            
            
            communicationSignalPinIsHigh = false;
            
            if (P0 & 0x10)
            {
                // P0.4 is HIGH
                
                communicationSignalPinIsHigh = true;
            }       
            
            
            if (communicationSignalPinIsHigh == false)
            {
                lowPulseGrabbingCompleted = true;
            }
            
            if (communicationSignalPinIsHigh == true)
            {
                grabLowCommunicationSignalPulse = true;
            }                                        
        }
        
        
        
        
        
        
        
        if (initialEscFirmwareStartupWaitingForGrabbingAttentionSignalLowPulse == true && lowPulseGrabbingCompleted == true)
        {
            initialEscFirmwareStartupWaitingForGrabbingAttentionSignalLowPulse = false;
            
            lowPulseGrabbingCompleted = false;
            
            
            //mark time of first high beginning point
                
            systemTime = ((unsigned int)TMR2H << 8) | TMR2L;
            
            firstAttentionSignalHighPulseEndTime = systemTime;
            
            attentionSignalLowPulseStartTime = systemTime;
            
            

            durationOfFirstAttentionSignalHighPulse = firstAttentionSignalHighPulseEndTime - firstAttentionSignalHighPulseStartTime;
            
            
            firstAttentionSignalHighPulseIsValid = false;
            
            if (durationOfFirstAttentionSignalHighPulse >= 150 && durationOfFirstAttentionSignalHighPulse <= 250)
            {
                firstAttentionSignalHighPulseIsValid = true;
            }
            
            
            if (firstAttentionSignalHighPulseIsValid == true)
            {
                grabCommunicationSignalHighPulse = true;
            
                initialEscFirmwareStartupWaitingForGrabbingSecondAttentionSignalHighPulse = true;                                         
            }
            
            if (firstAttentionSignalHighPulseIsValid == false)
            {
                checkForArduinoIncomingCommand = true;                                         
            }
        }
        
        
        
        
        
        
        
        if (initialEscFirmwareStartupWaitingForGrabbingSecondAttentionSignalHighPulse == true && highPulseGrabbingCompleted == true)
        {
            initialEscFirmwareStartupWaitingForGrabbingSecondAttentionSignalHighPulse = false;
            
            highPulseGrabbingCompleted = false;
            
            
            //mark time of second high beginning point
                
            systemTime = ((unsigned int)TMR2H << 8) | TMR2L;
            
            secondAttentionSignalHighPulseStartTime = systemTime;
            
            attentionSignalLowPulseEndTime = systemTime;
            
            
            
            durationOfAttentionSignalLowPulse = attentionSignalLowPulseEndTime - attentionSignalLowPulseStartTime;
            
            
            
            //attention signal is: normally low..... high pulse 200 > low pulse 500 > high pulse 200
            
            attentionSignalLowPulseIsValid = false;
            
            if (durationOfAttentionSignalLowPulse >= 450 && durationOfAttentionSignalLowPulse <= 550)
            {
                attentionSignalLowPulseIsValid = true;
            }
            
            
            if (attentionSignalLowPulseIsValid == true)
            {
                grabCommunicationSignalHighPulse = true;
            
                initialEscFirmwareStartupWaitingForGrabbingEndOfSecondAttentionSignalHighPulse = true;                                         
            }
            
            if (attentionSignalLowPulseIsValid == false)
            {
                checkForArduinoIncomingCommand = true;                                         
            }                                                 
        }
        
        
        
        
        
        
        
        if (initialEscFirmwareStartupWaitingForGrabbingEndOfSecondAttentionSignalHighPulse == true && highPulseGrabbingCompleted == true)
        {
            initialEscFirmwareStartupWaitingForGrabbingEndOfSecondAttentionSignalHighPulse = false;
            
            highPulseGrabbingCompleted = false;
            
            
            //mark time of second high ending point
                
            systemTime = ((unsigned int)TMR2H << 8) | TMR2L;
            
            secondAttentionSignalHighPulseEndTime = systemTime;
            
            
            
            
            durationOfAttentionSignalHighPulse = secondAttentionSignalHighPulseEndTime - secondAttentionSignalHighPulseStartTime;
            
            
            
            //attention signal is: normally low..... high pulse 200 > low pulse 500 > high pulse 200
            
            attentionSignalHighPulseIsValid = false;
            
            if (durationOfAttentionSignalHighPulse >= 150 && durationOfAttentionSignalHighPulse <= 250)
            {
                attentionSignalHighPulseIsValid = true;
            }
            
            
            if (attentionSignalHighPulseIsValid == true)
            {
                we now acvtively listen for a command from arduino having successfully grabbed a valid attention pulse sequence                                   
            }
            
            if (attentionSignalHighPulseIsValid == false)
            {
                checkForArduinoIncomingCommand = true;                                         
            }                                                 
        }
        
        
        
        
        if (initialEscFirmwareStartupWaitingForArduinoIncomingCommandCheck == true && arduinoIncomingCommandCheckCompleted == true)
        {
            initialEscFirmwareStartupWaitingForArduinoIncomingCommandCheck = false;
            
            arduinoIncomingCommandCheckCompleted = false;
            
            
                                                                  
        }
        
        
        
        */
        
        
        
        
        
        
        
        
        
        
        if (turnOnFirstCommutationStep == true)
        {
            turnOnFirstCommutationStep = false;
            
            
            phaseOneHighsideMosfet = 1;
            phaseOneLowsideMosfet = 0;
            
            phaseTwoHighsideMosfet = 0;
            phaseTwoLowsideMosfet = 1;
            
            phaseThreeHighsideMosfet = 0;
            phaseThreeLowsideMosfet = 0;
            
            
            turnOnSpecificRegisterLocationsOfMCU = true;
            
            turningOnFirstCommutationStepWaitingForTurningOnSpecificRegisterLocationsOfMCU = true;                               
        }
        
        
        
        
        
        
        
        if (turnOnSpecificRegisterLocationsOfMCU == true)
        {
            turnOnSpecificRegisterLocationsOfMCU = false;
            
            
            //-/P1 = 0;

            if (phaseOneHighsideMosfet == 1)
            {
            //-/    P1 |= 0x01;                       
            } 
            
            if (phaseTwoHighsideMosfet == 1)
            {
            //-/    P1 |= 0x02;                       
            } 
            
            if (phaseThreeHighsideMosfet == 1)
            {
            //-/    P1 |= 0x04;                         
            } 
            
            
            if (phaseOneLowsideMosfet == 1)
            {
            //-/    P1 |= 0x08;                      
            } 
            
            if (phaseTwoLowsideMosfet == 1)
            {
            //-/    P1 |= 0x10;                      
            } 
            
            if (phaseThreeLowsideMosfet == 1)
            {
            //-/    P1 |= 0x20;                        
            } 
            
            
            turningOnSpecificRegisterLocationsOfMCUCompleted = true;                                         
        }
        
        
        
        
        
        
        
        if (turningOnFirstCommutationStepWaitingForTurningOnSpecificRegisterLocationsOfMCU == true && turningOnSpecificRegisterLocationsOfMCUCompleted == true)
        {
            turningOnFirstCommutationStepWaitingForTurningOnSpecificRegisterLocationsOfMCU = false;
            
            turningOnSpecificRegisterLocationsOfMCUCompleted = false;
            
            
            
            turningOnFirstCommutationStepCompleted = true;                                                                               
        }
        
        
        
        
        
        
        
        if (escFirmwareWaitingForTurningOnFirstCommutationStep == true && turningOnFirstCommutationStepCompleted == true)
        {
            escFirmwareWaitingForTurningOnFirstCommutationStep = false;
            
            turningOnFirstCommutationStepCompleted = false;
            
            
                                                                  
        }
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
    }
}











//nextSection                    interrupt for communication pin change detection and recording in realtime section

void communicationInterrupt(void) //-/interrupt 0
{
    communicationsSignalPinTurnedHigh = false;
    
    //-/if (P0 & 0x10)
    //-/{
    //-/    communicationsSignalPinTurnedHigh = true;
    //-/}
    
    
    
    if (communicationsSignalPinTurnedHigh == true)
    {
    //-/    IT01CF &= 0xF7;    // next interrupt = falling edge  
    }
    
    if (communicationsSignalPinTurnedHigh == false)
    {
    //-/    IT01CF |= 0x08;    // next interrupt = rising edge                                                                  
    }
    
    
    runInterruptCodeMainWhileLoop = true;
    
    seeIfThisIsFirstEverRecordedHighSignal = true;
    interruptCodeSectionWaitingToSeeIfThisIsFirstEverRecordedHighSignal = true;
    
    
    while (runInterruptCodeMainWhileLoop == true)
    {
        if (seeIfThisIsFirstEverRecordedHighSignal == true)
        {
            seeIfThisIsFirstEverRecordedHighSignal = false;
            
            
            if (highSignalRecordedPreviously == false && communicationsSignalPinTurnedHigh == true)
            {
                highSignalRecordedPreviously = true;      
                
                
                
                weAreHighSignalAsOfLastKnownStateFromLastTimeWeCheckedOnInterrupts = true;                     
            }  
            
            
            seeingIfThisIsFirstEverRecordedHighSignalCompleted = true;                                         
        }    
        
        
        
        
        
        
        
        if (interruptCodeSectionWaitingToSeeIfThisIsFirstEverRecordedHighSignal == true && seeingIfThisIsFirstEverRecordedHighSignalCompleted == true)
        {
            interruptCodeSectionWaitingToSeeIfThisIsFirstEverRecordedHighSignal = false;
            
            seeingIfThisIsFirstEverRecordedHighSignalCompleted = false;
            
            
            if (weAreHighSignalAsOfLastKnownStateFromLastTimeWeCheckedOnInterrupts == true && communicationsSignalPinTurnedHigh == true)
            {
                runInterruptCodeMainWhileLoop = false; //do nothing since we are high state and last known was also high state so its first time through...                                                                    
            } 
            
            if (weAreHighSignalAsOfLastKnownStateFromLastTimeWeCheckedOnInterrupts == true && communicationsSignalPinTurnedHigh == false)
            {
                markInitialLowSignalTriggerPointTimingForFutureReference = true;
                
                interruptCodeSectionWaitingForMarkingInitialLowSignalTriggerPointTimingForFutureReference = true;                                                                
            } 
            
            if (weAreHighSignalAsOfLastKnownStateFromLastTimeWeCheckedOnInterrupts == false && communicationsSignalPinTurnedHigh == true)
            {
                markTimeOfThisEndingLowSignalPointAndDecipherDurationToDetermineHighOrLowBitMeaning = true;
                
                interruptCodeSectionWaitingForMarkingTimeOfThisLowSignalEndingPointAndStoringBitMeaning = true;                                                                   
            }    
            
            if (weAreHighSignalAsOfLastKnownStateFromLastTimeWeCheckedOnInterrupts == false && communicationsSignalPinTurnedHigh == false)
            {
                runInterruptCodeMainWhileLoop = false; //do nothing since we are low state and last known was also low state - this 
                                                        //should never actually happen since interrupts would not be called if this were
                                                         //the case so this would be a faulty state some bug in my code if this ever ran oh well...
            }                                                                      
        }  
        
        
        
        
        
        
        
        if (markInitialLowSignalTriggerPointTimingForFutureReference == true)
        {
            markInitialLowSignalTriggerPointTimingForFutureReference = false;
            
                
            
            //mark time of low pulse beginning point
                
            //-/systemTime = ((unsigned int)TMR2H << 8) | TMR2L;
            
            //-/lowPulseStartTime = systemTime; 
            
            weAreHighSignalAsOfLastKnownStateFromLastTimeWeCheckedOnInterrupts = false;
            
            
            markingInitialLowTriggerPointTimingForFutureReferenceCompleted = true;                                                        
        }
        
        
        
        
        
        
        
        if (interruptCodeSectionWaitingForMarkingInitialLowSignalTriggerPointTimingForFutureReference == true && markingInitialLowTriggerPointTimingForFutureReferenceCompleted == true)
        {
            interruptCodeSectionWaitingForMarkingInitialLowSignalTriggerPointTimingForFutureReference = false;
            
            markingInitialLowTriggerPointTimingForFutureReferenceCompleted = false;
            
            
            runInterruptCodeMainWhileLoop = false;                                                                                         
        }
        
        
        
        
        
        
        
        if (markTimeOfThisEndingLowSignalPointAndDecipherDurationToDetermineHighOrLowBitMeaning == true)
        {
            markTimeOfThisEndingLowSignalPointAndDecipherDurationToDetermineHighOrLowBitMeaning = false;
            
            
            
            //mark time of low ending point
            
            //-/systemTime = ((unsigned int)TMR2H << 8) | TMR2L;
            
            //-/lowPulseEndTime = systemTime;
            
            
            
            durationOfLowPulse = lowPulseEndTime - lowPulseStartTime;
            
            
            if (durationOfLowPulse >= 0 && durationOfLowPulse < 100)
            {
                //short/standard low pulse duration
                
                receiveBuffer[receiveBufferWritePosition] = 0;
        
                receiveBufferWritePosition = receiveBufferWritePosition + 1;
                
                
                if (receiveBufferWritePosition >= 256)
                {
                    receiveBufferWritePosition = 0;
                }
            }
            
            if (durationOfLowPulse >= 100)
            {
                //long low pulse duration
                
                receiveBuffer[receiveBufferWritePosition] = 1;
        
                receiveBufferWritePosition = receiveBufferWritePosition + 1;
                
                
                if (receiveBufferWritePosition >= 256)
                {
                    receiveBufferWritePosition = 0;
                }
            }
            
            
            
            storingLowBitEndingMeaningCompleted = true;                                                                                       
        }
        
        
        
        
        
        
        
        if (interruptCodeSectionWaitingForMarkingTimeOfThisLowSignalEndingPointAndStoringBitMeaning == true && storingLowBitEndingMeaningCompleted == true)
        {
            interruptCodeSectionWaitingForMarkingTimeOfThisLowSignalEndingPointAndStoringBitMeaning = false;
            
            storingLowBitEndingMeaningCompleted = false;
            
            
            weAreHighSignalAsOfLastKnownStateFromLastTimeWeCheckedOnInterrupts = true;   
            
            
            runInterruptCodeMainWhileLoop = false;                                                                                      
        }
        
        
        
        
                        
    }
    
    
    
    
    
    
    
}
        
        
        
        
        
        
        
        
        
        
        
        
        
