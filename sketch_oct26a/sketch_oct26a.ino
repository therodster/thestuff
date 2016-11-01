#include <SPI.h>  
#include <Pixy.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();  
Adafruit_DCMotor* leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor* rightMotor = AFMS.getMotor(2);

#define X_CENTER    160L    //Taken from the Pixy CMUcam5 code
#define Y_CENTER    100L
#define RCS_MIN_POS     0L
#define RCS_MAX_POS     1000L
#define RCS_CENTER_POS  ((RCS_MAX_POS-RCS_MIN_POS)/2)

class ServoLoop
{
public:
  ServoLoop(int32_t proportionalGain, int32_t derivativeGain);

  void update(int32_t error);

  int32_t m_pos;
  int32_t m_prevError;
  int32_t m_proportionalGain;
  int32_t m_derivativeGain;
};

ServoLoop::ServoLoop(int32_t proportionalGain, int32_t derivativeGain)
{
  m_pos = RCS_CENTER_POS;
  m_proportionalGain = proportionalGain;
  m_derivativeGain = derivativeGain;
  m_prevError = 0x80000000L;
}
void ServoLoop::update(int32_t error)
{
  long int velocity;
  char buf[32];
  if (m_prevError!=0x80000000)
  { 
    velocity = (error*m_proportionalGain + (error - m_prevError)*m_derivativeGain)>>10;

    m_pos += velocity;
    if (m_pos>RCS_MAX_POS) 
    {
      m_pos = RCS_MAX_POS; 
    }
    else if (m_pos<RCS_MIN_POS) 
    {
      m_pos = RCS_MIN_POS;
    }
  }
  m_prevError = error;
}

Pixy pixy;  
ServoLoop panLoop(200, 200);  // Servo loop for pan
ServoLoop tiltLoop(150, 200); // Servo loop for tilt


void setup()
{
  Serial.begin(9600);
  Serial.print("Starting...\n");

  pixy.init();

 leftMotor->setSpeed(0);
 rightMotor->setSpeed(0);

 leftMotor->run(RELEASE);
 rightMotor->run(RELEASE);
}

uint32_t lastBlockTime = 0;

void loop()
{ 
  uint16_t blocks;
  blocks = pixy.getBlocks();

  // If we have blocks in sight, track and follow them
  if (blocks)
  {
    int trackedBlock = TrackBlock(blocks);
    FollowBlock(trackedBlock);
    lastBlockTime = millis();
  }  
  else if (millis() - lastBlockTime > 100)
  {
    leftMotor->setSpeed(255);
    leftMotor->run(FORWARD);

    rightMotor->setSpeed(255);
    rightMotor->run(FORWARD);
    
    ScanForBlocks();
  }
}

int oldX, oldY, oldSignature;

int TrackBlock(int blockCount)
{
  int trackedBlock = 0;
  long maxSize = 0;

  Serial.print("blocks =");
  Serial.println(blockCount);

  for (int i = 0; i < blockCount; i++)
  {
    if ((oldSignature == 0) || (pixy.blocks[i].signature == oldSignature))
    {
      long newSize = pixy.blocks[i].height * pixy.blocks[i].width;
      if (newSize > maxSize)
      {
        trackedBlock = i;
        maxSize = newSize;
      }
    }
  }

  int32_t panError = X_CENTER - pixy.blocks[trackedBlock].x;
  int32_t tiltError = pixy.blocks[trackedBlock].y - Y_CENTER;

  panLoop.update(panError);
  tiltLoop.update(tiltError);

  pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);

  oldX = pixy.blocks[trackedBlock].x;
  oldY = pixy.blocks[trackedBlock].y;
  oldSignature = pixy.blocks[trackedBlock].signature;
  return trackedBlock;
}

int32_t size = 400;
void FollowBlock(int trackedBlock)
{
  int32_t followError = RCS_CENTER_POS - panLoop.m_pos;  // How far off-center are we looking now?

  size += pixy.blocks[trackedBlock].width * pixy.blocks[trackedBlock].height; 
  size -= size >> 3;

  int forwardSpeed = constrain(400 - (size/256), -100, 400);  

  int32_t differential = (followError + (followError * forwardSpeed))>>8;

  int leftSpeed = constrain(forwardSpeed + differential, -400, 400);
  int rightSpeed = constrain(forwardSpeed - differential, -400, 400);

  leftMotor->setSpeed(100);
  rightMotor->setSpeed(100);
}

int scanIncrement = (RCS_MAX_POS - RCS_MIN_POS) / 150;
uint32_t lastMove = 0;

void ScanForBlocks()
{
  if (millis() - lastMove > 20)
  {
    lastMove = millis();
    panLoop.m_pos += scanIncrement;
    if ((panLoop.m_pos >= RCS_MAX_POS)||(panLoop.m_pos <= RCS_MIN_POS))
    {
      tiltLoop.m_pos = random(RCS_MAX_POS * 0.6, RCS_MAX_POS);
      scanIncrement = -scanIncrement;
      if (scanIncrement < 0)
      {
        leftMotor->run(RELEASE);
        delay(1000);
        rightMotor->run(FORWARD);
        delay(1000);
      }
      else
      {
        leftMotor->run(FORWARD);
        delay(1000);
        rightMotor->run(RELEASE);
        delay(1000);
      }
      delay(random(250, 500));
    }

    pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
  }
}
