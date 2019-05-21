//***********************************************************************
// Matlab .fis to arduino C converter v2.0.1.25122016                   
// - Karthik Nadig, USA                                                  
// Please report bugs to:                                                
// https://github.com/karthiknadig/ArduinoFIS/issues                     
// If you don't have a GitHub account mail to karthiknadig@gmail.com     
//***********************************************************************

#include "fis_header.h"
#include<AFMotor.h>

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

#define trigPin A1
#define echoPin A0

#define trigPin2 A3
#define echoPin2 A2

#define trigPin3 A5
#define echoPin3 A4

// Number of inputs to the fuzzy inference system
const int fis_gcI = 3;
// Number of outputs to the fuzzy inference system
const int fis_gcO = 2;
// Number of rules to the fuzzy inference system
const int fis_gcR = 16;

FIS_TYPE g_fisInput[fis_gcI];
FIS_TYPE g_fisOutput[fis_gcO];

// Setup routine runs once when you press reset:
void setup()
{
  Serial.begin(9600);
    // initialize the Analog pins for input.
    // Pin mode for Input: FrontProximity
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin , INPUT);
    // Pin mode for Input: LeftProximity
    pinMode(trigPin2, OUTPUT);
    pinMode(echoPin2 , INPUT);
    // Pin mode for Input: RightProximity
    pinMode(trigPin3, OUTPUT);
    pinMode(echoPin3 , INPUT);

    setMotorVelocities(200,200);

    // initialize the Analog pins for output.
    // Pin mode for Output: LeftVelocity
    //pinMode(3 , OUTPUT);
    // Pin mode for Output: RightVelocity
    //pinMode(4 , OUTPUT);

}

float first(){
  float duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
 
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW); 
  
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) * 0.0344;
  
  if (distance >= 400 || distance <= 2){
    Serial.println("=============");
    Serial.print("Distance1 = ");
    Serial.println("Out of range");
  }
  else {
    Serial.println("=============");
    Serial.print("Distance1 = ");
    Serial.print(distance);
    Serial.println(" cm");
    delay(500);
  }
  return distance;
}

float second(){
   float duration2, distance2;
  digitalWrite(trigPin2, LOW); 
  delayMicroseconds(2);
 
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW); 

  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = (duration2 / 2) * 0.0344;
  if (distance2 >= 400 || distance2 <= 2){
    Serial.print("Distance2 = ");
    Serial.println("Out of range");
  }
  else {
    Serial.print("Distance2 = ");
    Serial.print(distance2);
    Serial.println(" cm");
    delay(500);
  } 
  return distance2;
}
float third(){
   float duration3, distance3;
  digitalWrite(trigPin3, LOW); 
  delayMicroseconds(2);
 
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW); 

  duration3 = pulseIn(echoPin3, HIGH);
  distance3 = (duration3 / 2) * 0.0344;
  if (distance3 >= 400 || distance3 <= 2){
    Serial.print("Distance3 = ");
    Serial.println("Out of range");
  }
  else {
    Serial.print("Distance3 = ");
    Serial.print(distance3);
    Serial.println(" cm");
    delay(500);
  } 
  return distance3;
}

// Loop routine runs over and over again forever:
void loop()
{
    // Read Input: FrontProximity
    g_fisInput[0] = first();
    // Read Input: LeftProximity
    g_fisInput[1] = second();
    // Read Input: RightProximity
    g_fisInput[2] = third();

    g_fisOutput[0] = 0;
    g_fisOutput[1] = 0;

    fis_evaluate();

    Serial.print("Velocity left: ");
    Serial.println(g_fisOutput[0]);

    Serial.print("Velocity right: ");
    Serial.println(g_fisOutput[1]);
    Serial.println("=============");
  
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
  
    setMotorVelocities(g_fisOutput[0],g_fisOutput[1]);
  
    // Set output vlaue: LeftVelocity
    //analogWrite(3 , g_fisOutput[0]);
    // Set output vlaue: RightVelocity
    //analogWrite(4 , g_fisOutput[1]);
}
void setMotorVelocities(float left,float right){
    //Serial.print("LeftVel = ");
    //Serial.print(left);
    //Serial.println(" ");

    //Serial.print("RightVel = ");
    //Serial.print(right);
    //Serial.println(" ");
    motor1.setSpeed(right);
    motor2.setSpeed(right);
    motor3.setSpeed(left);
    motor4.setSpeed(left);
  
  if(right <0){
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
  }
  if(left < 0){
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
  }
  if(left < 0 && right < 0){
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
  }
}
//***********************************************************************
// Support functions for Fuzzy Inference System                          
//***********************************************************************
// Trapezoidal Member Function
FIS_TYPE fis_trapmf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2], d = p[3];
    FIS_TYPE t1 = ((x <= c) ? 1 : ((d < x) ? 0 : ((c != d) ? ((d - x) / (d - c)) : 0)));
    FIS_TYPE t2 = ((b <= x) ? 1 : ((x < a) ? 0 : ((a != b) ? ((x - a) / (b - a)) : 0)));
    return (FIS_TYPE) min(t1, t2);
}

FIS_TYPE fis_prod(FIS_TYPE a, FIS_TYPE b)
{
    return (a * b);
}

FIS_TYPE fis_probor(FIS_TYPE a, FIS_TYPE b)
{
    return (a + b - (a * b));
}

FIS_TYPE fis_sum(FIS_TYPE a, FIS_TYPE b)
{
    return (a + b);
}

FIS_TYPE fis_array_operation(FIS_TYPE *array, int size, _FIS_ARR_OP pfnOp)
{
    int i;
    FIS_TYPE ret = 0;

    if (size == 0) return ret;
    if (size == 1) return array[0];

    ret = array[0];
    for (i = 1; i < size; i++)
    {
        ret = (*pfnOp)(ret, array[i]);
    }

    return ret;
}


//***********************************************************************
// Data for Fuzzy Inference System                                       
//***********************************************************************
// Pointers to the implementations of member functions
_FIS_MF fis_gMF[] =
{
    fis_trapmf
};

// Count of member function for each Input
int fis_gIMFCount[] = { 3, 3, 3 };

// Count of member function for each Output 
int fis_gOMFCount[] = { 4, 4 };

// Coefficients for the Input Member Functions
FIS_TYPE fis_gMFI0Coeff1[] = { 0, 15, 40, 60 };
FIS_TYPE fis_gMFI0Coeff2[] = { 50, 72.6, 106.2, 128 };
FIS_TYPE fis_gMFI0Coeff3[] = { 120.2, 157, 300, 399 };
FIS_TYPE* fis_gMFI0Coeff[] = { fis_gMFI0Coeff1, fis_gMFI0Coeff2, fis_gMFI0Coeff3 };
FIS_TYPE fis_gMFI1Coeff1[] = { 0, 15, 30, 50 };
FIS_TYPE fis_gMFI1Coeff2[] = { 40, 54.09, 83.39, 114.4 };
FIS_TYPE fis_gMFI1Coeff3[] = { 101.8, 126, 300, 399 };
FIS_TYPE* fis_gMFI1Coeff[] = { fis_gMFI1Coeff1, fis_gMFI1Coeff2, fis_gMFI1Coeff3 };
FIS_TYPE fis_gMFI2Coeff1[] = { 0, 15, 30, 50 };
FIS_TYPE fis_gMFI2Coeff2[] = { 40, 54.09, 83.39, 114.4 };
FIS_TYPE fis_gMFI2Coeff3[] = { 101.8, 126, 300, 399 };
FIS_TYPE* fis_gMFI2Coeff[] = { fis_gMFI2Coeff1, fis_gMFI2Coeff2, fis_gMFI2Coeff3 };
FIS_TYPE** fis_gMFICoeff[] = { fis_gMFI0Coeff, fis_gMFI1Coeff, fis_gMFI2Coeff };

// Coefficients for the Output Member Functions
FIS_TYPE fis_gMFO0Coeff1[] = { 0, 0, 0, 100 };
FIS_TYPE fis_gMFO0Coeff2[] = { 0, 0, 0, 160 };
FIS_TYPE fis_gMFO0Coeff3[] = { 0, 0, 0, 200 };
FIS_TYPE fis_gMFO0Coeff4[] = { 0, 0, 0, -240 };
FIS_TYPE* fis_gMFO0Coeff[] = { fis_gMFO0Coeff1, fis_gMFO0Coeff2, fis_gMFO0Coeff3, fis_gMFO0Coeff4 };
FIS_TYPE fis_gMFO1Coeff1[] = { 0, 0, 0, -240 };
FIS_TYPE fis_gMFO1Coeff2[] = { 0, 0, 0, 100 };
FIS_TYPE fis_gMFO1Coeff3[] = { 0, 0, 0, 160 };
FIS_TYPE fis_gMFO1Coeff4[] = { 0, 0, 0, 200 };
FIS_TYPE* fis_gMFO1Coeff[] = { fis_gMFO1Coeff1, fis_gMFO1Coeff2, fis_gMFO1Coeff3, fis_gMFO1Coeff4 };
FIS_TYPE** fis_gMFOCoeff[] = { fis_gMFO0Coeff, fis_gMFO1Coeff };

// Input membership function set
int fis_gMFI0[] = { 0, 0, 0 };
int fis_gMFI1[] = { 0, 0, 0 };
int fis_gMFI2[] = { 0, 0, 0 };
int* fis_gMFI[] = { fis_gMFI0, fis_gMFI1, fis_gMFI2};

// Output membership function set

int* fis_gMFO[] = {};

// Rule Weights
FIS_TYPE fis_gRWeight[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Type
int fis_gRType[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Inputs
int fis_gRI0[] = { 1, 1, 1 };
int fis_gRI1[] = { 1, 1, 3 };
int fis_gRI2[] = { 2, 2, 3 };
int fis_gRI3[] = { 2, 1, 3 };
int fis_gRI4[] = { 2, 2, 2 };
int fis_gRI5[] = { 2, 1, 2 };
int fis_gRI6[] = { 3, 3, 3 };
int fis_gRI7[] = { 1, 2, 3 };
int fis_gRI8[] = { 1, 1, 0 };
int fis_gRI9[] = { 0, 1, 0 };
int fis_gRI10[] = { 2, 2, 1 };
int fis_gRI11[] = { 2, 3, 1 };
int fis_gRI12[] = { 1, 2, 1 };
int fis_gRI13[] = { 1, 0, 1 };
int fis_gRI14[] = { 0, 0, 1 };
int fis_gRI15[] = { 1, 0, 0 };
int* fis_gRI[] = { fis_gRI0, fis_gRI1, fis_gRI2, fis_gRI3, fis_gRI4, fis_gRI5, fis_gRI6, fis_gRI7, fis_gRI8, fis_gRI9, fis_gRI10, fis_gRI11, fis_gRI12, fis_gRI13, fis_gRI14, fis_gRI15 };

// Rule Outputs
int fis_gRO0[] = { 4, 3 };
int fis_gRO1[] = { 3, 2 };
int fis_gRO2[] = { 2, 2 };
int fis_gRO3[] = { 3, 2 };
int fis_gRO4[] = { 1, 2 };
int fis_gRO5[] = { 3, 2 };
int fis_gRO6[] = { 3, 4 };
int fis_gRO7[] = { 1, 4 };
int fis_gRO8[] = { 1, 4 };
int fis_gRO9[] = { 3, 2 };
int fis_gRO10[] = { 2, 1 };
int fis_gRO11[] = { 1, 4 };
int fis_gRO12[] = { 3, 2 };
int fis_gRO13[] = { 1, 3 };
int fis_gRO14[] = { 4, 3 };
int fis_gRO15[] = { 4, 3 };
int* fis_gRO[] = { fis_gRO0, fis_gRO1, fis_gRO2, fis_gRO3, fis_gRO4, fis_gRO5, fis_gRO6, fis_gRO7, fis_gRO8, fis_gRO9, fis_gRO10, fis_gRO11, fis_gRO12, fis_gRO13, fis_gRO14, fis_gRO15 };

// Input range Min
FIS_TYPE fis_gIMin[] = { 0, 0, 0 };

// Input range Max
FIS_TYPE fis_gIMax[] = { 300, 300, 300 };

// Output range Min
FIS_TYPE fis_gOMin[] = { 0, 0 };

// Output range Max
FIS_TYPE fis_gOMax[] = { 200, 200 };

//***********************************************************************
// Data dependent support functions for Fuzzy Inference System           
//***********************************************************************
// None for Sugeno

//***********************************************************************
// Fuzzy Inference System                                                
//***********************************************************************
void fis_evaluate()
{
    FIS_TYPE fuzzyInput0[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput1[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput2[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyInput[fis_gcI] = { fuzzyInput0, fuzzyInput1, fuzzyInput2, };
    FIS_TYPE fuzzyOutput0[] = { 0, 0, 0, 0 };
    FIS_TYPE fuzzyOutput1[] = { 0, 0, 0, 0 };
    FIS_TYPE* fuzzyOutput[fis_gcO] = { fuzzyOutput0, fuzzyOutput1, };
    FIS_TYPE fuzzyRules[fis_gcR] = { 0 };
    FIS_TYPE fuzzyFires[fis_gcR] = { 0 };
    FIS_TYPE* fuzzyRuleSet[] = { fuzzyRules, fuzzyFires };
    FIS_TYPE sW = 0;

    // Transforming input to fuzzy Input
    int i, j, r, o;
    for (i = 0; i < fis_gcI; ++i)
    {
        for (j = 0; j < fis_gIMFCount[i]; ++j)
        {
            fuzzyInput[i][j] =
                (fis_gMF[fis_gMFI[i][j]])(g_fisInput[i], fis_gMFICoeff[i][j]);
        }
    }

    int index = 0;
    for (r = 0; r < fis_gcR; ++r)
    {
        if (fis_gRType[r] == 1)
        {
            fuzzyFires[r] = 1;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_prod(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_prod(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_prod(fuzzyFires[r], 1);
            }
        }
        else
        {
            fuzzyFires[r] = 0;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_probor(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_probor(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_probor(fuzzyFires[r], 0);
            }
        }

        fuzzyFires[r] = fis_gRWeight[r] * fuzzyFires[r];
        sW += fuzzyFires[r];
    }

    if (sW == 0)
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = ((fis_gOMax[o] + fis_gOMin[o]) / 2);
        }
    }
    else
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            FIS_TYPE sWI = 0.0;
            for (j = 0; j < fis_gOMFCount[o]; ++j)
            {
                fuzzyOutput[o][j] = fis_gMFOCoeff[o][j][fis_gcI];
                for (i = 0; i < fis_gcI; ++i)
                {
                    fuzzyOutput[o][j] += g_fisInput[i] * fis_gMFOCoeff[o][j][i];
                }
            }

            for (r = 0; r < fis_gcR; ++r)
            {
                index = fis_gRO[r][o] - 1;
                sWI += fuzzyFires[r] * fuzzyOutput[o][index];
            }

            g_fisOutput[o] = sWI / sW;
        }
    }
}
