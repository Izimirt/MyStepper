#include "MyStepper.h"

uint8_t MyStepper::interrupterStepMicros = 10;

#if defined(ESP32)

    hw_timer_t* MyStepper::interrupter = nullptr;

    MyStepper::MyStepper()
    {
        ptrOnOther = currentPtr;
        currentPtr = this;
        if(numSteppers == 0)
        {
            interrupter = timerBegin(0,240,true); //ESP32 240MHz
            timerAttachInterrupt(interrupter,&MyStepper::Step,true);
            timerAlarmWrite(interrupter,interrupterStepMicros,true);
            timerAlarmEnable(interrupter);
        }
        numSteppers++;
        ID = numSteppers;
    }

    void IRAM_ATTR MyStepper::Step()
    {
        unitPtr = currentPtr;

        while(unitPtr != nullptr)
        {
            if(unitPtr->moveFlag)
            {
                if(unitPtr->speed != 0)
                {
                    if(unitPtr->timer >= unitPtr->speed)
                    {
                        unitPtr->stepState = !unitPtr->stepState;
                        digitalWrite(unitPtr->stepPin,unitPtr->stepState);
                        unitPtr->currentStep++;
                        unitPtr->timer = 0;
                    }
                    else 
                        unitPtr->timer++;
                }
            }
            else
                unitPtr->timer = 0;

            unitPtr = unitPtr->ptrOnOther;
        }
    }

#else

    #include "GyverTimers.h"

    MyStepper::MyStepper()
    {
        ptrOnOther = currentPtr;
        currentPtr = this;
        numSteppers++;
        ID = numSteppers;
    }

    void MyStepper::Step()
    {
        unitPtr = currentPtr;

        while(unitPtr != nullptr)
        {
            if(unitPtr->moveFlag)
            {
                if(unitPtr->speed != 0)
                {
                    if(unitPtr->timer >= unitPtr->speed)
                    {
                        unitPtr->stepState = !unitPtr->stepState;
                        digitalWrite(unitPtr->stepPin,unitPtr->stepState);
                        unitPtr->currentStep++;
                        unitPtr->direction ? unitPtr->currentPoint++ : unitPtr->currentPoint--;
                        unitPtr->timer = 1;
                    }
                    else 
                        unitPtr->timer++;
                }
            }
            else
                unitPtr->timer = 1;

            unitPtr = unitPtr->ptrOnOther;
        }
    }

    void MyStepper::StartInterrupter()
    {
        Timer1.setPeriod(interrupterStepMicros*2);
        Timer1.enableISR(CHANNEL_B);
    }

    ISR(TIMER1_B)
    {
        MyStepper::Step();
    }

#endif

uint8_t MyStepper::numSteppers = 0;

MyStepper* MyStepper::currentPtr = nullptr;

MyStepper* MyStepper::unitPtr = nullptr;

MyStepper::MyStepper(uint16_t stepPin, uint16_t dirPin, uint16_t enPin, bool freeStay, void (*Error)(void*)) : MyStepper()
{
    SetEngine(stepPin,dirPin,enPin,freeStay,Error);
}

void MyStepper::SetEngine(uint16_t stepPin, uint16_t dirPin, uint16_t enPin, bool freeStay, void (*Error)(void*))
{
    this-> stepPin = stepPin;
    this-> dirPin = dirPin;
    this-> enPin = enPin;
    this-> freeStay = freeStay;
    this->Error = Error;
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enPin, OUTPUT);
    if(freeStay)
        digitalWrite(enPin, true);
    else
        digitalWrite(enPin, false);
}

void MyStepper::SetSteps(uint16_t distanceNumber, uint32_t steps, uint32_t understeps, uint32_t oversteps)
{
    if(understeps == 0)
        understeps = steps;
    SSet[distanceNumber].distance = steps;
    SSet[distanceNumber].understeps = understeps;
    SSet[distanceNumber].oversteps = oversteps;
}

void MyStepper::SetPoint(uint16_t pointNumber, int32_t point, int32_t understeps, int32_t oversteps)
{
    if(understeps == 0)
        understeps = point;
    PSet[pointNumber].point = point;
    PSet[pointNumber].understeps = abs(understeps);
    PSet[pointNumber].oversteps = abs(oversteps);
}

void MyStepper::SetMove(uint16_t minSpeed, uint16_t maxSpeed, uint16_t accelerationTimeMsec, uint8_t setNumber)
{
    MSet[setNumber].minSpeed = minSpeed;
    MSet[setNumber].maxSpeed = maxSpeed;
    MSet[setNumber].millisStep = (accelerationTimeMsec / (float)(minSpeed - maxSpeed));
    if(MSet[setNumber].millisStep < 1)
        MSet[setNumber].millisStep = 1;
} 

bool MyStepper::Move(bool direction, int8_t setNumber, int8_t distanceNumber, int8_t interrupter, bool soft_brake)
{
    if(manualFlag)
        return 0;

    if(stopFlag)
        return 0;    

    if(errorCommand != 0)
        return 0;

    if(finishFlag)
        return 1;

    if(interrupter > 0)
    {
        if(distanceNumber > -1 && (currentStep < (SSet[distanceNumber].distance - SSet[distanceNumber].understeps)))
        {
            Stop();
            errorCommand = SetError(UNDERSTEP);
            if(Error != nullptr)
                Error(this);
            return 0;
        }

        if(soft_brake)
        {
            brakeFlag = true;
            if(speed > MSet[setNumber].minSpeed)
            {
                Stop();
                brakeFlag = false;
                finishFlag = true;
                return 1;
            }
        }
        else
        {            
            Stop();
            finishFlag = true;
            return 1;
        }
    }

    if(distanceNumber > -1)
    {
        if((currentStep >= SSet[distanceNumber].distance) && (interrupter < 0))
        {
            Stop();
            finishFlag = true;
            return 1;
        }

        if(currentStep >= SSet[distanceNumber].distance + SSet[distanceNumber].oversteps)
        {
            Stop();
            errorCommand = SetError(OVERSTEP);
            if(Error != nullptr)
                Error(this);
            return 0;
        }
    }

    if(!moveFlag)
    {
        this->direction = direction;
        if(direction)
            digitalWrite(dirPin, true);
        else
            digitalWrite(dirPin, false);
        digitalWrite(enPin, false);
        speed = MSet[setNumber].minSpeed;
        moveFlag = true;
    }

    uint32_t mls = millis();
    if(mls >= previousMillis + MSet[setNumber].millisStep)
    {
        if ((speed > MSet[setNumber].maxSpeed) &&
            (currentStep < SSet[distanceNumber].distance / 2 || distanceNumber < 0) &&
            (brakeFlag == false))
        {
            speed--;
            accelerationSteps = currentStep;
        }
        else if(((speed < MSet[setNumber].minSpeed) && (currentStep > SSet[distanceNumber].distance - accelerationSteps) && (distanceNumber >= 0)) ||
                (brakeFlag == true))
        {
            speed++;
        }
        previousMillis = mls;  
    }

    if(previousMillis > mls)
        previousMillis = mls;  

    return 0;
}

bool MyStepper::MoveToPoint(int8_t pointNumber, int8_t setNumber, int8_t interrupter)
{
    if(manualFlag)
        return 0;

    if(stopFlag)
        return 0;

    if(errorCommand != 0)
        return 0;

    if(finishFlag)
        return 1;

    if(!moveFlag)
    {
        if(PSet[pointNumber].point > currentPoint)
        {
            direction = true;
            digitalWrite(dirPin, true);
        } 
        else
        {
            direction  = false;
            digitalWrite(dirPin, false);
        }  
        distanceToPoint = abs(currentPoint - PSet[pointNumber].point);
        digitalWrite(enPin, false);
        speed = MSet[setNumber].minSpeed;
        moveFlag = true;
    }

    if(interrupter > 0)
    {
        Stop();

        if(direction)
            if(currentPoint <= (PSet[pointNumber].point - PSet[pointNumber].understeps))
                errorCommand = SetError(UNDERSTEP);
        else
            if(currentPoint > (PSet[pointNumber].point + PSet[pointNumber].understeps))
                errorCommand = SetError(UNDERSTEP);

        if(errorCommand != 0)
        {
            if(Error != nullptr)
                Error(this);
            return 0;
        } 

        finishFlag = true;
        return 1;
    }

    if(direction)
    {
        if((currentPoint >= (PSet[pointNumber].point)) && (interrupter < 0))
        {
            Stop();
            finishFlag = true;
            return 1;
        }
        if(currentPoint > (PSet[pointNumber].point + PSet[pointNumber].oversteps))
        {
            Stop();
            errorCommand = SetError(OVERSTEP);
            if(Error != nullptr)
                Error(this);
            return 0;
        }
    }
    else
    {
        if((currentPoint <= (PSet[pointNumber].point)) && (interrupter < 0))
        {
            Stop();
            finishFlag = true;
            return 1;
        }
        if(currentPoint < (PSet[pointNumber].point - PSet[pointNumber].oversteps))
        {
            Stop();
            errorCommand = SetError(OVERSTEP);
            if(Error != nullptr)
                Error(this);
            return 0;
        }
    }

    uint32_t mls = millis();
    if(mls >= previousMillis + MSet[setNumber].millisStep)
    {
        if((speed > MSet[setNumber].maxSpeed) && (currentStep < distanceToPoint / 2))
        {
            speed--;
            accelerationSteps = currentStep;
        }
        else if((speed < MSet[setNumber].minSpeed) && (currentStep > distanceToPoint - accelerationSteps))
            speed++;
        previousMillis = mls;  
    }

    if(previousMillis > mls)
        previousMillis = mls;  

    return 0;
}

void MyStepper::ManualMove(bool direction, uint16_t momentalSpeed)
{
    manualFlag = true;
    moveFlag = true;
    this->direction = direction;
    if(direction)
        digitalWrite(dirPin, true);
    else
        digitalWrite(dirPin, false);
    digitalWrite(enPin, false);
    speed = momentalSpeed;
}

void MyStepper::Stop()
{
    moveFlag = false;
    stopFlag = true;
    if(freeStay)
        digitalWrite(enPin, true);
}

void MyStepper::StopAll()
{
    MyStepper* ptr = currentPtr;
    while(ptr != nullptr)
    {
        ptr->Stop();
        ptr = ptr->ptrOnOther;
    }
}

void MyStepper::Refresh()
{
    Stop();
    finishFlag = false;
    manualFlag = false;
    stopFlag = false;
    currentStep = 0;
}

void MyStepper::RefreshAll()
{
    MyStepper* ptr = currentPtr;
    while(ptr != nullptr)
    {
        ptr->Refresh();
        ptr = ptr->ptrOnOther;
    }
}

void MyStepper::ResetCurrentPoint()
{
    currentPoint = 0;
}

bool MyStepper::GetFinish()
{
    return finishFlag;
}

uint8_t MyStepper::GetError()
{
    return errorCommand;
}

uint32_t MyStepper::GetCurrentStep()
{
    return currentStep;
}

int32_t MyStepper::GetCurrentPoint()
{
    return currentPoint;
}

int32_t MyStepper::GetPoint(uint16_t point)
{
    if(point >= PSetNumber)
        return 0;
    return PSet[point].point;
}

uint8_t MyStepper::GetID()
{
    return ID;
}

void MyStepper::CleanError()
{
    errorCommand = 0;
}

uint8_t MyStepper::SetError(err_t error)
{
    uint8_t result = (uint8_t)error;
    return (result |= ID << 3); 
}