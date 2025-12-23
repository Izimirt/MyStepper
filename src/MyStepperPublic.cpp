#include "MyStepper.h"

#if defined(ESP32)

    MyStepper::MyStepper()
    {
        ptrOnOther = currentPtr;
        currentPtr = this;
        if(numSteppers == 0)
        {
            interrupter = timerBegin(0,80,true);    //  80MHz (это APB_frequency, а не CPU_frequency)
            timerAttachInterrupt(interrupter,&MyStepper::Step,true);
            timerAlarmWrite(interrupter,interrupterStep_us,true);
            timerAlarmEnable(interrupter);
        }
        numSteppers++;
        ID = numSteppers;
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

    void MyStepper::StartInterrupter()
    {
        interrupterStep_us *= 2;
        Timer1.setPeriod(interrupterStep_us);
        Timer1.enableISR(CHANNEL_B);
    }

#endif

MyStepper::MyStepper(uint8_t stepPin, uint8_t dirPin, uint8_t enPin, bool freeStay) : MyStepper()
{
    SetEngine(stepPin,dirPin,enPin,freeStay);
}

void MyStepper::SetEngine(uint8_t stepPin, uint8_t dirPin, uint8_t enPin, bool freeStay)
{
    this-> stepPin = stepPin;
    this-> dirPin = dirPin;
    this-> enPin = enPin;
    this-> freeStay = freeStay;
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enPin, OUTPUT);
    if(freeStay)
        digitalWrite(enPin, true);
    else
        digitalWrite(enPin, false);
}

void MyStepper::SetSteps(st_step_t** setPtr, uint32_t steps, uint32_t understeps, uint32_t oversteps)
{
    if(*setPtr == nullptr)
        *setPtr = new st_step_t;
    if(understeps == 0)
        understeps = steps;
    (*setPtr)->steps = steps;
    (*setPtr)->understeps = understeps;
    (*setPtr)->oversteps = oversteps;
}

void MyStepper::SetPoint(st_point_t** setPtr, int32_t point, int32_t understeps, int32_t oversteps, bool noUndersteps, uint16_t pointNumber)
{
    if(*setPtr == nullptr)
        *setPtr = new st_point_t;
    (*setPtr)->ptrOnPrev = pPtrOnHead;
    pPtrOnHead =  *setPtr;
    (*setPtr)->pointNumber = pointNumber;

    if(noUndersteps)
        (*setPtr)->noUndersteps = true;
    (*setPtr)->point = point;
    (*setPtr)->understeps = understeps;
    (*setPtr)->oversteps = oversteps;
}

void MyStepper::SetPointInArea(st_point_t** setPtr, st_point_t* minEdge, st_point_t* maxEdge, int32_t point, int32_t understeps, int32_t oversteps, bool noUndersteps, uint16_t pointNumber)
{
    if((minEdge == NO_POINT) && (maxEdge == NO_POINT))
    {
        Error(UNAVAILABLE_POINT,this);
        return;
    }
    if((minEdge != NO_POINT) && (maxEdge != NO_POINT))
    {
        if(minEdge->point > maxEdge->point)
        {
            Error(UNAVAILABLE_POINT,this);
            return;
        }
    }

    point = constrain(point,minEdge->point,maxEdge->point);

    SetPoint(setPtr,point,understeps,oversteps,noUndersteps,pointNumber);
}

void MyStepper::SetMove(st_move_t** setPtr, uint8_t startSpeed, uint8_t workSpeed, uint8_t finishSpeed, uint16_t accelerationTime_ms, uint16_t decelerationTime_ms)
{
    if((startSpeed < workSpeed) && (startSpeed != 0))              // In this case engine won't run. If you erase the error, 
    {                                                              // engine will work, but without breaked acceleration/deceleration,
        Error(INCORRECT_MOVE_PARAMETERS);                          // because flag noAccel will be turned ON.
        startSpeed = workSpeed;
    }
    if(finishSpeed < workSpeed)
    {
        Error(INCORRECT_MOVE_PARAMETERS);
        finishSpeed = workSpeed;
    }
    if(*setPtr == nullptr)
    {
        *setPtr = new st_move_t;
        (*setPtr)->startAccel = new st_accel_t{};
        (*setPtr)->finishAccel = new st_accel_t{};  
    }
    CountAccel((*setPtr)->startAccel,startSpeed,workSpeed,accelerationTime_ms);
    CountAccel((*setPtr)->finishAccel,workSpeed,finishSpeed,decelerationTime_ms);
}
void MyStepper::SetCommonErrorHandler(void (*ExError)(void *))
{
    CommonExError = ExError;
}

void MyStepper::SetSerial(HardwareSerial* Serial, uint32_t baudRate)
{
    MySerial = Serial;
    MySerial->begin(baudRate);
}

void MyStepper::SetErrorHandler(void (*ExError)(void*), bool ignoreCommonExError)
{
    this->ExError = ExError;
    this->ignoreCommonExError = ignoreCommonExError;
}

bool MyStepper::Move(st_dir_t dir, st_move_t* mv, st_step_t* dist, int8_t interrupter)
{
    if(manualFlag)
        return 0;

    if(stopFlag)
        return 0;    

    if((errorCommand != 0) || (staticErrorCommand != 0))
        return 0;

    if(finishFlag)
        return 1;

    if(interrupter > 0)
    {
        Stop();
        if((dist != NO_DISTANCE) && 
           (currentStep < (dist->steps - dist->understeps)))
        {
            Error(UNDERSTEP,this);
            return 0;
        }
        else
        {
            finishFlag = true;
            return 1;
        }
    }

    if(dist != NO_DISTANCE)
    {
        if((currentStep >= dist->steps) && (interrupter < 0))
        {
            Stop();
            finishFlag = true;
            return 1;
        }
        if(currentStep >= (dist->steps + dist->oversteps))
        {
            Stop();
            Error(OVERSTEP,this);
            return 0;
        }
    }

    if(phase != START)
    {
        if((dir != prevDirection) || 
           (mv != prevMove) || 
           (dist != prevDistance))
        {
            Error(UNEXPECTED_ENTER_IN_FUNCTION,this);
            return 0;
        }
    }
    prevDirection = dir;
    prevMove = mv;
    prevDistance = dist;

    InternalMove(mv,nullptr,dist,dir);
    
    return 0;
}

bool MyStepper::MoveToPoint(st_point_t* pnt, st_move_t* mv, int8_t interrupter)
{
    if(manualFlag)
        return 0;

    if(stopFlag)
        return 0;

    if((errorCommand != 0) || (staticErrorCommand != 0))
        return 0;

    if(finishFlag)
        return 1;

    if((pnt != prevPoint) || (mv != prevMove))
        InternalRefresh();
    prevPoint = pnt;
    prevMove = mv;

    InternalMove(mv,pnt,nullptr);

    if(interrupter > 0)
    {
        Stop();
        if((!pnt->noUndersteps) && (internalDistance > pnt->understeps))
        {
            if(currentStep < (internalDistance - pnt->understeps))
            {
                Error(UNDERSTEP,this);
                return 0;
            }
        }
        finishFlag = true;
        return 1;
    }

    if((currentStep >= internalDistance) && (interrupter < 0))
    {
        Stop();
        finishFlag = true;
        return 1;
    }
    if(currentStep >= (internalDistance + pnt->oversteps))
    {
        Stop();
        Error(OVERSTEP,this);
        return 0;
    }

    return 0;
}

void MyStepper::SetCurrentSpeed(st_dir_t dir, uint8_t currentSpeed)
{
    manualFlag = true;
    InternalSetCurrentSpeed(dir,currentSpeed);
}

bool MyStepper::ChangeSpeed(uint8_t dstSpeed, uint32_t time_ms)
{
    bool refresh = false;

    manualFlag == true;

    if((dstSpeed != prevDstSpeed) || (time_ms != prevTime_ms))
    {
        CountAccel(&tmpAccel,speed,dstSpeed,time_ms);
        refresh = true;
    }

    return InternalChangeSpeed(&tmpAccel,refresh);
}

void MyStepper::Stop()
{
    moveFlag = false;
    stopFlag = true;
    speed = 0;
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
    phase = START;
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

uint8_t MyStepper::GetStaticError()
{
    return staticErrorCommand;
}

uint32_t MyStepper::GetCurrentStep()
{
    return currentStep;
}

int32_t MyStepper::GetCurrentPoint()
{
    return currentPoint;
}

uint8_t MyStepper::GetCurrentSpeed()
{
    return speed;
}

uint8_t MyStepper::GetID()
{
    return ID;
}

void MyStepper::CleanError()
{
    errorCommand = 0;
}

void MyStepper::CleanStaticError()
{
    staticErrorCommand = 0;
}

void MyStepper::CleanAllErrors()
{
    MyStepper* ptr = currentPtr;
    while(ptr != nullptr)
    {
        ptr->CleanError();
        ptr = ptr->ptrOnOther;
    }
    CleanStaticError();
}