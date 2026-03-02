#include "MyStepper.h"

#if defined(ESP32)

    hw_timer_t* MyStepper::interrupter = nullptr;

    void IRAM_ATTR MyStepper::Step()
    {
        unitPtr = currentPtr;

        while (unitPtr != nullptr)
        {
            if (unitPtr->moveFlag)
            {
                if (unitPtr->speed != 0)
                {
                    if (unitPtr->timer >= unitPtr->speed)
                    {
                        unitPtr->stepState = !unitPtr->stepState;
                        digitalWrite(unitPtr->stepPin,unitPtr->stepState);
                        unitPtr->currentStep++;
                        (unitPtr->direction == st_dir::FWD) ? unitPtr->currentPoint++ : unitPtr->currentPoint--;
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

#else

    void MyStepper::Step() //=============================================== IRAM_ATTR
    {
        unitPtr = currentPtr;

        while (unitPtr != nullptr)
        {
            if (unitPtr->moveFlag)
            {
                if (unitPtr->speed != 0)
                {
                    if (unitPtr->timer >= unitPtr->speed)
                    {
                        unitPtr->stepState = !unitPtr->stepState;
                        digitalWrite(unitPtr->stepPin,unitPtr->stepState);
                        unitPtr->currentStep++;
                        (unitPtr->direction == st_dir::FWD) ? unitPtr->currentPoint++ : unitPtr->currentPoint--;
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

    ISR(TIMER1_B)
    {
        MyStepper::Step();
    }

#endif

uint8_t MyStepper::interrupterStep_us = 10;

uint8_t MyStepper::numSteppers = 0;

uint8_t MyStepper::staticErrorCommand = 0;

void (*MyStepper::CommonExError)(void*) = nullptr;

MyStepper* MyStepper::currentPtr = nullptr;

MyStepper* MyStepper::unitPtr = nullptr;

HardwareSerial* MyStepper::MySerial = nullptr;

void MyStepper::InternalSetCurrentSpeed(st_dir_t dir, uint16_t currentSpeed)
{
    moveFlag = true;
    this->direction = dir;
    if (dir == st_dir::FWD)
        digitalWrite(dirPin, true);
    else
        digitalWrite(dirPin, false);
    digitalWrite(enPin, false);
    speed = currentSpeed;
}

bool MyStepper::InternalChangeSpeed(st_accel_t* accel, bool refresh)
{
    if (refresh)
    {
        moveFlag = true;

        if (accel->period_us == 0)   // no acceleration
        {
            speed = accel->dstSpeed;
            accelSuccess = true;
            return 1;
        }

        accelSuccess = false;
        speedCounter = accel->numPeriods + 1; // +1 нужен тк в первой итерации выставляется начальная скорость
        currentUnrealNumStepsPerPeriod = accel->bgnNumStepsPerPeriod;
    }

    if (accelSuccess)
        return 1;

    uint32_t us = micros();
    if (previousUs > us)
        previousUs = us;
    if (us >= previousUs + accel->period_us)
    {
        if (speedCounter > 0)
        {
            speed = constrain(round(accel->period_us / (currentUnrealNumStepsPerPeriod * interrupterStep_us)), 1, 0xFFFF);
            currentUnrealNumStepsPerPeriod += accel->stepNumSteps;
            speedCounter--;
        }
        else
            accelSuccess = true;

            
        previousUs = us;  
    }
    return accelSuccess;
}

bool MyStepper::InternalMove(st_move_t* mv, st_point_t* pnt, st_step_t* dist, st_dir_t dir)
{
    bool done = false;
    bool refresh = false;
    if (phase == START)
    {
        if (CheckNeedCountSteps(mv->finishAccel))
            if (!CountSteps(mv->finishAccel))
                return 0;

        if (dist != nullptr)
        {
            this->direction = dir;
            if (dist != NO_DISTANCE)
                internalDistance = dist->steps;
        }
        else if (pnt != nullptr)
        {
            internalDistance = abs(currentPoint - pnt->point);
            if (pnt->point > currentPoint)
                this->direction = st_dir::FWD;
            else
                this->direction = st_dir::BWD;
        }
        else
        {
            Error(UNKNOWN_DISTANCE,this);
            return 0;
        }

        currentLvl = bPtrOnHead;

        if (currentLvl->steps != 0)  //  in case "no deceleration"
        {
            while (currentLvl->speed > mv->startAccel->bgnSpeed)
            {
                if (currentLvl->ptrOnPrev == nullptr)
                    currentLvl = bPtrOnHead; // in case when finish speed less then start speed

                currentLvl = currentLvl->ptrOnPrev;
            }
        }
                    
        InternalSetCurrentSpeed(this->direction,mv->startAccel->bgnSpeed);
        refresh = true;
        phase = ACCELERATION_AND_GO;
    }
    if (phase == ACCELERATION_AND_GO)
    {
        bool normalBrake = InternalChangeSpeed(mv->startAccel,refresh);

        if (currentLvl->steps != 0)  //  in case "no deceleration"
            if (speed < currentLvl->speed)
                if (currentLvl->ptrOnPrev != nullptr)
                    currentLvl = currentLvl->ptrOnPrev;


        if (dist != NO_DISTANCE)
        {
            if (currentStep + (uint32_t)(currentLvl->steps) >= internalDistance)
            {
                refresh = true;

                if (normalBrake)
                    ptrTmpAccel = mv->finishAccel;
                else
                    CountAccel(&ptrTmpAccel,speed,mv->finishAccel->dstSpeed,(currentLvl->time_us / 1000));
                    
                phase = DECELERATION;
            }
        }
    }
    if (phase == DECELERATION)
    {
        if (InternalChangeSpeed(ptrTmpAccel,refresh))
            done = true; 
    }

    return done;
}

void MyStepper::InternalRefresh()
{
    currentStep = 0;
    phase = START;
}

void MyStepper::CountAccel(st_accel_t** accelPtr, uint16_t bgnSpeed, uint16_t dstSpeed, uint32_t time_ms)
{
    if (*accelPtr == nullptr)
        *accelPtr = new st_accel_t{};

    if (dstSpeed == 0)
        dstSpeed = 0xFFFF;
    if (bgnSpeed == 0)           
        bgnSpeed = 0xFFFF;   

    (*accelPtr)->bgnSpeed = bgnSpeed;
    (*accelPtr)->dstSpeed = dstSpeed;
    (*accelPtr)->time_ms = time_ms;
    (*accelPtr)->numPeriods = abs(bgnSpeed - dstSpeed);

    if ((bgnSpeed == dstSpeed) || (time_ms == 0))    // no acceleration
    {
        (*accelPtr)->period_us = 0;
        (*accelPtr)->bgnNumStepsPerPeriod = 0;
        (*accelPtr)->dstNumStepsPerPeriod = 0;
        (*accelPtr)->stepNumSteps = 0;
    }
    else
    {
        (*accelPtr)->period_us = (time_ms * 1000UL) / (*accelPtr)->numPeriods;
        if ((*accelPtr)->period_us < 1)
            (*accelPtr)->period_us = 1;
        (*accelPtr)->bgnNumStepsPerPeriod = (float)((*accelPtr)->period_us) / (uint16_t)(interrupterStep_us * bgnSpeed);
        (*accelPtr)->dstNumStepsPerPeriod = (float)((*accelPtr)->period_us) / (uint16_t)(interrupterStep_us * dstSpeed);
        (*accelPtr)->stepNumSteps = (float)(((*accelPtr)->dstNumStepsPerPeriod - (*accelPtr)->bgnNumStepsPerPeriod) / (float)((*accelPtr)->numPeriods));
    }
}

bool MyStepper::CountSteps(st_accel_t* accel)
{
    uint16_t curSpeed = accel->bgnSpeed;                                         
    float currentNumStepsPerPeriod = accel->bgnNumStepsPerPeriod;            
           
    if (currentNumStepsPerPeriod == 0)   //  in case "no acceleration" currentNumStepsPerPeriod = 0
    {
        bPtrOnHead = bPtrOnTail = new st_brake_t {};
        bPtrOnHead->steps = 0;
        return 1;
    }

    do
    {                                                                                                                       
        if ((bPtrOnHead == nullptr) || (bPtrOnHead->speed != curSpeed))                                                  
        {
            st_brake_t* node = new st_brake_t {};
            if (node == nullptr)
            {
                Error(CANT_COUNT_BRAKE,this);
                return 0;
            }
            node->ptrOnPrev = bPtrOnHead;
            bPtrOnHead = node;
            bPtrOnHead->ptrOnNext = nullptr;
            if (bPtrOnTail == nullptr)
                bPtrOnTail = bPtrOnHead; 
            else
                bPtrOnHead->ptrOnPrev->ptrOnNext = bPtrOnHead;

            bPtrOnHead->speed = curSpeed;
        }    

        bPtrOnHead->steps += (float)(accel->period_us) / (curSpeed * interrupterStep_us);
        bPtrOnHead->time_us += accel->period_us;

        currentNumStepsPerPeriod += accel->stepNumSteps;
        curSpeed = round(accel->period_us / (currentNumStepsPerPeriod * interrupterStep_us));
    }
    while (curSpeed < accel->dstSpeed);
            
    if (bPtrOnHead->ptrOnPrev != nullptr)    //  Here we're summing steps, that engine do, while braking in case when we have several brake levels (not one)
    {
        st_brake_t* node = bPtrOnHead;
        do
        {
            node = node->ptrOnPrev;
            node->steps += node->ptrOnNext->steps;
            node->time_us += node->ptrOnNext->time_us;
        } while (node->ptrOnPrev != nullptr);
    }
    return 1;
}

bool MyStepper::CheckNeedCountSteps(st_accel_t* accel)
{
    if (lastFinishAccel != nullptr)
    {
        if ((accel->bgnSpeed == lastFinishAccel->bgnSpeed) &&    //  If the current braking is the same like the previous one,
           (accel->dstSpeed == lastFinishAccel->dstSpeed) &&    //  we shouldnt recount it
           (accel->time_ms == lastFinishAccel->time_ms))
        {
            return 0;
        }
    }
    lastFinishAccel = accel;

    st_brake_t* node;
    while (bPtrOnHead != nullptr)                               // This is the deleting of a previous braking, in case we need to recount it
    {
        node = bPtrOnHead;
        bPtrOnHead = bPtrOnHead->ptrOnPrev;
        delete node;
    }
    bPtrOnTail = nullptr;

    return 1;
}

void MyStepper::Error(st_err_t error, MyStepper* unit)
{
    uint8_t err = (uint8_t)error;
    if (unit != nullptr)
    {
        err |= unit->ID << 3;
        unit->errorCommand = err;
    }
    else
        staticErrorCommand = err;

    if (MySerial != nullptr)
    {
        MySerial->print("STEPPER_ERROR_");
        MySerial->println(err,BIN);
    }

    if (unit == nullptr)
    {
        if (CommonExError != nullptr)
            CommonExError(nullptr);
    }
    else
    {
        if (unit->ExError != nullptr)
            unit->ExError(unit);
        if (!(unit->ignoreCommonExError))
            if (CommonExError != nullptr)
                CommonExError(unit);
    }
}