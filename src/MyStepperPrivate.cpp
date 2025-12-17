#include "MyStepper.h"

#if defined(ESP32)

    hw_timer_t* MyStepper::interrupter = nullptr;

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
                        (unitPtr->direction == FWD) ? unitPtr->currentPoint++ : unitPtr->currentPoint--;
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
                        (unitPtr->direction == FWD) ? unitPtr->currentPoint++ : unitPtr->currentPoint--;
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

uint8_t MyStepper::numSteppers = 0;

uint8_t MyStepper::staticErrorCommand = 0;

void (*MyStepper::CommonExError)(void*) = nullptr;

MyStepper* MyStepper::currentPtr = nullptr;

MyStepper* MyStepper::unitPtr = nullptr;

HardwareSerial* MyStepper::MySerial = nullptr;

uint8_t MyStepper::interrupterStep_us = 10;

void MyStepper::InternalSetCurrentSpeed(dir_t dir, uint8_t currentSpeed)
{
    moveFlag = true;
    this->direction = dir;
    if(dir == FWD)
        digitalWrite(dirPin, true);
    else
        digitalWrite(dirPin, false);
    digitalWrite(enPin, false);
    speed = currentSpeed;
}

bool MyStepper::InternalChangeSpeed(accel_t* accel, bool refresh)
{
    if(refresh)
    {
        if(accel->period_ms == 0)   // no acceleration
        {
            accelSuccess = true;
            return 1;
        }

        accelSuccess = false;
        moveFlag = true;
        currentUnrealNumStepsPerPeriod = accel->bgnNumStepsPerPeriod;
    }

    if(accelSuccess)
        return 1;

    uint32_t ms = millis();
    if(previousMillis > ms)
        previousMillis = ms;
    if(ms >= previousMillis + accel->period_ms)
    {
        if(speed != accel->dstSpeed)
        {
            speed = round(accel->period_us / (currentUnrealNumStepsPerPeriod * interrupterStep_us));
            currentUnrealNumStepsPerPeriod += accel->stepNumSteps;
        }
        else
            accelSuccess = true;

        //MySerial->println(speed);
        previousMillis = ms;  
    }
    return accelSuccess;
}

bool MyStepper::InternalMove(move_t* mv, point_t* pnt, step_t* dist, dir_t dir)
{
    bool done = false;
    bool refresh = false;
    if(phase == START)
    {
        MySerial->print("bPtrOnTail_1_");
        MySerial->println((int)bPtrOnTail);

        if(CheckNeedCountSteps(mv->finishAccel))
        {
            MySerial->print("bPtrOnTail_2_");
            MySerial->println((int)bPtrOnTail);

            if(!CountSteps(mv->finishAccel))
                return 0;
        }

        MySerial->print("bPtrOnTail_3_");
        MySerial->println((int)bPtrOnTail);

        if(dist != nullptr)
        {
            this->direction = dir;
            if(dist != NO_DISTANCE)
                internalDistance = dist->steps;
        }
        else if(pnt != nullptr)
        {
            internalDistance = abs(currentPoint - pnt->point);
            if(pnt->point > currentPoint)
                this->direction = FWD;
            else
                this->direction = BWD;
        }
        else
        {
            Error(UNKNOWN_DISTANCE,this);
            return 0;
        }

        currentLvl = bPtrOnTail;

        // MySerial->print("currentLvl->speed_");
        // MySerial->println(currentLvl->speed);
        // MySerial->print("mv->startAccel->bgnSpeed_");
        // MySerial->println(mv->startAccel->bgnSpeed);
        // MySerial->println();

        while(currentLvl->speed > mv->startAccel->bgnSpeed)
        {
            if(currentLvl->ptrOnNext == nullptr)
                currentLvl = bPtrOnTail;

            currentLvl = currentLvl->ptrOnNext;
        }

        InternalSetCurrentSpeed(this->direction,mv->startAccel->bgnSpeed);
        refresh = true;
        MySerial->println("TO_PH_\"ACCELERATION_AND_GO\"");
        phase = ACCELERATION_AND_GO;
    }
    if(phase == ACCELERATION_AND_GO)
    {
        InternalChangeSpeed(mv->startAccel,refresh);

        if(speed > currentLvl->speed)
            if(currentLvl->ptrOnNext != nullptr)
                currentLvl = currentLvl->ptrOnNext;

        if(dist != NO_DISTANCE)
            if(currentStep + currentLvl->steps >= internalDistance)
            {
                refresh = true;
                MySerial->print("currentStep_");
                MySerial->println(currentStep);
                MySerial->print(currentLvl->speed);
                MySerial->println(currentLvl->speed);
                MySerial->print("currentLvl->steps_");
                MySerial->println(currentLvl->steps);
                MySerial->print("internalDistance_");
                MySerial->println(internalDistance);
                MySerial->println("TO_PH_\"DECELERATION\"");
                phase = DECELERATION;
            }
    }
    if(phase == DECELERATION)
    {
        if(InternalChangeSpeed(mv->finishAccel,refresh))
            done = true;
    }

    return done;
}

void MyStepper::InternalRefresh()
{
    currentStep = 0;
    phase = START;
}

void MyStepper::CountAccel(accel_t* accel, uint8_t bgnSpeed, uint8_t dstSpeed, uint32_t time_ms)
{
    if(dstSpeed == 0)
        dstSpeed = 255;
    if(bgnSpeed == 0)           
        bgnSpeed = 255;   

    accel->bgnSpeed = bgnSpeed;
    accel->dstSpeed = dstSpeed;
    accel->time_ms = time_ms;
    accel->numPeriods = abs(bgnSpeed - dstSpeed);

    if(bgnSpeed == dstSpeed)    // no acceleration
    {
        accel->period_ms = 0;
        accel->period_us = 0;
        accel->bgnNumStepsPerPeriod = 0;
        accel->dstNumStepsPerPeriod = 0;
        accel->stepNumSteps = 0;
    }
    else
    {
        accel->period_ms = (time_ms / accel->numPeriods);
        if(accel->period_ms < 1)
            accel->period_ms = 1;
        accel->period_us = accel->period_ms * 1000UL;
        accel->bgnNumStepsPerPeriod = accel->period_us / (uint16_t)(interrupterStep_us * bgnSpeed);
        accel->dstNumStepsPerPeriod = accel->period_us / (uint16_t)(interrupterStep_us * dstSpeed);
        accel->stepNumSteps = (accel->dstNumStepsPerPeriod - accel->bgnNumStepsPerPeriod) / accel->numPeriods;
    }
}

bool MyStepper::CountSteps(accel_t* accel)
{
    uint8_t curSpeed = accel->dstSpeed;                                        
    uint16_t currentNumStepsPerPeriod = accel->dstNumStepsPerPeriod;
    do                                                                      // Перебор происходит в сторону, обратную течению времени. От конца движения
    {                                                                       // При торможении, значение accel->stepNumSteps будет отрицательным. Так как отсчёт идёт с конца, то есть с периода с меньшим числом шагов,
        if((bPtrOnHead == nullptr) || (bPtrOnHead->speed != curSpeed))      // accel->stepNumSteps необходимо отнимать, чтобы двойным отрицанием прибавлять это значение
        {
            brake_t* node = new brake_t {};
            if(node == nullptr)
            {
                Error(CANT_COUNT_BRAKE,this);
                return 0;
            }

            node->ptrOnPrev = bPtrOnHead;   // создаётся двусвязный список тк при записи (здесь) его необходимо проходить из конца в начало, а при работе наоборот
            bPtrOnHead = node;
            bPtrOnHead->ptrOnNext = nullptr;
            if(bPtrOnTail == nullptr)
                bPtrOnTail = bPtrOnHead; 
            else
                bPtrOnHead->ptrOnPrev->ptrOnNext = bPtrOnHead;

            bPtrOnHead->speed = curSpeed;
            if(bPtrOnHead->ptrOnPrev != nullptr)
                bPtrOnHead->steps += bPtrOnHead->ptrOnPrev->steps;

            // MySerial->println();
            // MySerial->print("curSpeed_");
            // MySerial->println(curSpeed);
            // MySerial->println();
        }    

        bPtrOnHead->steps += accel->period_us / (curSpeed * interrupterStep_us);
        
        if(currentNumStepsPerPeriod != 0)   // in case "no acceleration" currentNumStepsPerPeriod = 0
        {
            curSpeed = round(accel->period_us / (currentNumStepsPerPeriod * interrupterStep_us));
            currentNumStepsPerPeriod -= accel->stepNumSteps;
        } 
    }
    while(curSpeed != accel->bgnSpeed);
    return 1;
}

bool MyStepper::CheckNeedCountSteps(accel_t* accel)
{
    if(lastFinishAccel != nullptr)
    {
        if((accel->bgnSpeed == lastFinishAccel->bgnSpeed) && // если торможение при предыдущем движении было таким же, как и 
           (accel->dstSpeed == lastFinishAccel->dstSpeed) && // при текущем движении, то перещитывать его не нужно
           (accel->time_ms == lastFinishAccel->time_ms))
        {
            return 0;
        }
    }
    lastFinishAccel = accel;

    brake_t* node;
    while(bPtrOnHead != nullptr)                               // удаление предыдущего торможения если торможение нужно пересчитать
    {
        node = bPtrOnHead;
        bPtrOnHead = bPtrOnHead->ptrOnPrev;
        delete node;
    }
    bPtrOnTail = nullptr;

    return 1;
}

void MyStepper::Error(err_t error, MyStepper* unit)
{
    uint8_t err = (uint8_t)error;
    if(unit != nullptr)
    {
        err |= unit->ID << 3;
        unit->errorCommand = err;
    }
    else
        staticErrorCommand = err;

    if(MySerial != nullptr)
    {
        MySerial->print("STEPPER_ERROR_");
        MySerial->println(err,BIN);
    }

    if(unit == nullptr)
    {
        if(CommonExError != nullptr)
            CommonExError(nullptr);
    }
    else
    {
        if(unit->ExError != nullptr)
            unit->ExError(unit);
        if(!(unit->ignoreCommonExError))
            if(CommonExError != nullptr)
                CommonExError(unit);
    }
}