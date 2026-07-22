#include "MyStepper.h"


MyStepper::MyStepper()
{
    if (numSteppers == 7)
    {
        // ERR
    }

    numSteppers++;
    ID = numSteppers;
    rmt_channel = (rmt_channel_t)ID;

    ptrOnOther = currentPtr;
    currentPtr = this;




    if (numSteppers == 0)
    {
        interrupter = timerBegin(0,80,true);    //  80MHz (это APB_frequency, а не CPU_frequency)
        timerAttachInterrupt(interrupter,&MyStepper::Step,true);
        timerAlarmWrite(interrupter,interrupterStep_us,true);
        timerAlarmEnable(interrupter);
    }
}

MyStepper::MyStepper(uint8_t stepPin, uint8_t dirPin, uint8_t enPin, bool powerStay) : MyStepper()
{
    SetEngine(stepPin,dirPin,enPin,powerStay);
}

void MyStepper::SetEngine(uint8_t stepPin, uint8_t dirPin, uint8_t enPin, bool powerStay)
{
    this-> stepPin = stepPin;
    this-> dirPin = dirPin;
    this-> enPin = enPin;
    this-> powerStay = powerStay;

    rmt_config_t config = RMT_DEFAULT_CONFIG_TX((gpio_num_t)stepPin, rmt_channel);
    rmt_config(&config);
    rmt_driver_install(rmt_channel, 0, 0);

    taskMailbox = xQueueCreate(1, sizeof(task_mail_t));


    xTaskCreatePinnedToCore(
        MyStepper::TaskWrapper,     // Функция обертки
        "StepperTask",              // Имя задачи
        4096,                       // Размер стека в байтах
        this,                       // Указатель на себя
        5,                          // Приоритет задачи (выше среднего)
        &taskHandle,                // Хендл
        1                           // Ядро (1, чтобы не мешать WiFi на 0 ядре)
    );

    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enPin, OUTPUT);
    if (powerStay)
        digitalWrite(enPin, false);
    else
        digitalWrite(enPin, true);
}

void MyStepper::SetSteps(st_dist_t* stepStruct, uint32_t steps, uint32_t understeps, uint32_t oversteps)
{
    if (stepStruct == nullptr)
    {
        Error(NO_STRUCT);
        return;
    }
 
    if (understeps == 0)
        understeps = steps;
    stepStruct->steps = steps;
    stepStruct->understeps = understeps;
    stepStruct->oversteps = oversteps;
}

void MyStepper::SetPoint(st_point_t* pntStruct, int32_t point, int32_t understeps, int32_t oversteps)
{
    if (pntStruct == nullptr)
    {
        Error(NO_STRUCT);
        return;
    }

    pntStruct->point = point;
    pntStruct->understeps = understeps;
    pntStruct->oversteps = oversteps;
}

void MyStepper::SetPointInArea(st_point_t* pntStruct, st_point_t minEdgePnt, st_point_t maxEdgePnt, int32_t point, int32_t understeps, int32_t oversteps)
{
    if (minEdgePnt.point > maxEdgePnt.point)
    {
        Error(UNAVAILABLE_POINT,this);
        return;
    }

    point = constrain(point,minEdgePnt.point, maxEdgePnt.point);
    SetPoint(pntStruct,point,understeps,oversteps);
}

void MyStepper::SetMove(st_move_t* mvStruct, uint32_t v_start_hz, uint32_t v_work_hz, uint32_t v_fin_hz, uint32_t t_accel_ms, uint32_t t_decel_ms)
{
    if (mvStruct == nullptr)
    {
        Error(NO_STRUCT);
        return;
    }
 
    mvStruct->v_start_hz = v_start_hz;
    mvStruct->v_work_hz = v_work_hz;
    mvStruct->v_fin_hz = v_fin_hz;

    if (t_accel_ms == 0)
        mvStruct->a_accel = 0.0f;
    else
        mvStruct->a_accel = ((float)v_work_hz - (float)v_start_hz) * 1000.0f / (float)t_accel_ms;

    if (t_decel_ms == 0)
        mvStruct->a_decel = 0.0f;
    else
        mvStruct->a_decel = ((float)v_fin_hz - (float)v_work_hz) * 1000.0f / (float)t_decel_ms;
}

void MyStepper::SetCommonErrorHandler(void (*ExError)(void *))
{
    CommonExError = ExError;
}

void MyStepper::SetSerial(HardwareSerial *Serial, uint32_t baudRate)
{
    MySerial = Serial;
    MySerial->begin(baudRate);
}

void MyStepper::SetErrorHandler(void (*ExError)(void*), bool ignoreCommonExError)
{
    this->ExError = ExError;
    this->ignoreCommonExError = ignoreCommonExError;
}

bool MyStepper::Move(st_dir_t dir, st_move_t mv, st_step_t dist, int8_t* interrupter)
{






    
    if (manualFlag)
        return 0;

    if (stopFlag)
        return 0;

    if ((errorCommand != 0) || (staticErrorCommand != 0))
        return 0;

    if (finishFlag)
        return 1;

    if (interrupter > 0)
    {
        if ((dist != NO_DISTANCE) && 
           (currentStep < (dist->steps - dist->understeps)))
        {
            Stop();
            Error(UNDERSTEP,this);
            return 0;
        }
        else
        {
            SoftStop();
            finishFlag = true;
            return 1;
        }
    }

    if (dist != NO_DISTANCE)
    {
        if ((currentStep >= dist->steps) && (interrupter < 0))
        {
            SoftStop();
            finishFlag = true;
            return 1;
        }
        if (currentStep >= (dist->steps + dist->oversteps))
        {
            Stop();
            Error(OVERSTEP,this);
            return 0;
        }
    }

    if (phase != START)
    {
        if ((dir != prevDirection) || 
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
    if (manualFlag)
        return 0;

    if (stopFlag)
        return 0;

    if ((errorCommand != 0) || (staticErrorCommand != 0))
        return 0;

    if ((pnt != prevPoint) || (mv != prevMove))
        InternalRefresh();
    prevPoint = pnt;
    prevMove = mv;

    if (finishFlag)
        return 1;

    InternalMove(mv,pnt,nullptr);

    if (interrupter > 0)
    {
        if ((!pnt->noUndersteps) && (internalDistance > pnt->understeps))
        {
            if (currentStep < (internalDistance - pnt->understeps))
            {
                Stop();
                Error(UNDERSTEP,this);
                return 0;
            }
        }
        SoftStop();
        finishFlag = true;
        return 1;
    }

    if ((currentStep >= internalDistance) && (interrupter < 0))
    {
        SoftStop();
        finishFlag = true;
        return 1;
    }
    if (currentStep >= (internalDistance + pnt->oversteps))
    {
        Stop();
        Error(OVERSTEP,this);
        return 0;
    }

    return 0;
}

void MyStepper::SetCurrentSpeed(st_dir_t dir, uint16_t currentSpeed)
{
    manualFlag = true;
    InternalSetCurrentSpeed(dir,currentSpeed);
}

bool MyStepper::ChangeSpeed(uint16_t dstSpeed, uint32_t time_ms)
{
    bool refresh = false;

    manualFlag = true;

    if ((dstSpeed != prevDstSpeed) || (time_ms != prevTime_ms))
    {
        CountAccel(&ptrTmpAccel,speed,dstSpeed,time_ms);
        refresh = true;
    }

    return InternalChangeSpeed(ptrTmpAccel,refresh);
}

void MyStepper::Stop()
{
    stopFlag = true;
    SoftStop();
}

void MyStepper::StopAll()
{
    MyStepper* ptr = currentPtr;
    while (ptr != nullptr)
    {
        ptr->Stop();
        ptr = ptr->ptrOnOther;
    }
}

void MyStepper::Refresh()
{
    SoftStop();
    finishFlag = false;
    manualFlag = false;
    stopFlag = false;
    currentStep = 0;
    phase = START;
}

void MyStepper::RefreshAll()
{
    MyStepper* ptr = currentPtr;
    while (ptr != nullptr)
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

bool MyStepper::GetFinishAll()
{
    MyStepper* ptr = currentPtr;

    while (ptr != nullptr)
    {
        if (!(ptr->GetFinish()))
            return false;
        ptr = ptr->ptrOnOther;
    }
    return true;
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

uint16_t MyStepper::GetCurrentSpeed()
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
    while (ptr != nullptr)
    {
        ptr->CleanError();
        ptr = ptr->ptrOnOther;
    }
    CleanStaticError();
}

void MyStepper::SetExtDirCallback(ExtPinCallback cb)
{
}

void MyStepper::SetExtEnCallback(ExtPinCallback cb)
{
}
