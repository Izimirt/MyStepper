#pragma once

#include <Arduino.h>
#include <math.h>

#include "Const.h"
#include "Structs.h"

class MyStepper 
{
    private:
        
        uint16_t stepPin;
        uint16_t dirPin;
        uint16_t enPin;
        bool freeStay;

        uint32_t currentStep = 0;
        int32_t currentPoint = 0;
        dir_t direction;

        bool moveFlag = false;
        bool finishFlag = false;
        bool brakeFlag = false;
        bool manualFlag = false;
        bool stopFlag = false;

        bool stepState = false;
        uint8_t speed;
        uint32_t previousMillis;
        uint32_t accelerationSteps = 0;
        uint16_t timer = 0;

        // for ChangeSpeed()
        uint16_t currentUnrealNumStepsPerPeriod;             

        // for Move()
        phase_t phase = START;
        accel_t tmpAccel;
        accel_t* lastFinishAccel = nullptr;
        brake_t* currentLvl;
        bool accelSuccess = false;
        uint32_t internalDistance;

        uint8_t prevDstSpeed = 0;
        uint32_t prevTime_ms = 0;
        dir_t prevDirection;
        move_t* prevMove = nullptr;
        step_t* prevDistance = nullptr;
        point_t* prevPoint = nullptr;

        point_t* pPtrOnHead = nullptr;
        brake_t* bPtrOnHead = nullptr;
        brake_t* bPtrOnTail = nullptr;

        MyStepper* ptrOnOther;
        uint8_t ID;

        uint8_t errorCommand = 0;
        static uint8_t staticErrorCommand;
        void (*ExError)(void*) = nullptr;
        static void (*CommonExError)(void*);
        bool ignoreCommonExError = false;

        static uint8_t numSteppers;
        static uint8_t interrupterStep_us;
        static MyStepper* currentPtr;
        static MyStepper* unitPtr;

        static HardwareSerial* MySerial;

        #ifdef ESP32
            static hw_timer_t* interrupter;
            static void Step();
        #endif

        static void Error(err_t error, MyStepper* unit = nullptr);

        void InternalSetCurrentSpeed(dir_t dir, uint8_t currentSpeed);

        bool InternalChangeSpeed(accel_t* accel, bool refresh);

        bool InternalMove(move_t* mv,
                          point_t* pnt = nullptr,     // for MoveToPiont()
                          step_t* dist = nullptr,   // for Move()
                          dir_t dir = FWD);       // for Move()

        void InternalRefresh();

        /// @brief This function is counting acceleration parameters for laiter counts.
        static void CountAccel(accel_t* accel, uint8_t bgnSpeed,uint8_t dstSpeed, uint32_t time_ms);

        bool CountSteps(accel_t* accel);

        bool CheckNeedCountSteps(accel_t* accel);

    public:

        MyStepper();

        MyStepper(uint16_t stepPin,
                  uint16_t dirPin,
                  uint16_t enPin,
                  bool freeStay);

        void SetEngine(uint16_t stepPin,
                       uint16_t dirPin,
                       uint16_t enPin,
                       bool freeStay);

        static void SetSteps(step_t* set,
                             uint32_t steps,
                             uint32_t understeps = 0,
                             uint32_t oversteps = 0);

        void SetPoint(point_t** setPtr,
                      int32_t point,
                      int32_t understeps = 0,
                      int32_t oversteps = 0,
                      bool noUndersteps = false,
                      uint16_t pointNumber = 0);

        void SetPointInArea(point_t** setPtr,
                            point_t* minEdge,
                            point_t* maxEdge,
                            int32_t point,
                            int32_t understeps = 0,
                            int32_t oversteps = 0,
                            bool noUndersteps = false,
                            uint16_t pointNumber = 0);

        /// @param maxSpeed (minSpeed) Количество циклов engine_step_micros = ширина шага(чем длиннее шаг, тем медленнее вращается мотор) (engine_step_micros = 10 microseconds)
        static void SetMove(move_t** setPtr,
                            uint8_t startSpeed,
                            uint8_t workSpeed,
                            uint8_t finishSpeed,
                            uint16_t accelerationTime_ms,
                            uint16_t decelerationTime_ms);

        static void SetSerial(HardwareSerial* Serial = nullptr, uint32_t baudRate = 9600);

        void SetErrorHandler(void (*ExError)(void*), bool ignoreCommonExError = false);

        static void SetCommonErrorHandler(void (*ExError)(void*));

        /// @param interrupter Как правило"PinObject.AntirattleSensor() > time"
        bool Move(dir_t dir,
                  move_t* mv,
                  step_t* dist = NO_DISTANCE,
                  int8_t interrupter = -1);

        /// @param interrupter Как правило"PinObject.AntirattleSensor() > time"
        bool MoveToPoint(point_t* pnt,
                         move_t* mv,
                         int8_t interrupter = -1);

        /// @param momentalSpeed Количество циклов engine_step_micros = ширина шага(чем длиннее шаг, тем медленнее вращается мотор) (engine_step_micros = 10 microseconds)
        void SetCurrentSpeed(dir_t dir,
                             uint8_t currentSpeed);

        /// @param funcID Values 254,255 are reserved. In case you will use it, function will immediately return 0.
        bool ChangeSpeed(uint8_t dstSpeed, uint32_t time_ms);

        void Stop();

        static void StopAll();

        void Refresh();

        static void RefreshAll();

        void ResetCurrentPoint();

        bool GetFinish();

        /// @return in BIN (младшие три бита это номер ошибки, начиная с 1, старшие 5 бит это ID двигателя, начиная с 1)
        /// Примеры:
        /// 0b00011001 - ошибка номер 1, двигатель с ID = 3
        /// 0b000010010 - ошибка номер 2, двигатель с ID = 1
        static uint8_t GetError(MyStepper* unit = nullptr);

        uint32_t GetCurrentStep();

        int32_t GetCurrentPoint();

        uint8_t GetCurrentSpeed();

        uint8_t GetID();

        static void CleanError(MyStepper* unit = nullptr);

        static void CleanAllErrors();

        #ifndef ESP32
            static void Step();
            static void StartInterrupter();
        #endif
};