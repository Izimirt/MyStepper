#pragma once

#include <Arduino.h>
#include <math.h>

#include "Const.h"
#include "Structs.h"

class MyStepper 
{
    public:

        MyStepper();

        MyStepper(uint8_t stepPin,
                  uint8_t dirPin,
                  uint8_t enPin,
                  bool freeStay);

        void SetEngine(uint8_t stepPin,
                       uint8_t dirPin,
                       uint8_t enPin,
                       bool freeStay);

        static void SetSteps(st_step_t** setPtr,
                             uint32_t steps,
                             uint32_t understeps = 0,
                             uint32_t oversteps = 0);

        void SetPoint(st_point_t** setPtr,
                      int32_t point,
                      int32_t understeps = 0,
                      int32_t oversteps = 0,
                      bool noUndersteps = false,
                      uint16_t pointNumber = 0);

        void SetPointInArea(st_point_t** setPtr,
                            st_point_t* minEdge,
                            st_point_t* maxEdge,
                            int32_t point,
                            int32_t understeps = 0,
                            int32_t oversteps = 0,
                            bool noUndersteps = false,
                            uint16_t pointNumber = 0);

        /// @param maxSpeed (minSpeed) Количество циклов engine_step_micros = ширина шага(чем длиннее шаг, тем медленнее вращается мотор) (engine_step_micros = 10 microseconds)
        static void SetMove(st_move_t** setPtr,
                            uint8_t startSpeed,
                            uint8_t workSpeed,
                            uint8_t finishSpeed,
                            uint16_t accelerationTime_ms,
                            uint16_t decelerationTime_ms);

        static void SetSerial(HardwareSerial* Serial = nullptr, uint32_t baudRate = 9600);

        void SetErrorHandler(void (*ExError)(void*), bool ignoreCommonExError = false);

        static void SetCommonErrorHandler(void (*ExError)(void*));

        /// @param interrupter Как правило"PinObject.AntirattleSensor() > time"
        bool Move(st_dir_t dir,
                  st_move_t* mv,
                  st_step_t* dist = NO_DISTANCE,
                  int8_t interrupter = -1);

        /// @param interrupter Как правило"PinObject.AntirattleSensor() > time"
        bool MoveToPoint(st_point_t* pnt,
                         st_move_t* mv,
                         int8_t interrupter = -1);

        /// @param momentalSpeed Количество циклов engine_step_micros = ширина шага(чем длиннее шаг, тем медленнее вращается мотор) (engine_step_micros = 10 microseconds)
        void SetCurrentSpeed(st_dir_t dir,
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
        /// 0b00010010 - ошибка номер 2, двигатель с ID = 1
        uint8_t GetError();

        /// Пример:
        /// 0b00000111 - ошибка номер 7, общая, ID = 0
        static uint8_t GetStaticError();

        uint32_t GetCurrentStep();

        int32_t GetCurrentPoint();

        uint8_t GetCurrentSpeed();

        uint8_t GetID();

        void CleanError();

        static void CleanStaticError();

        static void CleanAllErrors();

        #ifndef ESP32
            static void Step();
            static void StartInterrupter();
        #endif

    private:

        #ifdef ESP32
            static hw_timer_t* interrupter;
            static void Step();
        #endif

        static void Error(st_err_t error, MyStepper* unit = nullptr);

        void InternalSetCurrentSpeed(st_dir_t dir, uint8_t currentSpeed);

        bool InternalChangeSpeed(st_accel_t* accel, bool refresh);

        bool InternalMove(st_move_t* mv,
                          st_point_t* pnt = nullptr,     // for MoveToPiont()
                          st_step_t* dist = nullptr,     // for Move()
                          st_dir_t dir = FWD);           // for Move()

        void InternalRefresh();

        /// @brief This function is counting acceleration parameters for laiter counts.
        static void CountAccel(st_accel_t* accel, uint8_t bgnSpeed,uint8_t dstSpeed, uint32_t time_ms);

        bool CountSteps(st_accel_t* accel);

        bool CheckNeedCountSteps(st_accel_t* accel);
        
        uint8_t stepPin;
        uint8_t dirPin;
        uint8_t enPin;
        bool freeStay;

        uint32_t currentStep = 0;
        int32_t currentPoint = 0;
        st_dir_t direction;

        bool moveFlag = false;
        bool finishFlag = false;
        bool brakeFlag = false;
        bool manualFlag = false;
        bool stopFlag = false;

        bool stepState = false;
        uint8_t speed;
        uint32_t previousMs;
        uint32_t accelerationSteps = 0;
        uint16_t timer = 0;

        uint16_t currentUnrealNumStepsPerPeriod;    
        uint8_t speedCounter = 0;         

        st_phase_t phase = START;
        st_accel_t tmpAccel;
        st_accel_t* ptrTmpAccel = nullptr;
        st_accel_t* lastFinishAccel = nullptr;
        st_brake_t* currentLvl;
        bool accelSuccess = false;
        uint32_t internalDistance;

        uint8_t prevDstSpeed = 0;
        uint32_t prevTime_ms = 0;
        st_dir_t prevDirection;
        st_move_t* prevMove = nullptr;
        st_step_t* prevDistance = nullptr;
        st_point_t* prevPoint = nullptr;

        st_point_t* pPtrOnHead = nullptr;
        st_brake_t* bPtrOnHead = nullptr;
        st_brake_t* bPtrOnTail = nullptr;

        uint8_t errorCommand = 0;
        static uint8_t staticErrorCommand;
        void (*ExError)(void*) = nullptr;
        static void (*CommonExError)(void*);
        bool ignoreCommonExError = false;

        uint8_t ID;
        static uint8_t numSteppers;
        static uint8_t interrupterStep_us;

        MyStepper* ptrOnOther;
        static MyStepper* currentPtr;
        static MyStepper* unitPtr;

        static HardwareSerial* MySerial;
};