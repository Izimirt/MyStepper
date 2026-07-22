#pragma once

#include <Arduino.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt.h"

#include "Const.h"
#include "Structs.h"

typedef void (*FinishCallback)(); 
typedef void (*ExtPinCallback)(uint8_t motorID, bool state);

class MyStepper 
{
    public:

        MyStepper();

        MyStepper(uint8_t stepPin,
                  uint8_t dirPin = 0,
                  uint8_t enPin = 0,
                  bool powerStay = true);

        void SetEngine(uint8_t stepPin,
                       uint8_t dirPin = 0,
                       uint8_t enPin = 0,
                       bool powerStay = true);

        static void SetSteps(st_step_t* stepStruct,
                             uint32_t steps,
                             uint32_t understeps = 0,
                             uint32_t oversteps = 0);

        void SetPoint(st_point_t* pntStruct,
                      int32_t point,
                      int32_t understeps,
                      int32_t oversteps);

        void SetPointInArea(st_point_t* pntStruct,
                            st_point_t minEdgePnt,
                            st_point_t maxEdgePnt,
                            int32_t point,
                            int32_t understeps = 0,
                            int32_t oversteps = 0);


        static void SetMove(st_move_t* mvStruct,
                            uint32_t v_start_hz,
                            uint32_t v_work_hz,
                            uint32_t v_fin_hz,
                            uint32_t t_accel_ms,
                            uint32_t t_decel_ms);

        static void SetSerial(HardwareSerial* Serial = nullptr, uint32_t baudRate = 9600);

        void SetErrorHandler(void (*ExError)(void*), bool ignoreCommonExError = false);

        static void SetCommonErrorHandler(void (*ExError)(void*));

        /// @param interrupter Как правило"PinObject.AntirattleSensor() > time"
        bool Move(st_dir_t dir,
                  st_move_t mv,
                  st_step_t dist,
                  int8_t* interrupter = nullptr);

        /// @param interrupter Как правило"PinObject.AntirattleSensor() > time"
        bool MoveToPoint(st_point_t* pnt,
                         st_move_t* mv,
                         int8_t interrupter = -1);

        /// @param momentalSpeed Количество циклов engine_step_micros = ширина шага(чем длиннее шаг, тем медленнее вращается мотор) (engine_step_micros = 10 microseconds)
        void SetCurrentSpeed(st_dir_t dir,
                             uint16_t currentSpeed);

        bool ChangeSpeed(uint16_t dstSpeed, uint32_t time_ms);

        void Stop();

        static void StopAll();

        void Refresh();

        static void RefreshAll();

        bool GetFinish();

        static bool GetFinishAll();

        void ResetCurrentPoint();

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

        uint16_t GetCurrentSpeed();

        uint8_t GetID();

        void CleanError();

        static void CleanStaticError();

        static void CleanAllErrors();

        static void SetExtDirCallback(ExtPinCallback cb);
        static void SetExtEnCallback(ExtPinCallback cb);

    private:

        typedef enum class MoveType : uint8_t
        {
            STOP,
            DISTANCE,
            POINT,
            MANUAL
        } mv_t;

        // Обертка для FreeRTOS (обходит ограничение C++ на указатели функций) 
        static void TaskWrapper(void* arg);

        void MoveTask();


        static hw_timer_t* interrupter;
        static void Step();

        void SoftStop();

        static void Error(st_err_t error, MyStepper* unit = nullptr);

        void InternalSetCurrentSpeed(st_dir_t dir, uint16_t currentSpeed);

        bool InternalChangeSpeed(st_accel_t* accel, bool refresh);

        bool InternalMove(st_move_t* mv,
                          st_point_t* pnt = nullptr,     // for MoveToPiont()
                          st_step_t* dist = nullptr,     // for Move()
                          st_dir_t dir = st_dir::FWD);   // for Move()

        void InternalRefresh();

        /// @brief This function is counting acceleration parameters for laiter counts.
        static void CountAccel(st_accel_t** accelPtr, uint16_t bgnSpeed,uint16_t dstSpeed, uint32_t time_ms);

        bool CountSteps(st_accel_t* accel);

        bool CheckNeedCountSteps(st_accel_t* accel);
        
        uint8_t stepPin;
        uint8_t dirPin;
        uint8_t enPin;
        bool powerStay;

        volatile bool* interrupter = nullptr;
        FinishCallback OnFinishCb = nullptr;

        static ExtPinCallback dirCallback;
        static ExtPinCallback enCallback;

        TaskHandle_t taskHandle = nullptr;
        QueueHandle_t taskMailbox = nullptr;

        mv_t mv = mv_t::STOP;

        st_move_t* move = nullptr;
        st_dist_t* distance = nullptr;
        st_point_t* point = nullptr;



        uint32_t currentStep = 0;
        int32_t currentPoint = 0;
        st_dir_t direction;

        volatile bool moveFlag = false;
        volatile bool finishFlag = false;

        bool brakeFlag = false;
        bool manualFlag = false;
        bool stopFlag = false;

        bool stepState = false;
        uint16_t speed;
        uint32_t previousUs;
        uint32_t accelerationSteps = 0;
        uint16_t timer = 0;

        float currentUnrealNumStepsPerPeriod;    
        uint16_t speedCounter = 0;         

        st_phase_t phase = START;
        st_accel_t* ptrTmpAccel = nullptr;
        st_accel_t* lastFinishAccel = nullptr;
        bool accelSuccess = false;
        uint32_t internalDistance;

        uint16_t prevDstSpeed = 0;
        uint32_t prevTime_ms = 0;
        st_dir_t prevDirection;
        st_move_t* prevMove = nullptr;
        st_step_t* prevDistance = nullptr;
        st_point_t* prevPoint = nullptr;

        st_point_t* pPtrOnHead = nullptr;
        st_brake_t* bPtrOnHead = nullptr;
        st_brake_t* bPtrOnTail = nullptr;
        st_brake_t* currentLvl = nullptr;

        static uint16_t brakeLvlMaxAmoumt;

        uint8_t errorCommand = 0;
        static uint8_t staticErrorCommand;
        void (*ExError)(void*) = nullptr;
        static void (*CommonExError)(void*);
        bool ignoreCommonExError = false;

        uint8_t ID;
        static uint8_t numSteppers;
        rmt_channel_t rmt_channel;




        MyStepper* ptrOnOther;
        static MyStepper* currentPtr;
        static MyStepper* unitPtr;

        static HardwareSerial* MySerial;
};