#pragma once

#include <Arduino.h>

class MyStepper 
{
    public:

        MyStepper();

        MyStepper(uint16_t stepPin,
                  uint16_t dirPin,
                  uint16_t enPin,
                  bool freeStay,
                  void (*Error)(void*) = nullptr);

        void SetEngine(uint16_t stepPin,
                       uint16_t dirPin,
                       uint16_t enPin,
                       bool freeStay,
                       void (*Error)(void*));

        void SetSteps(uint16_t distanceNumber,
                      uint32_t steps,
                      uint32_t understeps = 0,
                      uint32_t oversteps = 0);

        void SetPoint(uint16_t pointNumber,
                      int32_t point,
                      int32_t understeps = 0,
                      int32_t oversteps = 0);

        /// @param maxSpeed (minSpeed) Количество циклов engine_step_micros = ширина шага(чем длиннее шаг, тем медленнее вращается мотор) (engine_step_micros = 10 microseconds)
        void SetMove(uint16_t minSpeed,
                     uint16_t maxSpeed,
                     uint16_t accelerationTimeMsec,
                     uint8_t setNumber = 0);

        /// @param interrupter Как правило"PinObject.AntirattleSensor() > time"
        bool Move(bool direction,
                  int8_t setNumber,
                  int8_t distanceNumber = -1,
                  int8_t interrupter = -1,
                  bool softBrake = 0);

        /// @param interrupter Как правило"PinObject.AntirattleSensor() > time"
        bool MoveToPoint(int8_t pointNumber,
                         int8_t setNumber,
                         int8_t interrupter = -1);

        /// @param momentalSpeed Количество циклов engine_step_micros = ширина шага(чем длиннее шаг, тем медленнее вращается мотор) (engine_step_micros = 10 microseconds)
        void ManualMove(bool direction,
                        uint16_t momentalSpeed);

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
        uint8_t GetError();

        uint32_t GetCurrentStep();

        int32_t GetCurrentPoint();

        int32_t GetPoint(uint16_t point);

        uint8_t GetID();

        void CleanError();

        #ifndef ESP32
            static void Step();
            static void StartInterrupter();
        #endif

    private:

        typedef enum
        {
            UNDERSTEP = 0b1,
            OVERSTEP = 0b10
        } err_t;

        #ifdef ESP32 
            static void Step();
        #endif

        uint8_t SetError(err_t error);

        static const uint16_t MSetNumber = 5;
        static const uint16_t SSetNumber = 10;
        static const uint16_t PSetNumber = 20;

        struct MoveSettings
        {
            uint16_t minSpeed;
            uint16_t maxSpeed;
            uint16_t millisStep;
        };
        MoveSettings MSet[MSetNumber];

        struct StepSettings
        {
            uint32_t distance;
            uint32_t understeps;
            uint32_t oversteps;
        };
        StepSettings SSet[SSetNumber];

        struct PointSettings
        {
            int32_t point;
            int32_t understeps;
            int32_t oversteps;
        };
        PointSettings PSet[PSetNumber];

        uint16_t stepPin;
        uint16_t dirPin;
        uint16_t enPin;
        bool freeStay;

        uint8_t errorCommand = 0;
        uint32_t currentStep = 0;
        int32_t currentPoint = 0;
        uint32_t distanceToPoint;
        bool direction;

        bool moveFlag = false;
        bool finishFlag = false;
        bool brakeFlag = false;
        bool manualFlag = false;
        bool stopFlag = false;

        bool stepState = false;
        uint16_t speed;
        uint32_t previousMillis;
        uint32_t accelerationSteps = 0;
        uint16_t timer = 0;

        MyStepper* ptrOnOther;
        uint8_t ID;

        static uint8_t numSteppers;
        static uint8_t interrupterStepMicros;
        static MyStepper* currentPtr;
        static MyStepper* unitPtr;
        #ifdef ESP32
            static hw_timer_t* interrupter;
        #endif

        void (*Error)(void*) = nullptr;
};