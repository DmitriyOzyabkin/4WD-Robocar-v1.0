#include <Arduino.h>

// --- НАСТРОЙКА ОТЛАДКИ ---
// Закомментируйте следующую строку, чтобы полностью выключить всю отладку 
// для финальной версии прошивки:
#define DEBUG_ENABLED 

#ifdef DEBUG_ENABLED
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
#else
    #define DEBUG_PRINT(x) ((void)0) // Ничего не делает
    #define DEBUG_PRINTLN(x) ((void)0) // Ничего не делает
#endif


#include <iarduino_HC_SR04.h>
#include <L298NX2.h>

// --- 1. ИСПОЛЬЗОВАНИЕ ПЕРЕЧИСЛЕНИЙ (ENUM) И СТРУКТУР (STRUCT) ---
// Это делает код самодокументируемым и защищает от ошибок.

// Перечисление направлений для маневров робота
enum class Side {
    Left,  // Поворот налево
    Right  // Поворот направо
};

// Перечисление для пар моторов (передняя/задняя)
// Не задействовано в проекте 4WD Car v1.0
enum class MotorPair {
    FrontPair, // Передняя пара
    BackPair,  // Задняя пара
    LeftPair,  // Левая пара
    RightPair  // Правая пара
};

// Структура для хранения конфигурации ОДНОГО драйвера L298NX2
struct MotorDriverConfig {
    // Конфигурация для Мотора А
    uint8_t enablePinA;  // Пин EN_A (ШИМ)
    uint8_t input1PinA;  // Пин IN1_A
    uint8_t input2PinA;  // Пин IN2_A

    // Конфигурация для Мотора B
    uint8_t enablePinB;  // Пин EN_B (ШИМ)
    uint8_t input1PinB;  // Пин IN1_B
    uint8_t input2PinB;  // Пин IN2_B
};

// --- 2. ОБЪЯВЛЕНИЕ КОНСТАНТ И ГЛОБАЛЬНЫХ ПЕРЕМЕННЫХ ---

// --- БАЗОВЫЕ СКОРОСТИ И КОЭФФИЦИЕНТЫ МОЩНОСТИ ---
const uint8_t NORMAL_SPEED = 150; // Максимальная скорость (0-255)
const float SLOW_MODE_POWER = 0.7; // Для движения вблизи препятствий
const float BACKWARD_MODE_POWER = 0.7; // Мощность движение назад, отъезд от препятствия
const float TURNING_MODE_POWER = 0.5; // Снижение мощности при повороте на месте, для плавности
// Коэффициенты для маневра уклонения (Tuning parameters)
const float TURN_INSIDE_POWER = 0.3; // Мощность внутренней стороны при повороте
const float TURN_OUTSIDE_POWER = 0.7; // Мощность внешней стороны при повороте

// --- РАССТОЯНИЯ ДЛЯ ОПРЕДЕЛЕНИЯ ЗОН ПОВЕДЕНИЯ ---
const int16_t DISTANCE_CRITICAL = 20; 
const int16_t DISTANCE_SLOW_ZONE = 50;
const int16_t DISTANCE_SAFE_ZONE = 100;

// --- ЗАДЕРЖКИ ДЛЯ ВЫПОЛНЕНИЯ МАНЕВРОВ ---
const uint16_t BACKUP_DELAY = 500; // Задержка при движении назад (мс)
const uint16_t TURN_DELAY = 500;   // Задержка при повороте (мс)

// Переменные для хранения и анализа изменения расстояния при движении
// и уклонении от препятствия
uint16_t CURRENT_DISTANCE;
uint16_t LAST_DISTANCE;
unsigned long obstacleAvoidanceTimer = 0; // Таймер длительности маневра
bool isAvoiding = false;                  // Флаг: находимся ли мы сейчас в режиме уклонения?
Side currentAvoidanceDir;                 // Куда поворачиваем прямо сейчас?
const unsigned long AVOIDANCE_DURATION = 1000; // Длительность попытки уклонения (мс)

// Полная конфигурация пинов для левого драйвера (моторы LF и LB)
const MotorDriverConfig LEFT_DRIVER_CONFIG = {
    .enablePinA = 3, .input1PinA = 2, .input2PinA = 4, // Мотор LF (A)
    .enablePinB = 5, .input1PinB = 6, .input2PinB = 7  // Мотор LB (B)
};

// Полная конфигурация пинов для правого драйвера (моторы RF и RB)
const MotorDriverConfig RIGHT_DRIVER_CONFIG = {
    .enablePinA = 9, .input1PinA = 8, .input2PinA = 10, // Мотор RF (A)
    .enablePinB = 11, .input1PinB = 12, .input2PinB = 13 // Мотор RB (B)
};

/// Создаем объект для левого драйвера, используя его полную конфигурацию
L298NX2 leftMotors(
    LEFT_DRIVER_CONFIG.enablePinA,
    LEFT_DRIVER_CONFIG.input1PinA,
    LEFT_DRIVER_CONFIG.input2PinA,
    LEFT_DRIVER_CONFIG.enablePinB,
    LEFT_DRIVER_CONFIG.input1PinB,
    LEFT_DRIVER_CONFIG.input2PinB
);

// Создаем объект для правого драйвера, используя его полную конфигурацию
L298NX2 rightMotors(
    RIGHT_DRIVER_CONFIG.enablePinA,
    RIGHT_DRIVER_CONFIG.input1PinA,
    RIGHT_DRIVER_CONFIG.input2PinA,
    RIGHT_DRIVER_CONFIG.enablePinB,
    RIGHT_DRIVER_CONFIG.input1PinB,
    RIGHT_DRIVER_CONFIG.input2PinB
);

// Конфигурация пинов для датчика расстояния
iarduino_HC_SR04 sensor(A0, A1); // Используем имена A0, A1 для наглядности

// --- 3. РЕАЛИЗАЦИЯ ФУНКЦИЙ ---

int updateSpeed(float factor) {

    // Функция изменения значения скорости.
    // Принимает на вход фактор изменения
    // и возвращает целое значение измененной скорости

    float rawSpeed = NORMAL_SPEED * factor; // Изменение скрости
        // Ограничиваем значения строго в диапазоне 0..NORMAL_SPEED
        uint8_t finalSpeed = constrain((uint8_t)(rawSpeed + 0.5), 0, NORMAL_SPEED);
        return finalSpeed;
}

void setPairMotorsSpeed(MotorPair pair, uint8_t speed){
    
    // Функция устанавливает скорость определенной пары моторов

    switch(pair) {
        case MotorPair::FrontPair:
            leftMotors.setSpeedA(speed);
            rightMotors.setSpeedA(speed);
            break;
        case MotorPair::BackPair:
            leftMotors.setSpeedB(speed);
            rightMotors.setSpeedB(speed);
            break;
        case MotorPair::LeftPair:
            leftMotors.setSpeedA(speed);
            leftMotors.setSpeedB(speed);
            break;
        case MotorPair::RightPair:
            rightMotors.setSpeedA(speed);
            rightMotors.setSpeedB(speed);
            break;
    }
}

void setupAllMotors(uint8_t speed) {

    // Функция принимает на вход значение скорости и устанавливаем его для всех моторов

    leftMotors.setSpeedA(speed);
    leftMotors.setSpeedB(speed);
    rightMotors.setSpeedA(speed);
    rightMotors.setSpeedB(speed);
}

void setupSensor() {
    // Библиотека iarduino_HC_SR04 сама настраивает пины, ручная настройка не требуется.
}

void moveForward(uint8_t speed) {

    // Функия движения вперед. Аргумент функции скорость, устанавливается для всех моторов

    setupAllMotors(speed);
    leftMotors.forwardA();
    rightMotors.forwardA();
    leftMotors.backwardB();
    rightMotors.backwardB();
}

void moveBackward(uint8_t speed) {

    // Функция движения назад. Аргумент функции скорость, устанавливаеться для всех моторов
    setupAllMotors(speed);
    leftMotors.backwardA();
    rightMotors.backwardA();
    leftMotors.forwardB();
    rightMotors.forwardB();
}

void stopAllMotors() {

    // Остановка всех двигателей

    leftMotors.stopA();
    leftMotors.stopB();
    rightMotors.stopA();
    rightMotors.stopB();
}

void turn(Side direction) {

    // Поворот на месте. Примерно на 90 градусов. 
    // Аргумент функции Side сторона в которую необходимо повернуть
    // В случае, если на входе фенкции неопределенное значение, двигатели останавливаются

    switch(direction) {
        case Side::Left:
            // Логика поворота налево
            setupAllMotors(updateSpeed(TURNING_MODE_POWER));
            leftMotors.backwardA(); // Левая сторона назад
            rightMotors.forwardA(); // Правая сторона едет вперед
            leftMotors.forwardB();
            rightMotors.backwardB();
            break;
        case Side::Right:
            // Логика поворота направо
            setupAllMotors(updateSpeed(TURNING_MODE_POWER));
            leftMotors.forwardA();  // Левая сторона едет вперед
            rightMotors.backwardA(); // Правая сторона движется назад
            leftMotors.backwardB();
            rightMotors.forwardB();
            break;

        default:
            // Это "страховочный" блок.
            // Если вдруг придет неизвестное значение,
            // мы просто ничего не будем делать (или остановим моторы)
            stopAllMotors();
            break;
    }
}

void avoidObstacle(Side direction) {

    // Функция для обхода препятствия. Агрумент функции - сторона,
    // в которую нужно начать смещение относотельно курса
    
    switch(direction){
        case Side::Left:
            // Левая сторона притормаживает
            setPairMotorsSpeed(MotorPair::LeftPair, updateSpeed(TURN_INSIDE_POWER));
            setPairMotorsSpeed(MotorPair::RightPair, updateSpeed(TURN_OUTSIDE_POWER));
            leftMotors.forwardA();
            rightMotors.forwardA();
            leftMotors.backwardB();
            rightMotors.backwardB();
            break;
        case Side::Right:
            // Правая сторона притормаживает
            setPairMotorsSpeed(MotorPair::RightPair, updateSpeed(TURN_INSIDE_POWER));
            setPairMotorsSpeed(MotorPair::LeftPair, updateSpeed(TURN_OUTSIDE_POWER));
            leftMotors.forwardA();
            rightMotors.forwardA();
            leftMotors.backwardB();
            rightMotors.backwardB();
            break;
    }
}

int getDistance() {

    // Функия измерения дистанции. 

    int dist = sensor.distance();
    if (dist == -1) return 9999; // Возвращаем "бесконечность", чтобы робот ехал вперед
    return dist;
}

// --- 4. ОСНОВНЫЕ НАСТРОЙКИ И ИНИЦИАЛИЗАЦИИ ---

void setup() {
    Serial.begin(9600);
    setupAllMotors(NORMAL_SPEED);
    setupSensor();   // Инициализация датчика
    DEBUG_PRINTLN("Робот инициализирован. Начинаем движение.");
}

// --- 5. ОСНОВНОЙ ЦИКЛ ПРОГРАММЫ ---

void loop() {

    int distance = getDistance();

    if (distance >= 0) { // Проверяем, что датчик вернул валидное значение
            DEBUG_PRINT("Расстояние: ");
            DEBUG_PRINT(distance);
            DEBUG_PRINTLN(" см");
        }
    if (distance >= DISTANCE_SAFE_ZONE) { 
        // Движение вперед на нормальной скорости.
        // Робот в SAFE_ZONE, препятствие дальше 100 см)
        moveForward(NORMAL_SPEED);
        DEBUG_PRINT("Препятствия нет. Движение вперед. Скорость: ");
        DEBUG_PRINTLN(NORMAL_SPEED);
    } 

    // Если расстояние от 50 до 100 см, то есть робот в SLOW_ZONE
    // снижаем скорость на 30%
    if (distance >= DISTANCE_SLOW_ZONE && distance < DISTANCE_SAFE_ZONE) {

        moveForward(updateSpeed(SLOW_MODE_POWER));
        DEBUG_PRINT("Обнаружено препятствие. Движение вперед. Скорость снижена. Скорость: ");
        DEBUG_PRINTLN(updateSpeed(SLOW_MODE_POWER));
    }
    // --- МАНЕВР УКЛОНЕНИЯ ---
    // 1. Если робот НЕ в процессе уклонения,
    // но увидел препятствие на расстоянии от 20 до 50 см (CRITICAL_ZONE)
    if (!isAvoiding && distance > DISTANCE_CRITICAL && distance < DISTANCE_SLOW_ZONE) {

        DEBUG_PRINTLN("Начинаем маневр уклонения.");

        // Инициализируем параметры маневра
        isAvoiding = true;
        LAST_DISTANCE = distance;
        currentAvoidanceDir = random(0, 2) == 0 ? Side::Left : Side::Right;

        // Заводим таймер
        obstacleAvoidanceTimer = millis(); 

        // Даем команду начать уклонение
        avoidObstacle(currentAvoidanceDir);

        DEBUG_PRINT("Уклонение: ");
        DEBUG_PRINTLN(currentAvoidanceDir == Side::Left ? "НАЛЕВО" : "НАПРАВО");
    }
    // 2. Если мы УЖЕ в процессе уклонения
    if (isAvoiding) {

        // ОБЯЗАТЕЛЬНО постоянно обновляем данные с датчика!
        CURRENT_DISTANCE = getDistance(); 

        // Проверяем, не стало ли хуже (упираемся в угол)
        if (CURRENT_DISTANCE < LAST_DISTANCE) {
            DEBUG_PRINTLN("Препятствие ближе! Меняем направление.");

            // Меняем направление поворота
            currentAvoidanceDir = (currentAvoidanceDir == Side::Left) ? Side::Right : Side::Left;
            avoidObstacle(currentAvoidanceDir);

            // Обновляем контрольную точку расстояния
            LAST_DISTANCE = CURRENT_DISTANCE; 

            // Сбрасываем таймер, даем новой попытке полные 1000 мс
            obstacleAvoidanceTimer = millis(); 
        }
        // Проверяем, прошло ли заданное время (ваш старый delay)
        if (millis() - obstacleAvoidanceTimer >= AVOIDANCE_DURATION) {

            DEBUG_PRINTLN("Время маневра истекло. Проверяем результат.");

            // Время вышло. Выясняем, свободен ли путь теперь?
            if (getDistance() >= DISTANCE_SLOW_ZONE) {
                // Путь чист, выходим из режима уклонения
                DEBUG_PRINTLN("Путь свободен. Возврат к нормальному движению.");
                isAvoiding = false;
                moveForward(NORMAL_SPEED); // Возвращаемся к обычному режиму
            } else {
                // Путь все еще закрыт. Продолжаем тот же маневр еще секунду.
                DEBUG_PRINTLN("Путь все еще занят. Продолжаем...");
                obstacleAvoidanceTimer = millis(); // Просто перезапускаем таймер
            }
        }
    }
    
    if (distance <= DISTANCE_CRITICAL) {
        // Если расстояние менее 20 см (STOP_ZONE), 
        // остановка и поворот в случайную сторону
        
        isAvoiding = false;    // Проверка, вернется ли в режим уклонения после отката от препятствия?
        
        stopAllMotors();
        delay(BACKUP_DELAY);
        DEBUG_PRINTLN("Остановка.");
        moveBackward(updateSpeed(BACKWARD_MODE_POWER));
        delay(BACKUP_DELAY);
        DEBUG_PRINTLN("Движение назад.");
        stopAllMotors();
        DEBUG_PRINTLN("Остановка.");
        delay(200); // Короткая пауза перед поворотом

        // Случайный поворот: 0 - Left, 1 - Right
        Side turnDirection = random(0, 2) == 0 ? Side::Left : Side::Right;
        DEBUG_PRINT("Поворот: ");
        DEBUG_PRINTLN(turnDirection == Side::Left ? "НАЛЕВО" : "НАПРАВО");
        turn(turnDirection);
        delay(TURN_DELAY);
        stopAllMotors();
        delay(200);
    }

}

