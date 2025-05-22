
#include <dataaverage.h>

#include <dht.h>

// Enable debug prints to serial monitor
#define MY_DEBUG

//#define MOULIN
//#define CAVE
#define EXTERIEUR
//#define SERRE
//#define ENTREE

#if defined(EXTERIEUR)
#define MY_NODE_ID 2  // extérieur sous abri
#define REPORT_BATTERY
#define NO_U
#elif defined(CAVE)
#define MY_NODE_ID 40 // cave moulin
#elif defined(MOULIN)
#define MY_NODE_ID 31 // moulin
// Enabled repeater feature for this node
#define MY_REPEATER_FEATURE
#elif defined(SERRE)
#define MY_NODE_ID 6
#define NO_U
#define REPORT_BATTERY
#elif defined(ENTREE)
#define NO_U
#define MY_NODE_ID 33 // ENTREE
#else
#define MY_NODE_ID 199 // test
#define REPORT_BATTERY
#endif

#include <MyConfigFlea.h>
// On attend le contrôleur pour démarrer
//#define MY_TRANSPORT_WAIT_READY_MS (0ul)

// Set blinking period
#define MY_DEFAULT_LED_BLINK_PERIOD 300

//#define MY_DEFAULT_ERR_LED_PIN 2  // Error led pin
//#define MY_DEFAULT_RX_LED_PIN  3  // Receive led pin
//#define MY_DEFAULT_TX_LED_PIN  4  // the PCB, on board LED

#include <MyConfig.h>
#include <MySensors.h>
#include <DallasTemperature.h>
#include <OneWire.h>

static const uint8_t ds18b20Pin = A2;
OneWire oneWire(ds18b20Pin); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature dsTemp(&oneWire); // Pass the oneWire reference to Dallas Temperature.
// to hold device address
DeviceAddress deviceTemperature;

void before()
{
    // Startup up the OneWire library
    dsTemp.begin();
    dsTemp.setResolution(12);
    if (!dsTemp.getAddress(deviceTemperature, 0))
    {
        //Serial.println("Unable to find address for first DS18B20");
    }
}

//bool metric = true;

enum {
    CHILD_ID_HUM,
    CHILD_ID_TEMP,
    CHILD_ID_TEMP_DHT,
    CHILD_ID_BATTERY
};

void presentation()
{
    //Serial.println(chainePresentation);
    // Send the sketch version information to the gateway and Controller
#if defined(EXTERIEUR)
    sendSketchInfo("Extérieur", "1.0");
#elif defined(CAVE)
    sendSketchInfo("Cave du moulin", "1.0");
#elif defined(MOULIN)
    sendSketchInfo("Moulin", "1.0");
#elif defined(SERRE)
    sendSketchInfo("Serre", "1.0");
#elif defined(ENTREE)
    sendSketchInfo("Entrée", "1.0");
#else
    sendSketchInfo("TU Test", "1.0");
#endif

    // Register all sensors to gateway (they will be created as child devices)
    present(CHILD_ID_TEMP, S_TEMP);
    //    present(CHILD_ID_TEMP_DHT, S_TEMP);
#if defined(NO_U)
    present(CHILD_ID_HUM, S_HUM);
#endif
#if defined(REPORT_BATTERY)
    present(CHILD_ID_BATTERY, S_MULTIMETER);
#endif
    //metric = getControllerConfig().isMetric;
}

#if defined(NO_U)
//static const uint8_t dhtPin = A4;
//dht1wire dht(dhtPin, dht::DHT22);
#elif defined(MOULIN)
static const uint8_t dhtPin = A3;
dht1wire dht(dhtPin, dht::DHT11);
#else
dht12 dht;
#endif

void setup()
{
    Serial.println("Node ID = " + String(getNodeId()));
    dsTemp.begin();
    // requestTemperatures() will not block current thread
    dsTemp.setWaitForConversion(false);
}

MyMessage temperatureMsg(CHILD_ID_TEMP, V_TEMP);
//MyMessage temperatureDhtMsg(CHILD_ID_TEMP_DHT, V_TEMP);
#if !defined(NO_U)
MyMessage humidityMsg(CHILD_ID_HUM, V_HUM);
#endif
#if defined(REPORT_BATTERY)
MyMessage batteryMsg(CHILD_ID_BATTERY, V_VOLTAGE);
#endif

static const uint8_t cycleCountSize = 6;
DataAverage temperatureAverage(cycleCountSize);
#if !defined(NO_U)
DataAverage humidityAverage(cycleCountSize);
#endif

void loop()
{
#if defined(REPORT_BATTERY)
    static float lastVoltage = 0;
    static const float batteryMax = 3.0;
    static const float batteryMin = 2.0;
#endif
    static const long cycleMs = 51500;
    static const long preReadMs = 1000;
    static bool startup = true;
    static uint8_t cycleCount = 0; // compteur de cycle pour la moyenne 6 minutes

    dsTemp.requestTemperatures();
#if !defined(NO_U)
    // Force reading sensor, so it works also after sleep()
    bool readDhtOk = (dht.read() == dht::OK);
#endif    sleep(preReadMs);

//#ifdef MY_DEBUG
//    Serial.print("dht.read(): ");
//    Serial.println(dht.read());
//#endif

    // Get temperature
    float temperature = dsTemp.getTempC(deviceTemperature);
    if (!isnan(temperature) && (temperature > -60) && (temperature < 85))
    {
//        if (!metric)
//        {
//            temperature = dsTemp.toFahrenheit(temperature);
//        }
        if (startup)
        {
            send(temperatureMsg.set(temperature, 1));
        }
        temperatureAverage.addSample(temperature);

#ifdef MY_DEBUG
        Serial.print("T: ");
        Serial.println(temperature);
#endif
    }

#if !defined(NO_U)
    if (readDhtOk)
    {
        // Get humidity from DHT library
        float humidity = dht.getHumidity() / 10.0;
        if (startup)
        {
            send(humidityMsg.set(humidity, 1));
            //            send(temperatureDhtMsg.set(dht.getTemperature() / 10.0, 1));
        }
        humidityAverage.addSample(humidity);

#ifdef MY_DEBUG
        Serial.print("H: ");
        Serial.println(humidity);
#endif
    }
#endif

#if defined(REPORT_BATTERY)
    // Tension batterie
    if (startup)
    {
        float voltage = lireTensionPile();
        int batteryPcnt = 100 * (voltage - batteryMin) / (batteryMax - batteryMin);
        if (batteryMin > voltage)
        {
            batteryPcnt = 0;
        }
        if (startup)
        {
            send(batteryMsg.set(voltage, 2));
            sendBatteryLevel(batteryPcnt);
#ifdef MY_DEBUG
            Serial.print("V: ");
            Serial.print(voltage);
            Serial.print(" (");
            Serial.print(batteryPcnt);
            Serial.println("%)");
#endif
        }
    }
#endif
    cycleCount++;
    if (cycleCount == cycleCountSize)
    {
      if (temperatureAverage.sampleCount() > 0)
      {
        send(temperatureMsg.set(temperatureAverage.average(), 1));
      }
#if !defined(NO_U)
      if (humidityAverage.sampleCount() > 0)
      {
        send(humidityMsg.set(humidityAverage.average(), 0));
      }
#endif
#if defined(REPORT_BATTERY)
        // Tension batterie
        float voltage = lireTensionPile();
        if (voltage != lastVoltage)
        {
            lastVoltage = voltage;
            int batteryPcnt = 100 * (voltage - batteryMin) / (batteryMax - batteryMin);
            if (batteryMin > voltage)
            {
                batteryPcnt = 0;
            }
            send(batteryMsg.set(voltage, 2));
            sendBatteryLevel(batteryPcnt);
#ifdef MY_DEBUG
            Serial.print("V: ");
            Serial.print(voltage);
            Serial.print(" (");
            Serial.print(batteryPcnt);
            Serial.println("%)");
#endif
        }
#endif
        cycleCount = 0;
    }
    if (startup)
    {
        startup = false;
    }
    sleep(cycleMs - preReadMs);
}

#if defined(REPORT_BATTERY)
float lireTensionPile()
{
    // Tension batterie
    unsigned int reference1v1 = analogReadReference();
    return (float)analogRead(A6) * 1.1 / (float)reference1v1;
}

/** Mesure la référence interne à 1.1 volts */
unsigned int analogReadReference(void)
{

    /* Elimine toutes charges résiduelles */
    ADMUX = 0x4F;
    delayMicroseconds(5);

    /* Sélectionne la référence interne à 1.1 volts comme point de mesure, avec comme limite haute VCC */
    ADMUX = 0x4E;
    delayMicroseconds(200);

    /* Active le convertisseur analogique -> numérique */
    ADCSRA |= (1 << ADEN);

    /* Lance une conversion analogique -> numérique */
    ADCSRA |= (1 << ADSC);

    /* Attend la fin de la conversion */
    while(ADCSRA & (1 << ADSC));

    /* Récupère le résultat de la conversion */
    return ADCL | (ADCH << 8);
}
#endif
