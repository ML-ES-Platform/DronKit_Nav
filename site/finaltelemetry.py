import Adafruit_DHT
import time

SENSOR_TYPE = Adafruit_DHT.DHT11
GPIO_PIN_NUMBER = 4

if __name__ == "__main__":
    start_time = time.time()
    humidity, temperature = Adafruit_DHT.read_retry(SENSOR_TYPE, GPIO_PIN_NUMBER)
    end_time = time.time()

    if humidity is not None and temperature is not None:
        print(f"Measured Temp={temperature:.1f}°C | Hum={humidity:.1f}%")
    else:
        print("Failed to retrieve data from DHT11 sensor. Check wiring and sensor type.")
    print(f"Measurement took {end_time - start_time:.2f}s")
