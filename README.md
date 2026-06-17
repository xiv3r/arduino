# Build the Firmware Binaries Automatically
> ⚠️ Only the unified .ino file is supported
1. Fork the repository
2. Edit and paste your code inside `arduino.ino`
3. Go to actions > general build > run workflows > select board > run
4. The compiled firmware is located in the release section

## Add Custom Libraries
> Find inside `.github/workflows/general.yml` and add another libraries.
```
arduino-cli lib install "ArduinoJson"
arduino-cli lib install "PubSubClient"
arduino-cli lib install "NTPClient"

# Install RTClib (external)
git clone --depth 1 --branch 1.14.1 https://github.com/adafruit/RTClib.git ~/Arduino/libraries/RTClib

```


