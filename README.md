# How to build the Firmware?
> ⚠️ Only the .ino file is supported
1. Fork the repository
2. Edit and paste your code inside `arduino.ino`
3. Go to actions > general buil > run workflows
4. The compiled firmware is located in the release section

# Custom libraries
> Find inside `.github/workflows/general.yml`
```
arduino-cli lib install "ArduinoJson"
arduino-cli lib install "NTPClient"

# Install RTClib (external)
git clone --depth 1 --branch 1.14.1 https://github.com/adafruit/RTClib.git ~/Arduino/libraries/RTClib

```
