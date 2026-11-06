# Moth application

This is an out-of-tree Zephyr application for the QCC744M EVK. The current
pipeline is:

```text
QCC74x ADC driver -> ADC reader -> light acquisition -> phototaxis controller
```

The two ADC inputs are provisionally mapped in
`boards/qcc744m_evk.overlay`:

- left sensor: GPIO20 / ADC channel 0
- right sensor: GPIO19 / ADC channel 1

`adc_reader.c` obtains the channel specifications from devicetree and returns
raw samples. `light_acquisition.c` samples at 20 Hz, filters them, and
publishes frames through a bounded message queue. `phototaxis.c` compares the
two light values and logs a forward, left, right, or stop request. No motor
driver is connected yet.

Phototaxis tuning is configured through the application Kconfig symbols:

- `CONFIG_MOTH_PHOTOTAXIS_MINIMUM_LIGHT`
- `CONFIG_MOTH_PHOTOTAXIS_DEADBAND`
- `CONFIG_MOTH_PHOTOTAXIS_BASE_SPEED`
- `CONFIG_MOTH_PHOTOTAXIS_TURN_GAIN`
- `CONFIG_MOTH_PHOTOTAXIS_MAXIMUM_SPEED`

They can be changed through `menuconfig` or supplied in `prj.conf`.

## Build

From the repository root, with the BSP workspace initialized:

```powershell
$env:ZEPHYR_BASE=(Resolve-Path .\zephyr).Path
$env:ZEPHYR_TOOLCHAIN_VARIANT='zephyr'
$env:ZEPHYR_SDK_INSTALL_DIR='C:\Users\JakubKwintal\zephyr-sdk-0.17.0'
.\.venv\Scripts\west.exe build -p always -b qcc744m_evk moth_app -d build\moth_app
```
