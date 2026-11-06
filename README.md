# Moth application

This is an out-of-tree Zephyr application for the QCC744M EVK.

```text
Zephyr BSP -> moth application
```

## Build

From the repository root, with the BSP workspace initialized:

```powershell
$env:ZEPHYR_BASE=(Resolve-Path .\zephyr).Path
$env:ZEPHYR_TOOLCHAIN_VARIANT='zephyr'
$env:ZEPHYR_SDK_INSTALL_DIR='C:\Users\JakubKwintal\zephyr-sdk-0.17.0'
.\.venv\Scripts\west.exe build -p always -b qcc744m_evk moth_app -d build\moth_app
```
