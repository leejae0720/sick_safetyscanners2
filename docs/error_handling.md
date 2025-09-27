# Error Handling

## Error Codes

The SICK laser scanners show error codes on the on-device displays in case of a hardware or configuration fault.
These codes are hexadecimal numbers.

They can also be accessed with the `status_overview` service.
The service response field `error_info_code` contains the same error code displayed on the display, but encoded as decimal number.
A code of `0` indicates that no error is present.
The following table gives some guidance in interpreting the error code.

| Hexadecimal Code | Decimal Code | Error class | Short description |
| ---------------- | ------------ | ----------- | ----------------- |
| 0x-A-xxxxxxx     | 2684354560 - 2952790015 | Error_Level_APP | **Application errors (A)** are related to issues that arise with Input/Output (I/O) signals, e.g., control inputs, OSSD's, or application tasks within the configuration, e.g., a control signal detected, but not configured. <br /> Application is stopped. <br /> These errors must be corrected either in the configuration and/or external circuits/controllers. A device restart (of safety function) or starting the application is required to clear these errors. |
| 0x-C-xxxxxxx     | 3221225472 - 3489660927 | Error_Level_CONFIG | **Configuration errors (C)** are related to issues of the configuration data (e.g., corrupted data, defective system plug), configuration compatibility (e.g., configuration does not match the physical hardware), or firmware issues (e.g., compatibility between firmware and configuration). <br /> Application is stopped. <br /> These errors require either a reconfiguration or correction of the hardware used. |
| 0x-E-xxxxxxx     | 3758096384 - 4026531839 | Error_Level_CRITICAL | **Critical errors (E)** are related to any internal failure of the device. This includes errors in the sensor head (e.g., laser power level, excessive vibration, firmware version error) or system plug (e.g., failure of EEPROM). It also includes corrupted calculations, e.g., EMC has corrupted calculations of a micro-controller. <br /> Application is stopped. <br /> These errors may be caused by external influences. Sometimes, device or system plug replacement may be necessary. |