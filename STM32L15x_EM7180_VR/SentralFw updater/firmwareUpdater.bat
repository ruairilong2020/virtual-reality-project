@echo off
COLOR 17
ECHO.
 
 
ECHO ...............................................

ECHO DFU utility for updating firmware

ECHO ...............................................

ECHO.

ECHO 1 - EXIT

ECHO 2 - Update sentral firmware

ECHO 3 - Update MCU firmware

  
ECHO.

 
SET /P M=Type 1, 2 then press ENTER: 

IF %M%==1 GOTO sel1
IF %M%==2 GOTO sel2
IF %M%==3 GOTO sel3

 
 
ECHO.
ECHO Invalid selection.
pause
exit /b 0

 
 
 
:sel1

exit /b 0



:sel2
set /p fw_file_name="Enter sentral firmware file name: " %=%

ECHO.
FOR %%? IN (%fw_file_name%) DO (
ECHO Last-Modified Date   : %%~t?
)
pause
 
dfu-util-static.exe -a0 -d 0x413c:0x8197 -s 0x08028d00 -D %fw_file_name% -R 
pause
exit /b 0

:sel3
set /p fw_file_name="Enter dfu file name: " %=%

ECHO.
FOR %%? IN (%fw_file_name%) DO (
ECHO Last-Modified Date   : %%~t?
)
pause
 
dfu-util-static.exe -a0 -d 0x413c:0x8197 -D %fw_file_name% -R
pause
exit /b 0

