@echo off

FOR /f %%F IN ('where python.exe') DO (
 set python_executable=%%F
  goto run_injector
)
:run_injector
start %python_executable% Scripts/Python/injector.py