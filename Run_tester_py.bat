@echo off
FOR /f %%F IN ('where python.exe') DO (
 set python_executable=%%F
 goto run_test
)
:run_test
start %python_executable% Scripts/Python/tester.py
