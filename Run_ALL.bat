@echo off

FOR /f %%F IN ('where pythonw.exe') DO (
 set python_executable=%%F
  goto run_GUI
)
:run_GUI
start %python_executable% GUI/Gui_v1_1.py

FOR /f %%F IN ('where python.exe') DO (
 set python_executable=%%F
 goto run_main
)
:run_main
start %python_executable% "Main Computer"/main.py
