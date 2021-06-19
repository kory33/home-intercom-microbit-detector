REM cd to project directory
cd /D "%~dp0"

set microbit-drive-letter=D

python build.py
copy MICROBIT.hex %microbit-drive-letter%:\MICROBIT.hex
