@echo off
IF %1.==. GOTO Build
IF %1==clean (
	GOTO Clean
) ELSE (
	GOTO Build
)
:Build
	REM cl  /Ox /O2 zjtag.c ftdixx.c libusb.c 

	call "%VS100COMNTOOLS%vsvars32.bat"
	
	"%VCINSTALLDIR%bin\cl.exe" /Fe..\ /Ox /O2 zjtag.c ftdixx.c j-link.c libusb.c busbasp.c stmhid.c
	
	set /P clean_closet=Do you want to clean build files [Y/n]?
	if /I "%clean_closet%" NEQ "N" goto :Clean
GOTO Done
:Clean
	del *.obj -y
GOTO Done
:Done
echo on
@pause