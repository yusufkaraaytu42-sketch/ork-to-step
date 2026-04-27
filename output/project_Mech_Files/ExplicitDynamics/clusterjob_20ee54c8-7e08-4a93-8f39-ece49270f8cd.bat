@ECHO OFF
SETLOCAL
CHCP 65001
ECHO Job started on %COMPUTERNAME%
SET /A errno=0
ECHO ClusterSharedDirectory=C:\Users\ekryt\Desktop\ork to ansys - Copy\output\project_Mech_Files\ExplicitDynamics
IF NOT EXIST "C:\Users\ekryt\Desktop\ork to ansys - Copy\output\project_Mech_Files\ExplicitDynamics\." goto NOSTAGINGDIR
ECHO AWP_ROOT261=%AWP_ROOT261%
IF "%AWP_ROOT261%" == "" GOTO NOAWPROOTENV
IF NOT EXIST "%AWP_ROOT261%\." goto NOAWPROOTDIR
ECHO Command=%AWP_ROOT261%/commonfiles/CPython/3_10/winx64/Release/python/python.exe
IF NOT EXIST "%AWP_ROOT261%/commonfiles/CPython/3_10/winx64/Release/python/python.exe" goto NOCOMMAND
ECHO running the commmand
ECHO command: "%AWP_ROOT261%/commonfiles/CPython/3_10/winx64/Release/python/python.exe" -B -E "%AWP_ROOT261%/RSM/Config/scripts/ClusterJobs.py" "C:\Users\ekryt\Desktop\ork to ansys - Copy\output\project_Mech_Files\ExplicitDynamics\control_20ee54c8-7e08-4a93-8f39-ece49270f8cd.rsm"
"%AWP_ROOT261%/commonfiles/CPython/3_10/winx64/Release/python/python.exe" -B -E "%AWP_ROOT261%/RSM/Config/scripts/ClusterJobs.py" "C:\Users\ekryt\Desktop\ork to ansys - Copy\output\project_Mech_Files\ExplicitDynamics\control_20ee54c8-7e08-4a93-8f39-ece49270f8cd.rsm"
IF %ERRORLEVEL% NEQ 0 SET /A errno=%ERRORLEVEL%
GOTO END
:NOAWPROOTENV
ECHO The AWP_ROOT261 environment variable was NOT detected.
ECHO 1000 > "C:\Users\ekryt\Desktop\ork to ansys - Copy\output\project_Mech_Files\ExplicitDynamics\exitcode_20ee54c8-7e08-4a93-8f39-ece49270f8cd.rsmout"
SET /A errno=1000
GOTO END
:NOCOMMAND
ECHO Command was NOT detected on execution host.
ECHO 1007 > "C:\Users\ekryt\Desktop\ork to ansys - Copy\output\project_Mech_Files\ExplicitDynamics\exitcode_20ee54c8-7e08-4a93-8f39-ece49270f8cd.rsmout"
SET /A errno=1007
GOTO END
:NOSTAGINGDIR
ECHO Shared cluster directory does not exist on execution node, make sure it is shared and can be accessed from all nodes.
ECHO 1008 > "C:\Users\ekryt\Desktop\ork to ansys - Copy\output\project_Mech_Files\ExplicitDynamics\exitcode_20ee54c8-7e08-4a93-8f39-ece49270f8cd.rsmout"
SET /A errno=1008
GOTO END
:NOAWPROOTDIR
ECHO AWP_ROOT261 directory does not exist on execution host.
ECHO 1009 > "C:\Users\ekryt\Desktop\ork to ansys - Copy\output\project_Mech_Files\ExplicitDynamics\exitcode_20ee54c8-7e08-4a93-8f39-ece49270f8cd.rsmout"
SET /A errno=1009
GOTO END
:END
EXIT /B %errno%
