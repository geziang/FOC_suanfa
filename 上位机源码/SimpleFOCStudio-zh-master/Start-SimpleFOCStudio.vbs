Option Explicit

Dim shell, fso, root, pythonPath, entryPoint, qtRoot, qtPlugins, qtPlatforms, command
Set shell = CreateObject("WScript.Shell")
Set fso = CreateObject("Scripting.FileSystemObject")

root = fso.GetParentFolderName(WScript.ScriptFullName)
pythonPath = fso.BuildPath(root, ".venv\Scripts\python.exe")
entryPoint = fso.BuildPath(root, "simpleFOCStudio.py")
qtRoot = fso.BuildPath(root, ".venv\Lib\site-packages\PyQt5\Qt5")
qtPlugins = fso.BuildPath(qtRoot, "plugins")
qtPlatforms = fso.BuildPath(qtPlugins, "platforms")

If Not fso.FileExists(pythonPath) Then
    shell.Run "powershell.exe -NoProfile -ExecutionPolicy Bypass -File " & Chr(34) & fso.BuildPath(root, "配置SimpleFOCStudio环境.ps1") & Chr(34), 1, True
End If

command = Chr(34) & pythonPath & Chr(34) & " -u " & Chr(34) & entryPoint & Chr(34)

shell.CurrentDirectory = root
shell.Environment("Process")("QT_PLUGIN_PATH") = qtPlugins
shell.Environment("Process")("QT_QPA_PLATFORM_PLUGIN_PATH") = qtPlatforms
shell.Environment("Process")("PATH") = qtRoot & ";" & qtPlatforms & ";" & shell.Environment("Process")("PATH")
shell.Run command, 1, False
