@echo off
echo "Building PDF .."
for /r %%i in (*.docx) do (
echo "--> %%i"
..\..\_installer_\Installer_scripts\Windows\word_to_pdf.vbs %%i
)
