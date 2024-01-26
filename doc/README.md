# Prepare **COINES** documentation 
* Switch to **doc** directory
* Install required packages by running
```
pip install -r requirements.txt
```
* Install GTK [https://github.com/tschoonj/GTK-for-Windows-Runtime-Environment-Installer/releases/tag/2022-01-04]

* Set the WEASYPRINT_DLL_DIRECTORIES system path environment variable to list the folders where the libraries can be found
```
set WEASYPRINT_DLL_DIRECTORIES=C:\GTK3\bin;
```
* Run the command
```
mkdocs build
```
* The documentation file will be stored in site/pdf path.
