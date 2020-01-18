#requires -Version 5.0
#3309 MASTER SOFTWARE INSTALLER
#VERSION 2020-0.1
#BY NICOLAS MACHADO
param (
    [string[]]$install = @("7Zip","NISuite","WPILib","IDEA","JDK11","CTRE","GRIP")
 )
[Reflection.Assembly]::LoadWithPartialName( "System.IO.Compression.FileSystem")
[Net.ServicePointManager]::SecurityProtocol = "tls12, tls11, tls"
  if (-Not ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole] 'Administrator')) {
   if ([int](Get-CimInstance -Class Win32_OperatingSystem | Select-Object -ExpandProperty BuildNumber) -ge 6000) {
    $CommandLine = $MyInvocation.MyCommand.Path + " -install " + ($install -join ',')
    Write-Output $CommandLine
    Start-Process PowerShell.exe -Verb Runas -Argument $CommandLine
    Exit
   }
  }
$ProgressPreference = 'SilentlyContinue'
Write-Output "
        .dO0O00OOOOkdc.   cOOO0O0OOOOxo,.       .,ldkOOkxl'        .,ldkOOOxo,.
       .oNWWWWWWWWWWWNO, ;0WWWWWWWWWWWWX0l.   ,d0NWWWWWWWWKl.    ,d0NWWWWWWWWXx'
                   ,::;.            .::::'.;::,.        ':::..;::,.        .;::,
                 .oNMWx.           ;0WMKxokWMXl.       .OMMXdxWMNo.       .dWMWo
       .,,,,,,,;lONMNd. .',,,,,,,cxXMWOc;dWMWd.        lNMWxoXMMK;       .dNMMO'
      cXWWWWWWWWMMWk,  'kNWWWWWWWMMMKc.:OXMMO'        ;KMM0,:XMMWk,....;oKWMMK;
     .o00000KKNMMMWo   :O00000KXWMMMO''OMMMX:        .kWMNc .dWMMMNK0KXWMMMMK;
       .......;OMMMx.   ........dNMMKcdWMMWo         oNMWd.  .:kKNNWNNWMMMWk'
              .OMMNc            lNMWxlKMMMK,       .lXMWk.      ..,;lONMWKl.
             'xWMNo.          .lXMWO,cNMMMNl.    .:OWMXd.       .,ckXWWKo.
 ;llllllllodOXMW0c;clllllllooxKWMXd. '0MMMMNOdookKWMXx,  .:oodxOKNMWXkc.
;KMMMMMMMMMMWKx:..kMMMMMMMMMMWXOo'    'xKXWMMMMMWXkl'    lNMMMWNKko:.
oKKKKKKKKK0kl'   ,OKXKKKKKKKOo;.       ..;x0KK0kl,.     .xK0Oxo:.

 ______ _____   _____    _____  ____  ______ _________          __     _____  ______
|  ____|  __ \ / ____|  / ____|/ __ \|  ____|__   __\ \        / /\   |  __ \|  ____|
| |__  | |__) | |      | (___ | |  | | |__     | |   \ \  /\  / /  \  | |__) | |__
|  __| |  _  /| |       \___ \| |  | |  __|    | |    \ \/  \/ / /\ \ |  _  /|  __|
| |    | | \ \| |____   ____) | |__| | |       | |     \  /\  / ____ \| | \ \| |____
|_|    |_|  \_\\_____| |_____/ \____/|_|       |_|      \/  \/_/    \_\_|  \_\______|


           _____ _   _  _____ _______       _      _      ______ _____
          |_   _| \ | |/ ____|__   __|/\   | |    | |    |  ____|  __ \
            | | |  \| | (___    | |  /  \  | |    | |    | |__  | |__) |
            | | | . `` |\___ \   | | / /\ \ | |    | |    |  __| |  _  /
           _| |_| |\  |____) |  | |/ ____ \| |____| |____| |____| | \ \
          |_____|_| \_|_____/   |_/_/    \_\______|______|______|_|  \_\
"
Write-Output "This tool will Install:
"
ForEach($tool in $install) {
  Write-Output "                        $tool"
}
Read-Host 'Press Enter to continue' | Out-Null
Write-Output "All Downloaded files will be placed in $HOME\Downloads\3309Installer\"
If(!(Test-Path "$HOME\Downloads\3309Installer"))
{
  New-Item -Path "$HOME\Downloads\3309Installer" -ItemType directory -Force | Out-Null
}
If($install -contains "7Zip")
{
  If(!(Test-Path "$HOME\Downloads\3309Installer\7zipInstaller.msi") -Or !(Get-FileHash -Path  "$HOME\Downloads\3309Installer\7zipInstaller.msi").Hash.Equals("0F5D4DBBE5E55B7AA31B91E5925ED901FDF46A367491D81381846F05AD54C45E"))
  {
    Write-Output "Downloading 7-Zip...
    "
    Invoke-WebRequest "https://www.7-zip.org/a/7z1900-x64.msi" -UseBasicParsing -OutFile "$HOME\Downloads\3309Installer\7zipInstaller.msi"
  }
}
If($install -contains "IDEA")
{
  If(!(Test-Path "$HOME\Downloads\3309Installer\ideaInstaller.exe") -Or !(Test-Path "$HOME\Downloads\3309Installer\silent.config") -Or
    !(Get-FileHash -Path  "$HOME\Downloads\3309Installer\ideaInstaller.exe").Hash.Equals("22F643BCC3A0ADDCA3A82F1E6334D5EA26D06D97069BFED7401AA22210DE6FD6") -Or
    !(Get-FileHash -Path  "$HOME\Downloads\3309Installer\silent.config").Hash.Equals("4C9B1EEAA100DC0FF6C808675F8FCC72BDEE1CCB1A245C9B78174B80829F8375"))
  {
    Write-Output "Downloading IntelliJ...
    "
    Invoke-WebRequest "https://download.jetbrains.com/idea/silent.config" -UseBasicParsing -OutFile "$HOME\Downloads\3309Installer\silent.config"
    Invoke-WebRequest "https://download.jetbrains.com/idea/ideaIC-2019.3.1.exe" -UseBasicParsing -OutFile "$HOME\Downloads\3309Installer\ideaInstaller.exe"
  }
}
If($install -contains "JDK11")
{
  If(!(Test-Path "$HOME\Downloads\3309Installer\javaInstaller.msi") -Or !(Get-FileHash -Path  "$HOME\Downloads\3309Installer\javaInstaller.msi").Hash.Equals("B1B17FB7F64788B6C212BA440DFD2A624A4EA2BFCD89F138634505B26599E0A1"))
  {
    Write-Output "Downloading Java 11...
    "
    Invoke-WebRequest "https://github.com/ojdkbuild/ojdkbuild/releases/download/java-11-openjdk-11.0.5.10-2/java-11-openjdk-jre-11.0.5.10-2.windows.ojdkbuild.x86_64.msi" -UseBasicParsing -OutFile "$HOME\Downloads\3309Installer\javaInstaller.msi"
  }
}
If($install -contains "GRIP")
{
  If(!(Test-Path "$HOME\Downloads\3309Installer\GRIPInstaller.exe") -Or !(Get-FileHash -Path  "$HOME\Downloads\3309Installer\GRIPInstaller.exe").Hash.Equals("F02DC2E59F19323563E49249EDFE19A77569EE6B58717943AC62EC009D8EE213"))
  {
    Write-Output "Downloading GRIP...
    "
    Invoke-WebRequest "https://github.com/WPIRoboticsProjects/GRIP/releases/download/v1.5.2/GRIP-v1.5.2-x64.exe" -UseBasicParsing -OutFile "$HOME\Downloads\3309Installer\GRIPInstaller.exe"
  }
}
If($install -contains "CTRE")
{
  If( -Not ((Test-Path "$HOME\Downloads\3309Installer\CTREInstaller.zip") -And (Get-FileHash -Path  "$HOME\Downloads\3309Installer\CTREInstaller.zip").Hash.Equals("FABB0C58AD88351ADD1DA4D182E0FB18153D4C9F77CD8170DED651968BBEC45E")))
  {
    Write-Output "Downloading CTRE Tools...
    "
    #Use this once we get an official release..
    Invoke-WebRequest "https://github.com/CrossTheRoadElec/Phoenix-Releases/releases/download/v5.17.3.1/CTRE.Phoenix.Framework.v5.17.3.1.zip" -UseBasicParsing -OutFile "$HOME\Downloads\3309Installer\CTREInstaller.zip"
  }
}
If($install -contains "NISuite")
{
  If(!(Test-Path "$HOME\Downloads\3309Installer\NI_FRC2020.iso")  -Or !(Get-FileHash -Path  "$HOME\Downloads\3309Installer\NI_FRC2020.iso").Hash.Equals("63DA2E87BD66C6003DAA2E31283207EECAF2A91DBDAE2B060848AD95AB63C094"))
  {
    Write-Output "Downloading NI Suite...
    "
    Invoke-WebRequest "https://download.ni.com/support/nipkg/products/ni-f/ni-frc-2020-game-tools/19.0/offline/ni-frc-2020-game-tools_19.0.0_offline.iso" -UseBasicParsing -OutFile $HOME\Downloads\3309Installer\NI_FRC2020.iso
  }
}
If($install -contains "WPILib")
{
  If(!(Test-Path "$HOME\Downloads\3309Installer\WPILib.zip") -Or !(Test-Path "$HOME\Downloads\3309Installer\NDP462.exe") -Or !(Get-FileHash -Path  "$HOME\Downloads\3309Installer\WPILib.zip").Hash.Equals("15EADF3639558677A7E986E51E53D9B5EEBE079606177A1C0D9FB4C84A80CEB3") -Or !(Get-FileHash -Path  "$HOME\Downloads\3309Installer\NDP462.exe").Hash.Equals("28886593E3B32F018241A4C0B745E564526DBB3295CB2635944E3A393F4278D4"))
  {
    Write-Output "Downloading WPILib Tools...
    "
    Invoke-WebRequest "http://go.microsoft.com/fwlink/?linkid=780600" -UseBasicParsing -OutFile "$HOME\Downloads\3309Installer\NDP462.exe"
    Invoke-WebRequest "https://github.com/wpilibsuite/allwpilib/releases/download/v2020.1.2/WPILibInstaller_Windows64-2020.1.2.zip" -UseBasicParsing -OutFile "$HOME\Downloads\3309Installer\WPILib.zip"
  }
}
If($install -contains "7Zip")
{
  Write-Output "Installing 7-Zip...
  "
  Start-Process msiexec.exe -ArgumentList  "/i $HOME\Downloads\3309Installer\7zipInstaller.msi /q INSTALLDIR=`"C:\Program Files\7-Zip`"" -Wait
}
If($install -contains "JDK11")
{
  Write-Output "Installing Java 11...
  "
  Start-Process msiexec.exe -Wait -ArgumentList '/I $HOME\Downloads\3309Installer\javaInstaller.msi /quiet'
}
If($install -contains "IDEA")
{
  Write-Output "Installing IntelliJ IDEA...
  "
  Start-Process "$HOME\Downloads\3309Installer\ideaInstaller.exe" -Wait -ArgumentList "/S /CONFIG=$HOME\Downloads\3309Installer\silent.config"
}
If($install -contains "CTRE")
{
  Write-Output "Installing CTRE Tools...
  "
  $zip = [System.IO.Compression.ZipFile]::OpenRead("$HOME\Downloads\3309Installer\CTREInstaller.zip")
  Expand-Archive "$HOME\Downloads\3309Installer\CTREInstaller.zip" -DestinationPath  "$HOME\Downloads\3309Installer\" -Force
  $exepath = "$HOME\Downloads\3309Installer\"+$zip.Entries[0].Name
  Start-Process $exepath -Wait -ArgumentList '/S'
}
If($install -contains "GRIP")
{
  Write-Output "Installing GRIP...
  "
  Start-Process "$HOME\Downloads\3309Installer\GRIPInstaller.exe" -Wait -ArgumentList '/SILENT'
}
If($install -contains "NISuite")
{
  Write-Output "Installing NI Suite...
  "
  $volume = (Mount-DiskImage -ImagePath $HOME\Downloads\3309Installer\NI_FRC2020.iso -PassThru | Get-Volume)
  Start-Process ($volume.DriveLetter + ":\Install.exe") -Wait -ArgumentList @('install','ni-frc-2020-labview-2019-update','--accept-eulas','--progress-only','--prevent-reboot')
}
If($install -contains "WPILib")
{
  Write-Output "Installing WPILib Tools... (This one needs to be manually run. Blame Thad.)
  "
  Start-Process "$HOME\Downloads\3309Installer\NDP462.exe" -Wait -ArgumentList '/q'
  $zip = [System.IO.Compression.ZipFile]::OpenRead("$HOME\Downloads\3309Installer\WPILib.zip")
  Expand-Archive "$HOME\Downloads\3309Installer\WPILib.zip" -DestinationPath  "$HOME\Downloads\3309Installer\" -Force
  $exepath = "$HOME\Downloads\3309Installer\"+$zip.Entries[0].Name
  Start-Process $exepath -Wait -ArgumentList '/S'
}
Write-Output "Setup Complete. Good Luck In The Season Ahead!
"
Write-Output "
            */***#(                                                                 (##
           (*****###%                                                              #####(
           /****(######                                                          ##########
          (****/#########                                                      ##############
          ******###########                                                  ##################
          *****/#############&                                             ######################
          *****################%                                         ##########################
         //***/##################&                                     (############/**/#############&
         ***/*(####################(               &((******(/&      #############/******/#############&
        /*****#########//#############       (*******************/*(############//*********/#############(
        ****//#########***/############( //**********************(############(**/*.    .*/***##############
        *****(#########  ***/#############*/****/*****,,  ,,***/############(****/        **/**/#############&
       *****/##########    ****(############*,               %############(/****             */**/#############(
       *****/#########       ***/(############             ############******(               *****(###############
      /***//(########(         **///############%         #############*********/                *****(#############
      *****/#########            **///############&     #############/****    */*(                 ***/*(#############
      /***//#########            ****/*/############& #############/****       /**                   ***//#############
     %*****#########(            *   ****/###########/##############**           *                     *#############/*
     *****/#########   (               ***//(########///(#############           ,                    %############/*//
     *****(#########   /                 *****#######*/*//(#############                  (         #############//*//*
    (****/#########(   (%           (***//*/###########/*/**(#############                *       #############(*/***
    *****/#########    (*           /**/#################****//#############             **     ###########(******
   %/****/#########     **           .####################(*****/############           **(   %############(***/*
   ******#########   (*****(            (############(       ***///#######             ***   #############****/
   *****/#########*****/**/**              /#####(             //*///##              (***  #############***/*
   ****/(########//**/(###//***                                                    ***** #############/**/*
  *****/#########/#########/*****%                                               /*****#############/**/*
  *****/#####################//*****(                                        &*****///############/****
 //****########################*********/&                               (*******/*(############*****
 ****//###################(      /********//**(&                   %/*//*******//#############//**/
 *****(###############/             *******************************************//(##########///*/.
(*****############.                     **********************************    *****#######/*****
*****/#######,                                **********************.           **///(##(*/***
*****/###                                                                         /***/*****
                                                                                    *****/    "
Read-Host -Prompt "Press Enter to exit"