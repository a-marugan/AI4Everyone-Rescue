## Preamble

The following is a basic guide to make a live bootable USB running Ubuntu. This guide was made due to the limitations of a GPU on the VM machine, whereby external GPUs are needed to run Yolo5 and Detectron2. This guide may also be used as an alternative for using VM since VM tend to run slower compared to live USB.

NOTE: This guide was originally intended for Windows. MacOS would also work but a different guide for installation might be needed.

## Items/Software needed

You can also watch this [VIDEO](https://www.youtube.com/watch?v=g1tZ7X0U-8c) while reading through the guide for a better visual understanding.

- 2x usb sticks (usb c is not required):
    - 1x minimum 8gb usb stick. This one will be the "**SOURCE**" or "**DUMMY**" where the Ubuntu OS will be loaded on first.
    - 1x minimum 32gb (64gb+ preferrably for more space as a premeasure) usb stick. This will be the "**TARGET**" or "**MAIN**" which will be the one you will use at the end. (Since this will be main target usb, having usb 3.0 will boost read-write speed and overall smoothness of the OS)
- A flashing software (e.g. Rufus, Universal USB, Etcher). I used balenaetcher and you can install it using this [link](https://www.balena.io/etcher/)
- Downloaded Ubuntu ISO. You can pick and choose which version of Ubuntu you would like to run. The competition plans on running Ubuntu 18.04 (subject to change from time of writing this document 5/10/2021).
- Last but not least, another device to read this guide from if you are going to be using this on your main computer. (it will require 2 usb ports)

## Setting up and installing Ubuntu on Dummy USB
1. Plug your **DUMMY** usb in and open up BalenaEtcher.
2. Choose the desired ISO and flash the **DUMMY** usb.
3. **DO NOT REMOVE** the **DUMMY** usb and move onto the next phase.

## Installing Ubuntu on MAIN USB
3. Once it is finished flashing, if using Windows 10, hit the Window Key and type in "BIOS" or "Recovery Options" and hit Enter. This should open a menu window called "Recovery".
4. Scroll down to "Advanced Options" and hit restart (please open this guide on a seperate device)
5. If successful, you should be prompted into the BIOS screen. Select bootable USB drive (it may display the usb name) and hit Enter.
6. Once ubuntu is running, plug in your **MAIN** usb, and on the menu screen select "Intall Ubuntu"
7. Select language and keyboard options (usually default is fine) and click "next"
8. The installation step should be "Updates and other software". Select "Normal Installation" and also check "install third-party software". This is to increase chances of recognising hardware and compatibility.
9. **IMPORTANT:** Before you hit "continue" make sure that your **MAIN** usb is **PLUGGED INTO THE COMPUTER**. This is because the the installation will scan all the storages (this includes usb's) on your computer when you hit "continue"
10. If prompted to select "Installation Type", pick "Something else" and hit continue.

## Selecting MAIN USB
***These steps are very important so please do not wipe out your hard-drive by mistake!!!***

11. A selection menu will show up for you to select your target usb. You will need to recognise which usb it is by its size. Usually it is under "/dev/sdc/" and "/dev/sdc1/".
12. **This step is optional** (did not perform this when I tried it myself so enter at your own risk), you can add a partition but clicking on the "+". A create partition window should pop up and then enter your partition size. Following that make sure you choose the "Ex4 Journalling format" if you plan to do so. Make sure you also choose "/" (root) for mounting options.
13. Under the drop-down options for "devices for boot loader installation" make sure you select your **MAIN** usb.
14. **DOUBLE CHECK IT IS THE SELECTED MAIN USB AND NOT YOUR HARDDRIVE.**
15. Click "continue" and follow instructions of normal installation (name, password, timezone etc). Normal installation should start and take some time. Treat yourself to some snacks while you wait.

## Booting from MAIN USB
16. Once the installation is complete, you will be prompted to "Restart Now"
17. Click on it and wait for instructions. You will then be asked to remove the DUMMY USB. **Please be patient**.
18. Press F12 on your computer during the restart/booting setup and you should be able to enter Ubuntu automatically.
19. Then select "Ubuntu" and hit Enter.
20. Sign in and welcome to your very own bootable usb Ubuntu system :)

## Installing NeoFetch for persistence
21. Enter these commands:
```
sudo apt update && sudo apt upgrade -y 

sudo add-apt-repository universe 

sudo add-apt-repository multiverse

sudo apt install neofetch -y
```

22. Once complete, open a new terminal and type
```
neofetch
``` 
to test if it has been successfully installed. It should show up an Ubuntu Logo and also data about your device hardware if successful. Check the GPU section if your GPU is recognised.

For a visual representation, check out this [VIDEO](https://youtu.be/cHF1ByFKtZo?t=300).

## Reformatting the DUMMY USB
Please follow this [LINK](https://tails.boum.org/doc/reset/windows/index.en.html) and reformat your DUMMY USB to re-use it.
