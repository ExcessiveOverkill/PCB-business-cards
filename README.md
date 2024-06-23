# Persistence of vision display business cards

PCB cards designed for opensauce 2024 to give out to people

## Who is this repo for?
Anyone looking to make these cards for themselves or their own business.
It is NOT for people looking to make and sell the cards to others. (non-commercial use only)

## Software
- STM32CubeIDE
- python (numpy, cv2)
- Visual Studio Code


## Assembly
### The boards themselves
I ordered my PCBs from PCB way with .8mm thickness and ENIG coating since it is important to have a good contact surface for the batteries (and it looks nicer than HASL)

### Components
> [!WARNING]
> These exact parts I ordered in bulk from LCSC.
> The parts below are the closest matches I could find from DigiKey, but have not been tested yet

A DigiKey cart containing all of the needed components can be found [here](https://www.digikey.com/en/mylists/list/D8ND2J1IRM)
> [!IMPORTANT]
> The battery clips I used were from LCSC not DigiKey. The ones in the digikey cart have a slightly different footprint and may not fit perfectly.
> The exact clips I used can be found [here](https://www.lcsc.com/product-detail/Button-And-Strip-Battery-Connector_MYOUNG-MY-2032-30_C7525432.html)

Two standard CR2032 batteries are required, I used the cheap amazon ones.

### Soldering
These boards are NOT designed to be hand-solderable.

I highly recommend at least a hotplate.

A stencil could be purchaced from a PCB manufacturer to make placing the solder paste easier, but it is not requred. (PCB and stencil files are in the repo)

The accelerometer is very susceptible to bad solder joints and will result in the card not activating when shaken. I had to resolder many of them to fix this.

## Programming
### Generating the graphics
Run "imageConverter.py", select which images or animations you want to display normally.

A second file select window will open for SECRET images to display. They will be displayed randomly according to the parameters set at the top of the python file.

Simply don't select anything if you do not want a secret image.

Images should be 32 pixel high PNGs, width does not matter, the card will show the entire image.

Once the images are selected, the program will show a demo of what will be displayed then write to a file defining the frame data.

### Programming the card
After the graphics have been saved, you simply build and upload the STM32 project using STM32CubeIDE.



