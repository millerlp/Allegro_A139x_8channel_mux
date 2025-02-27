# Allegro_A139x_8channel_mux

Luke Miller 2023

 Library for Allegro A1395, A1393, A1931 Hall effect sensors hooked up to an 8-channel
 multiplexing setup via adapter board onto a EnviroDIY Mayfly v1.1 data logger (https://www.envirodiy.org/). The
 library relies on the presence of a PCA9557 I2C expander on the adapter board to toggle the individual
 sleep lines on 8 Allegro A139x hall effect sensors, and then uses the Mayfly's 
 onboard PCA9536 I2C expander to manipulate the address pins on a TMUX1208
 analog multiplexer on the adapter board, to route the voltage output from each A139x 
 sensor to the Mayfly's onboard ADC (via Mayfly's analog input pin A0). 
 
 This library is built to be compatible with the EnviroDIY ModularSensors
 library framework. https://github.com/EnviroDIY/ModularSensors

 Because this library is effectively just routing an analog voltage signal from one of the 8 addresses
 on the TMUX1208 analog multiplexer to the analog input pin on the Mayfly, you could feed any analog
 signal (in the 0-3.3V range) to the Mayfly with this library, with or without the use of the sleep lines
 used to put the A139x Hall sensors in and out of sleep mode. 
 

 
