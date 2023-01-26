# Allegro_A139x_8channel_mux

Luke Miller 2023

 Library for Allegro A1395, A1393, A1931 Hall effect sensors hooked up to an 8-channel
 multiplexing setup via adapter board onto a EnviroDIY Mayfly v1.1 data logger. The
 library relies on the presence of a PCA9557 I2C expander to toggle the individual
 sleep lines on 8 Allegro A139x hall effect sensors, and then uses the Mayfly's 
 onboard PCA9536 I2C expander to manipulate the address pins on a TMUX1208
 analog multiplexer on the adapter board, to route the voltage output from each A139x 
 sensor to the Mayfly's onboard ADC (via Mayfly's analog input pin A0). 
 
 This library is built to be compatible with the EnviroDIY ModularSensors
 library framework. https://github.com/EnviroDIY/ModularSensors
 

 
