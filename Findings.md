**1. White LED**
   
Simply connect 5V and GND, and the third pin to a configurable pin. Then simply toggle the configurable pin with power. A later project will include a potentionmeter for variable brightness. Sample code shown below.

```
HAL_GPIO_WritePin (LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); 
HAL_Delay (500); 
HAL_GPIO_WritePin (LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); 
HAL_Delay (500);
```
---
**2. RGB LED**

V connects to 5V, and the pins marked R, G, and B corresponds to the different colored LEDs. To be able to set values other than 0 or 255, 3 different DACs are needed, and the STM32 only has 1. A later project will include a potentionmeter for custom colors. Also, connecting the R G B pins to GND equal a value of 255, and connected to power equals a value of 0. 

The following code roughly moves through values 0 and 255. The min and max values depend on the voltage running through the LEDs. The current min and max are for 5V, and for 3V3, the min and max would be smaller, from roughly 500 to 1500.   

```
int color = 1000;
HAL_DAC_Init (&hdac);
HAL_DAC_Start (&hdac, DAC_CHANNEL_1);
...
while (color <= 3000) {
  HAL_DAC_SetValue (&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, color++);
	HAL_Delay(5);
}

while (color >= 1000) {
	HAL_DAC_SetValue (&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, color--);
	HAL_Delay (5);
}
```
---
**3. 3W LED**



1. Analog Temperature Sensor
> The voltage out pin should be connected to an ADC configured pin in the microcontroller. The raw data should be adjusted by some constant to calibrate sensor properly, then divided by 100 to get temperature in C.
