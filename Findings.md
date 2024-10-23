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
**11. Photo Interrupter**

V connects to power, G connects to GND, and S is the out pin that is connected to PA0 configured to ADC1_IN1. When no objects are detected, the value returned is low, otherwise it returns the max value possible given the voltage.

```
HAL_ADC_Start (&hadc1);
HAL_ADC_PollForConversion (&hadc1, 100);
raw = (double) HAL_ADC_GetValue (&hadc1);
```
---
**14. Digital Tilt Sensor**

S, or the out pin, is connected to PA0 configured to ADC1_IN1. Only checks if the sensor is angled, not at the angle of the sensor. 

```
HAL_ADC_Start (&hadc1);
HAL_ADC_PollForConversion (&hadc1, 100);
raw = (double) HAL_ADC_GetValue (&hadc1);
```
---
**15. Capacitive Touch Sensor**

S is connected to a pin configured to be pulled up. Touching both sides of the sensor work, even though only the top part is decorated.

```
if (HAL_GPIO_ReadPin (CTB_GPIO_Port, CTB_Pin)) {
	...
}
```
---
**17. Reed Switch Module**

S is connected to PA0 configured to ADC1_IN1. If no magnet is detected, then PA0 is High, and and Low if otherwise.

```
HAL_ADC_Start (&hadc1);
HAL_ADC_PollForConversion (&hadc1, 100);
raw = (double) HAL_ADC_GetValue (&hadc1);
```
---
**18. PIR Motion Sensor**

S is connected to PA0 configured to ADC1_IN1. If no movement is detected, output is Low, else it is high. Can cover my entire room. 

```
HAL_ADC_Start (&hadc1);
HAL_ADC_PollForConversion (&hadc1, 100);
raw = (double) HAL_ADC_GetValue (&hadc1);
```
---
**20. Analog Rotation Sensor**

S is connected to PA0 configured to ADC1_IN1. The sensor turned all the way to the left returns Low, and all the way to the right returns High. An LED connected to PA4 configured to DAC_OUT1 changes brightness relative amount of turn in the sensor. Because my LED's have certain thresholds, after which they stop getting brighter or dimmer, the following code limits the value returned to an LED-operable range.

The factor of multiplication was derived from dividing the range of possible return values of the sensor, 0 or Low, to 4095, or High (5V), by the range of LED-operables values, 2960 to 3240.

```
HAL_ADC_Start (&hadc1);
HAL_ADC_PollForConversion (&hadc1, 100);
raw = ((double) HAL_ADC_GetValue (&hadc1) * 0.0683761) + 2960;
HAL_DAC_SetValue (&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, raw);
```
---
**21. Photoresistor Sensor**

S is connected to PA0 configured to ADC1_IN1. The more light there is, the higher the voltage returned. An LED connected to PA4 configured to DAC_OUT1 changes brightness relative to the amount of light in the environment.

```
HAL_ADC_Start (&hadc1);
HAL_ADC_PollForConversion (&hadc1, 100);
raw = (double) HAL_ADC_GetValue (&hadc1);
HAL_DAC_SetValue (&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, raw);
```
---
**22. Analog Sound Sensor**

S is connected to PA0 configured to ADC1_IN1. The quieter the area, the lower the voltage. An LED connected to PA4 configured to DAC_OUT1 changes brightness relative amount of noise being picked up by microphone. Because my LED's have certain thresholds, after which they stop getting brighter or dimmer, the following code limits the value returned to an LED-operable range. Also, there is a lot of noise in the out voltage, so the LED doesn't shine consistently, even when continuous noise is picked up.

The factor of multiplication was derived from dividing the range of possible return values of the sensor, 0 or Low, to 4095, or High (5V), by the range of LED-operables values, 2960 to 3240.

```
HAL_ADC_Start (&hadc1);
HAL_ADC_PollForConversion (&hadc1, 100);
raw = ((double) HAL_ADC_GetValue (&hadc1) * 0.0683761) + 2960;
HAL_DAC_SetValue (&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, raw);
```
---
**23. Water Sensor**

S, or the out pin, is connected to PA0 configured to ADC1_IN1. Only checks if there is water present on the exposed wires, and not how much of the wires are submerged in water. 

```
HAL_ADC_Start (&hadc1);
HAL_ADC_PollForConversion (&hadc1, 100);
raw = (double) HAL_ADC_GetValue (&hadc1);
```
---
**24. Soil Humidity Sensor**

S, or the out pin, is connected to PA0 configured to ADC1_IN1. Checks the amount of humidity in the soil. More humidity, less resistance, higher voltage. 

```
HAL_ADC_Start (&hadc1);
HAL_ADC_PollForConversion (&hadc1, 100);
raw = (double) HAL_ADC_GetValue (&hadc1);
```
---
**29. Voltage Sensor**

S, or the out pin, is connected to PA0 configured to ADC1_IN1. Voltage divider using 30K and 7.5K ohm resistors. No 5V from board is needed, only GND.

S = 7.5 / (30 + 7.5) * VCC

VCC = 37.5 / 7.5 * Vout

VCC = 37.5 / 7.5 * 3.3 = 16.5

Note that calibtration is still needed to get correct readings. Pressing finger on the back of the board where the output pins are gives accurate description. Reason is unknown.

```
HAL_ADC_Start (&hadc1);
HAL_ADC_PollForConversion (&hadc1, 100);
raw = ((double) HAL_ADC_GetValue (&hadc1) / 4095) * 16.5;
```
---
**30. Thin-film Pressure Sensor**

S, or the out pin, is connected to PA0 configured to ADC1_IN1. When pressure is applied, high voltage is read. Note that there is no smooth transisition in voltage when applying pressure.

```
HAL_ADC_Start (&hadc1);
HAL_ADC_PollForConversion (&hadc1, 100);
raw = (double) HAL_ADC_GetValue (&hadc1);
```
---
**31. TEMT6000 Ambient Light Sensor**

S, or the out pin, is connected to PA0 configured to ADC1_IN1. The more light that the sensor detects, the higher the voltage.

```
HAL_ADC_Start (&hadc1);
HAL_ADC_PollForConversion (&hadc1, 100);
raw = (double) HAL_ADC_GetValue (&hadc1);
```
---
**36. Joystick Module**

Y is connected to PC3 configured to ADC1_IN9, X is connected to PA0 configured to ADC1_IN1, B is connected to a no pull input button. To allow multiple channels to be used, Continuous Conversion Mode in ADC1 must be Enabled, and the Number of Conversions must be 2, one for each channel. Each rank must be connected to a channel. A DMA request must be enabled for ADC1 going from peripheral to memory, with the data width being a word.

```
uint32_t raws[2];
uint8_t pressed = 0;
int count = 0;

if (HAL_GPIO_ReadPin (CTB1_GPIO_Port, CTB1_Pin)) {
	if (!pressed) {
		count++;
	  	pressed = 1;
  }
} else {
	pressed = 0;
}

HAL_ADC_Start_DMA(&hadc1, raws, 2);
```
---