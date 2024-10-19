1. White LED 

> Simply connect 5V and GND, and the third pin to a configurable pin. Then simply toggle the configurable pin with power. A later project will include a potentionmeter for variable brightness. Sample code shown below.

```
HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); 
HAL_Delay(500); 
HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); 
HAL_Delay(500);
```
---




1. Analog Temperature Sensor
> The voltage out pin should be connected to an ADC configured pin in the microcontroller. The raw data should be adjusted by some constant to calibrate sensor properly, then divided by 100 to get temperature in C.
