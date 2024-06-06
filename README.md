# Sound Localization with STM32 - README

## Overview

This project involves the implementation of a sound localization system using an STM32 microcontroller, specifically the NUCLEO-F401RE, paired with the X-NUCLEO-CCA02M1 extension board equipped with MEMS microphones. The goal is to detect and localize sound sources, such as a finger snap, by processing audio signals captured by the microphones.

## Hardware Components

1. **NUCLEO-F401RE Microcontroller**: Acts as the main processing unit.
2. **X-NUCLEO-CCA02M1 Extension Board**: Contains two MEMS microphones for capturing audio signals.
3. **Morpho Pin Connectors**: Used to establish physical connections between the NUCLEO-F401RE and the X-NUCLEO-CCA02M1.

## Software and Configuration

### Communication Protocol

- **Serial Peripheral Interface (SPI)**: Chosen for its high data transfer rates and simplicity. SPI allows the microcontroller to communicate efficiently with the extension board.

### Pin Configuration

- **MIC_CLK_NUCLEO**: PWM signal for the sampling frequency of the microphones.
- **MIC_CLK_x2**: PWM signal for the sampling frequency of the SPI, set at twice the microphone sampling frequency.
- **MIC_PDM12**: PDM signal representing audio captured by the microphones.

### SPI Configuration

- **DMA (Direct Memory Access)**: Configured in circular mode to ensure continuous data flow.
- **SPI2 in Receive-Only Slave Mode**: Used for capturing audio signals.
- **TIM3**: Generates the necessary sampling frequency signals.

### Signal Processing

1. **Decimation**: Reduces the sample rate to manage data processing efficiently. A decimation factor (N) of 8 is used, considering the distance between microphones and the speed of sound.
2. **Filtering**: Low-pass filtering removes unwanted high-frequency components. An FIR filter is used for its consistent delay and easier integration with the decimation process.

### Code Implementation

- **Interrupts**: Implemented to handle SPI buffer completion using the `DMA1_Stream3_IRQHandler` function.
- **FIR Filter and Decimation**: Combined in a single function to avoid aliasing and ensure the output signal is within the desired frequency range.

### Testing and Validation

- **SerialOscilloscope**: Used to visualize output signals.
- **Frequency Sound Generator**: Android app used to generate test sound waves at different frequencies.
- **Threshold Detection**: Finger snaps are detected based on a signal intensity threshold of 10.

## Results

- **Correct Signal Representation**: Verified by feeding known frequencies and observing the output.
- **Finger Snap Detection**: Successfully detected finger snaps based on the set threshold.
- **Position Recognition**: Attempts to use cross-correlation for direction detection were not consistently successful, though the code includes these techniques for further development.

## Conclusion

The project demonstrates a functional sound localization system using the STM32 microcontroller and MEMS microphones. While basic detection and signal processing are successfully implemented, further work is needed to reliably determine the direction of sound sources. This README provides an overview of the hardware setup, software configuration, and the key processes involved in achieving sound localization.

For detailed schematics, code snippets, and further explanations, please refer to the full project documentation in the PDF.