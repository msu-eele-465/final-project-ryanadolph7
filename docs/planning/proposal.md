# Final project proposal

- [x] I have reviewed the project guidelines.
- [x] I will be working alone on this project.
- [x] No significant portion of this project will be (or has been) used in other course work.

## Embedded System Description

At a high level, explain in 2-3 sentences what your embedded system will do.  Then, in 1-2 sentences each, explain each input, the process, and each output. This section is the bulk of the proposal.

My embedded system will allow me to detect altitude and pressure from a pololu sensor (https://www.pololu.com/product/2126).  The input will be from a pressure sensor and will communicate via I2C, and the output will be to an LCD screen displaying the current distance. I will also be using 2 microcontrollers (2355 and 2310) to read the values and will communicate via I2C. 

## Hardware Setup

What hardware will you require? Provide a conceptual circuit diagram and/or block diagram to help the reviewers understand your proposal. Be sure to introduce and discuss your figures in the text.

I will need 2 seperate MCU's, a LCD display, and an altitude sensor. All of these devices will communicate through I2C. One MCU will be reading the altitude sensor values, the other will be displaying the values to the LCD display.

## Software overview

Discuss, at a high level, a concept of how your code will work. Include a *high-level* flowchart. This is a high-level concept that should concisely communicate the project's concept.

I will be using C to communicate with 2 different microcontrollers via I2C. The first controller will be reading the sensors values using I2C then also communicating to the second MCU (FR2310) to display the values on an LCD display. 

## Testing Procedure

Briefly describe how you will test and verify that your project is a success. Think about how you will *demo* the final project. If you need extra equipment for the demo, be sure that you can either bring the equipment or convincingly simulate it. For example, if you want to build a system that uses CAN bus to interface with your car, you won't be able to bring your car into Cobleigh for the demo...

I will be testing by just moving the sensor on the stairs in order to read the altitude being detected. 

## Prescaler

Desired Prescaler level: 

- [ ] 100%
- [ ] 95% 
- [ ] 90% 
- [x] 85% 
- [ ] 80% 
- [ ] 75% 

### Prescalar requirements 

**Outline how you meet the requirements for your desired prescalar level**

**The inputs to the system will be:**
1. Digital Altitude and Pressure sensor

**The outputs of the system will be:**
1. LCD Display 
2. 

**The project objective is**

{text – 1 to 2 sentences}

**The new hardware or software modules are:**
1. A pololu Altitude / Pressure sensor
2. I2C communication is available
3. Interrupts to read values 


The Master will be responsible for:

Reading in the values of the altitude sensor and sending I2C communication to slave.

The Slave(s) will be responsible for:

Reading in I2C communication and displaying values on a LCD display. 



### Argument for Desired Prescaler

Consider the guidelines presented in the lecture notes and convince the reviewers that this proposal meets the minimum requirements for your desired prescale level.
It is a non-analog sensor with I2C communication that is able to test pressure and altitude. I have never worked on something like this and believe it is going to be challanging enough for an 85%. 
