# Velocity-controller-mobile-robot

This code is for the implementation of a velocity controller for a differential drive mobile robot using the Arduino board and Motor Shield v2.3. The feedback is done through Hall Effect sensors; magnets are placed around the wheel and its angular velocity can be computed.   A PI controller with roll-off is implemented using the Bilinear Transformation. Also a reference command Prefilter is used to reduce overshoot.   

Two libraries are needed. One is the Encoder library and the second is for the motor shield. You can go to https://www.arduino.cc/en/hacking/libraries to check how to install these libraries. 


