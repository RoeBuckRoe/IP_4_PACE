# IP_4_PACE

Immediate steps to take: 

+ Replace Jawads STM Interface code with Rok's
+ Modify the config files to accomodate the STM interface change.
+ Fix the csv error messaging to initiate state 3 - stabilization.
+ Figure out a way to refreash fileReader to go back to the first accelaration value after EOF is reached.
+ A seperate state 2 mode on the stm side is not needed as the pendulumn will be only controlled using accelaration. 
+ Implement State 3 controls - LQR controller. 
