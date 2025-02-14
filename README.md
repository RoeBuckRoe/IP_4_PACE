# IP_4_PACE

InvertedPendulum_PACE/Configurations/PACE_IP_App.cfg is application beign developed to run Sam's control model. Stuck at the State 2 implementation, having trouble with the file reader and STM code (C code) intergration. The rest of the configuration files are either legacy files from Jawad's repo or files created to perform tests on the pendulumn kit. InvertedPendulum_Rok file has a modified GAM that is used for state transition between State 2 and 3. 

Immediate steps to take: 

+ Replace Jawads STM Interface code with Rok's
+ Modify the config files to accomodate the STM interface change.
+ Fix the csv error messaging to initiate state 3 - stabilization.
+ Figure out a way to refreash fileReader to go back to the first accelaration value after EOF is reached.
+ A seperate state 2 mode on the stm side is not needed as the pendulumn will be only controlled using accelaration. 
+ Implement State 3 controls - LQR controller. 
