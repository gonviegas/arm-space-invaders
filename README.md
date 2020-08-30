# ARM - C 
## Space Invaders 
Two-dimensional shooter game in which the player controls a laser cannon by moving it horizontally across the bottom of the screen and firing at descending aliens. The aim is to defeat five rows of eleven aliens—some versions feature different numbers—that move horizontally back and forth across the screen as they advance toward the bottom of the screen. The two  discovery boards are used to implement the famous Atari game “Space Invaders”. In this 1 + 1 game the two players collaborate to defeat the invaders. Each player uses its own board both as user interface and analog joystick controller.  Both boards should show the graphic interface and the same game state, but each player controls a single ship. Bidirectional communications must therefore be implemented in the project, so that the actions of both players are visible synchronously on both screens. The project may be implemented using either serial port, SPI or the I2C protocols. Both board should show the same graphics and neither game should start without the other board being present.


### Guidelines

The full functioning system should be able to communicate bidirectionally between two Discovery boards with one of the serial communications protocols. Both boards should show the graphical game development but only one simulates the game. The master board executes the game and transfers operational data to the other - the slave board.  The user interface is the LCD touch sensor of both boards.

The general requirements are as follows:

 - The applications must be implemented in C language using the STM32F769-Discovery board;
 - The program must compile without any warnings;
 - The code should be developed using the FreeRTOS kernel;
 - Data should be saved to an SD Card;
 - Execution must not cause runtime errors.


![alt text](https://gitlab.com/gonv/arm-space-invaders/-/blob/master/ARM-1.png "Architecture")
