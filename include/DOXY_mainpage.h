/*!
 *  @mainpage Endovascular Sugery MainPage
 *  @brief This is program is part of Taha Abbasi-Hashemi's Thesis at Ryerson University
 *  @author Taha Abbasi-Hashemi
 *
 *  @section Introduction
 *      This is a program to control movement of endovascular surgical device while simotainously simulating movement and determining haptic feedback and identiftying optimal movement.
 *
 *      A catheter has three different degrees of freedom meaning it has 27 possible movements for each iteration.
 *      For the purpose of this only nine movements are considered for the simulation.
 *      Each degree of freedom can increase or decrease its Q value, so each degree of freedom has three possible motions it can take for each iteration.
 *      For the purpose of this experiment the only motions considered are the 9 where only 1 degree of freedom is taken at once, they will then be added together at the end providing one of the other 18 options.
 *      
 *
 *  @section Dependencies
 *      In order to compile the program the requirements are 
 *      1. Cmake
 *      2. g++  
 *
 *      In order to run the program the requirements are
 *      1. Boost    -> USB Serial communication
 *      2. VTK      -> Vizualization
 *      3. Eigen    -> Matrix
 * 
 *  @section    Additional-Requirements
 *      The simulation and vizualization can run without the need for haptic feedback or control of the actual endovascular surgical device. 
 *      In order to run the haptic feedback the progam requires addition hardware and software are required. 
 *      The additional hardware is for the haptic deveice is an Arduino and the hardware developed by Taha Abbasi-Hashemi with the assistance of Dr.Kourosh Zarenia.
 *      The haptic feedback device is able to provide haptic feedback to the user and take haptic input from the user and transfer it to the vizualization and simluation software.
 *
 *      The endovascular surgical device used for opperation of practice and real surgery is developed by Dr.Farrokh Sharifi. 
 *      This device is able to move within the vascular system of a given patient. 
 *  
 *  @section Simulation
 *      @subsection Keyboard-Controls
 *          The keyboard can be used to run a simulation and be used to move the system when the haptic device is not present. \n
 *          j. This decreases the Q value of the bending \n
 *          l. This increases the Q value of the bending \n
 *          m. This decreases the Q value of the rotation \n
 *          n. This increasse the Q value of the rotation \n
 *          k. This decreases the Q value of the translation \n
 *          h. This increases the Q value of the translation \n
 *          u. This runs the simulation \n
 *          enter. This shows the catheter body and the aorta \n
 *
 *      @subsection mouse-control
 *          The mouse can be used to navigate the visual system it can be used zoom, move and rotate the camera.
 *          The zoom feature works with the mouse wheel, movement works if the ctrl key is pressed, if no key is pressed the mouse will rotate the camera.
 *
 */
