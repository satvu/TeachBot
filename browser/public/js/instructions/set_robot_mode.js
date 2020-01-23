/**
 * Changes the current graphics mode.
 *
 * Sets the graphic mode to one of the following:
 * - 'image': Displays an image.
 * - 'video': Plays a video.
 * - 'canvas': Displays the canvas and draws objects on it from the drawings array.
 * - 'multiple choice': Displays the multiple choice graphic with custom options.
 * - 
 *
 * @param {object}  instr            A parameterized instruction imported from the JSON file.
 * @param {object}  instructionAddr  The address of the current instruction.
 */
Module.prototype.set_robot_mode = function(instr, instructionAddr) {
	checkInstruction(instr, ['mode'], instructionAddr);
	if (VERBOSE) console.log(`Entering robot mode: ${instr.mode}`);

	this.robot_mode = instr;

	this.SetRobotModeSrv.callService(new ROSLIB.ServiceRequest(instr), result => {
		if (VERBOSE) console.log('Success')
	});
};