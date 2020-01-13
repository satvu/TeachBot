/**
 * Draw a dynamic object.
 *
 * Draws a ball or bar chart whose properties are linked with the robot state. 
 *
 * @param {object}   ctx_in  The context of the canvas on which to draw.
 * @param {number}   x1      The x position of the first guide point in pixels.
 * @param {number}   y1      The y position of the first guide point in pixels.
 * @param {number}   x2      The x position of the second guide point in pixels.
 * @param {number}   y2      The y position of the second guide point in pixels.
 * @param {number}   x3      The x position of the third guide point in pixels.
 * @param {number}   y3      The y position of the third guide point in pixels.
 * @param {boolean}  ccw     Draw counterclockwise, as opposed to clockwise.
 */
Module.prototype.draw = function(instr) {
	checkInstruction(instr, ['shape'], instructionAddr);

	switch (instr.shape) {
		case ball:
			
	}
}