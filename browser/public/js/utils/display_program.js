/**
 * Draws icons for position and orientation on a canvas, with the active icon (if any) in color.
 *
 * @param {object}  ctx_in          The context of the canvas on which to draw.
 * @param {number}  x      			The x coordinate for the text.
 * @param {number}  y      			The y coordinate for the text.
 * @param {number}  width 			The width of the icons.
 * @param {string}  pos_bw       	The url of the position icon in black and white.
 * @param {string}  pos_color       The url of the position icon in color.
 * @param {string}  pos_bw       	The url of the position icon in black and white.
 * @param {string}  orien_color     The url of the orientation icon in color.
 * @param {string}  orien_bw        The url of the orientation icon in black and white.
 * @param {string}  status			status = "pos": Only the label and icon for position will be in color.
 									status = "orien": Only the label and icon for orientation will be in color.
 									For any other status, or if parameter is empty, all icons and labels will be in black and white.
 */

function display_program(ctx_in, x, y, program_arr) {

	ctx_in.font = "45px Raleway";
	ctx_in.textAlign = "left";
	ctx_in.fillStyle = "#373737";
    ctx_in.strokeStyle = '#333333';
    ctx_in.lineWidth = 7;
	for (let c=0; c<program_arr.length; c++){
		ctx_in.fillText(program_arr[c], x, y+75*c); 
	}
}