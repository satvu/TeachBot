/**
 * Draw a rectangle.
 *
 * Draws a rectangle object on a canvas. 
 *
 * @param {object}	ctx_in			The context of the canvas on which to draw.
 * @param {number}	x				The x position of the rectangle's top left corner in pixels.
 * @param {number}	y				The y position of the rectangle's top left corner in pixels.
 * @param {number}	w 				The width of the rectangle in pixels.
 * @param {number}	h 				The height of the rectangle in pixels.
 * @param {bool}	rotate=false	If set to true, the rectangle is rotated 90 degs ccw.
 * @param {string}  label=null		The label to display under the rectangle.
 */
function draw_rectangle(ctx_in,x,y,w,h,rotate=false,label=null) {
    ctx_in.beginPath();
    rotate ? ctx_in.rect(x,y,h,w) : ctx_in.rect(x,y,w,h);
    ctx_in.stroke();

    if (label!==null) {
    	fontSize = parseInt(/([0-9]+)px/g.exec(ctx_in.font)[1]);
    	ctx_in.textAlign = "center";
    	rotate ? ctx_in.fillText(label, x+h/2, y+w+1.2*fontSize) : ctx_in.fillText(label,x+w/2, y+h+1.2*fontSize);
    }
}