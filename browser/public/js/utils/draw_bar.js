/**
 * Draw a bar chart.
 *
 * Draws a bar chart object on a canvas. 
 *
 * @param {object}  ctx_in          The context of the canvas on which to draw.
 * @param {number}  val             The value to display on the bar chart.
 * @param {number}  maxVal          The maximum value displayable on the bar chart.
 * @param {number}  axisLeft        The x position of the left corner of the x-axis of the bar chart in pixels.
 * @param {number}  axisRight       The x position of the right corner of the x-axis of the bar chart in pixels.
 * @param {number}  maxHeight       The maximum height of the bar chart in pixels.
 * @param {number}  antiWidth       The distance of the edges of the bar chart from each side of the x-axis.
 * @param {string}  fillStyle       The color of the ball's fill.
 * @param {string}  [label=null]    The label to display under the bar chart.
 */
function draw_bar(ctx_in, val, maxVal, axisLeft, axisRight, maxHeight, antiWidth, fillStyle, label=null) {
    ctx_in.strokeStyle = '#000';
    ctx_in.lineWidth = 0;
    ctx_in.beginPath();
    ctx_in.moveTo(axisLeft+antiWidth,maxHeight);
    ctx_in.lineTo(axisLeft+antiWidth,maxHeight-maxHeight*val/maxVal);
    ctx_in.lineTo(axisRight-antiWidth,maxHeight-maxHeight*val/maxVal);
    ctx_in.lineTo(axisRight-antiWidth,maxHeight);
    ctx_in.lineTo(axisLeft+antiWidth,maxHeight);
    ctx_in.fillStyle = fillStyle;
    ctx_in.fill();

    ctx_in.strokeStyle = '#FFF';
    ctx_in.lineWidth = 5;
    ctx_in.beginPath();
    ctx_in.moveTo(axisLeft,maxHeight);
    ctx_in.lineTo(axisRight,maxHeight);
    ctx_in.stroke();
    ctx_in.strokeStyle = '#000';
    ctx_in.lineWidth = 0;
    ctx_in.closePath();
    if (label!==null) {
        fontSize = parseInt(/([0-9]+)px/g.exec(ctx_in.font)[1]);
        ctx_in.fillText(label,axisLeft+(axisRight-axisLeft)/2-fontSize*label.length/2,maxHeight+1.2*fontSize);
    }
/*    ctx_in.closePath();*/
}

/**
 * Draw a bar chart.
 *
 * Draws a bar chart object on a canvas. 
 *
 * @param {object}  ctx_in                  The context of the canvas on which to draw.
 * @param {number}  x                       The x position of the left-bottom corner of a bar in pixel.
 * @param {number}  y                       The y position of the left-bottom corner of a bar in pixel.
 * @param {number}  width                   The width of the bar in pixel.
 * @param {number}  max_height              The maximum height of the bar chart in pixel.
 * @param {number}  height_percent          The height of this bar in percentage of max_height.
 * @param {string}  fillStyle               The color of the ball's fill.
 * @param {string}  [label=null]            The label to display under the bar chart.
 * @param {string}  [label_align=center]    The alignment of the label, default at center.
 */

 function draw_bar_new(ctx_in, x, y, width, max_height, height_percent, fillStyle, label=null, label_align="center") {
    ctx_in.fillStyle = fillStyle;
    ctx_in.fillRect(x, y, width, -max_height*height_percent);
    // Draw a base line at the bottom of a bar.
    ctx_in.beginPath();
    ctx_in.moveTo(x-0.05*width,y);
    ctx_in.lineTo(x+1.05*width,y);
    ctx_in.strokeStyle = '#FFFFFF';
    ctx_in.lineWidth = 5;
    ctx_in.stroke();

    if(label!==null){
        fontSize = parseInt(/([0-9]+)px/g.exec(ctx_in.font)[1]);
        ctx_in.textAlign = label_align=="center" ? "center" : label_align;
        ctx_in.fillText(label, x+width/2, y+1.2*fontSize);
    }

 }
